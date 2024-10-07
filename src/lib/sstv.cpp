/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2023 Marat Fayzullin <luarvique@gmail.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANDRAS RETZLER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "sstv.hpp"
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <cstdio>

using namespace Csdr;

// Header timings in milliseconds
#define BREAK_OFFSET    (300)
#define LEADER_OFFSET   (BREAK_OFFSET + 10)
#define VIS_OFFSET      (LEADER_OFFSET + 300)
#define HDR_SIZE        (VIS_OFFSET + 30)
#define VIS_BIT_SIZE    (30)
#define HDR_WINDOW_SIZE (15)
#define HDR_STEP        (2)

// Color modes
#define COLOR_RGB       (1)
#define COLOR_GBR       (2)
#define COLOR_YUV       (3)
#define COLOR_BW        (4)

// Decoding states
#define STATE_HEADER    (-3)
#define STATE_VIS       (-2)
#define STATE_SYNC      (-1)
#define STATE_LINE0     (0)

// VIS IDs
#define ID_ROBOT36      (8)
#define ID_ROBOT72      (12)
#define ID_MARTIN2      (40)
#define ID_MARTIN1      (44)
#define ID_SC2_30       (51)
#define ID_SC2_180      (55)
#define ID_SCOTTIE2     (56)
#define ID_SC2_60       (59)
#define ID_SCOTTIE1     (60)
#define ID_SC2_120      (63)
#define ID_AVT90        (68)
#define ID_SCOTTIEDX    (76)
#define ID_PD50         (93)
#define ID_PD120        (95)
#define ID_PD180        (96)
#define ID_PD240        (97)
#define ID_PD160        (98)
#define ID_PD90         (99)

//
// Forward class declarations
//
class Robot36;
class Robot72;
class Martin1;
class Martin2;
class Scottie1;
class Scottie2;
class ScottieDX;
class PD50;
class PD90;
class PD120;
class PD160;
class PD180;
class PD240;
class AVT90;
class SC2_30;
class SC2_60;
class SC2_120;
class SC2_180;

//
// BMP file header
//
typedef struct
{
  unsigned char magic[2];
  unsigned char fileSize[4];
  unsigned char reserved[4];
  unsigned char dataOffset[4];

  unsigned char dibSize[4];
  unsigned char width[4];
  unsigned char height[4];
  unsigned char planes[2];
  unsigned char bitCount[2];
  unsigned char compression[4];
  unsigned char imageSize[4];
  unsigned char xPixelsPerM[4];
  unsigned char yPixelsPerM[4];
  unsigned char colorsUsed[4];
  unsigned char colorsImportant[4];
} BMPHeader;

template <typename T>
SstvDecoder<T>::SstvDecoder(unsigned int sampleRate, unsigned int dbgTime)
: sampleRate(sampleRate),
  curState(STATE_HEADER),
  dbgTime(dbgTime)
{
    // Total sizes and 2msec step
    hdrSize = (HDR_SIZE * sampleRate) / 1000;
    wndSize = (HDR_WINDOW_SIZE * sampleRate) / 1000;
    bitSize = (VIS_BIT_SIZE * sampleRate) / 1000;
    visSize = (8 * VIS_BIT_SIZE * sampleRate) / 1000;
    step    = (HDR_STEP * sampleRate) / 1000;

    // No scanlines or pixels yet
    syncSize  = 0;
    pixelSize = 0;
    halfpSize = 0;

    // Header tone offsets
    lead1_Start = 0;
    break_Start = (BREAK_OFFSET * sampleRate) / 1000;
    lead2_Start = (LEADER_OFFSET * sampleRate) / 1000;
    vis_Start   = (VIS_OFFSET * sampleRate) / 1000;

    // Allocate FFT plan and buffers
    // (wndSize*2 must be large enough for everyting!)
    fftIn     = new float[wndSize*2];
    fftOut    = new fftwf_complex[wndSize*2];
    fftHeader = fftwf_plan_dft_r2c_1d(wndSize, fftIn, fftOut, FFTW_ESTIMATE);
}

template <typename T>
SstvDecoder<T>::~SstvDecoder() {
    if(syncSize)  fftwf_destroy_plan(fftSync);
    if(pixelSize) fftwf_destroy_plan(fftPixel);
    if(halfpSize) fftwf_destroy_plan(fftHalfp);
    fftwf_destroy_plan(fftHeader);

    fftwf_free(fftIn);
    fftwf_free(fftOut);
}

template <typename T>
bool SstvDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return
        (this->reader->available() >= sampleRate*2) &&
        (this->writer->writeable() >= 2*MAX_LINE_WIDTH*3);
}

template <typename T>
void SstvDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    const T *buf = this->reader->getReadPointer();
    unsigned int size = this->reader->available();
    unsigned int j, i, lines;

    // Depending on the current state...
    switch(curState)
    {
        case STATE_HEADER:
            // Do not detect until we have enough samples for SSTV header
            if(size<hdrSize) break;
            // Detect SSTV frame header
            i = findHeader(buf, size);
            // If header detected...
            if(i)
            {
print(" [HEADER]");
                // Decoding VIS next, drop processed input data
                curState = STATE_VIS;
                skipInput(i);
            }
            else
            {
                // Header not found, skip input data
                skipInput(size - hdrSize + step);
            }
            // Done
            break;

        case STATE_VIS:
            // Do not decode until we have enough samples for VIS block
            if(size<visSize) break;
            // Try decoding VIS block
            curMode = decodeVIS(buf, visSize);
            // If succeeded decoding VIS...
            if(curMode)
            {
print(" [VIS %d %dx%d %s]", curMode->ID, curMode->LINE_WIDTH, curMode->LINE_COUNT, curMode->NAME);
                // Receiving scanline next
                curState  = curMode->HAS_START_SYNC? STATE_SYNC : STATE_LINE0;
                lastLineT = msecs(visSize);
                // Delete old FFT plans
                if(syncSize)  fftwf_destroy_plan(fftSync);
                if(pixelSize) fftwf_destroy_plan(fftPixel);
                if(halfpSize) fftwf_destroy_plan(fftHalfp);
                // Compute scanline sync and pixel sizes
                syncSize  = round(curMode->SYNC_PULSE * 1.4 * sampleRate);
                pixelSize = round(curMode->PIXEL_TIME * curMode->WINDOW_FACTOR * sampleRate);
                halfpSize = round(curMode->HALF_PIXEL_TIME * curMode->WINDOW_FACTOR * sampleRate);
                // Regenerate FFT plans
                fftSync   = fftwf_plan_dft_r2c_1d(syncSize, fftIn, fftOut, FFTW_ESTIMATE);
                fftPixel  = fftwf_plan_dft_r2c_1d(pixelSize, fftIn, fftOut, FFTW_ESTIMATE);
                fftHalfp  = fftwf_plan_dft_r2c_1d(halfpSize, fftIn, fftOut, FFTW_ESTIMATE);
                // Clear odd/even component buffer, just in case
                memset(linebuf, 0, sizeof(linebuf));
                // Output BMP file header
                printBmpHeader(curMode);
                // Drop processed input data
                skipInput(visSize);
            }
            else
            {
                // Go back to header detection, skip input data
                finishFrame();
                skipInput(visSize);
            }
            // Done
            break;

        case STATE_SYNC:
            // Do not detect until we have this many
            if(size<syncSize) break;
            // Detect SSTV frame sync
            i = findSync(curMode, buf, size);
            // If sync detected...
            if(i)
            {
                // Receiving scanline next
                curState  = STATE_LINE0;
                lastLineT = msecs(i);
                // Drop processed input data
                skipInput(i);
            }
            // If have not received sync for a while...
            else if(msecs() > lastLineT + round(curMode->SYNC_PULSE*32.0))
            {
                // Go back to header detection, skip all input
                finishFrame();
                skipInput(size);
            }
            else
            {
                // Sync not found, skip some input
                skipInput(size - syncSize);
            }
            // Done
            break;

        default:
            // If invalid state or done with scanlines...
            if(!curMode || (curState<0) || (curState>=curMode->LINE_COUNT))
            {
                // Go back to header detection
                finishFrame();
                break;
            }
            // We will need this many input samples for scanline
            j = round(curMode->LINE_TIME * sampleRate);
            // Do not detect until we have enough samples
            if(size<j*2) break;
            // Try decoding a scanline
            i = decodeLine(curMode, curState, buf, size);
            // If decoding successful...
            if(i)
            {
                // Mark last scanline decoding time
                lastLineT = msecs(i);
                // Drop processed input data
                skipInput(i);
                // Go to the next scanline
                curState += curState>0? curMode->LINE_STEP : 1;
                if(curState>=curMode->LINE_COUNT) finishFrame();
            }
            // If have not decoded a scanline for a while...
            else if(msecs() > lastLineT + round(curMode->LINE_TIME*8.0))
            {
                // Go back to header detection, skip all input
                skipInput(size);
                finishFrame();
            }
            else
            {
                // Scanline not found, draw empty line
                printBmpEmptyLines(curMode, curMode->LINE_STEP);
                // Skip scanline worth of input
                skipInput(j);
                // Go to the next scanline
                curState += curState>0? curMode->LINE_STEP : 1;
                if(curState>=curMode->LINE_COUNT) finishFrame();
            }
            break;
    }

    // Periodically print debug information, if enabled, but
    // only if we are not transferring an image
    unsigned long millis = msecs();
    if(dbgTime && (curState<0) && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }
}

template <typename T>
void SstvDecoder<T>::printBmpHeader(const SSTVMode *mode)
{
    BMPHeader bmp;

    // If there is enough output buffer available...
    if(this->writer->writeable()>=sizeof(bmp))
    {
        unsigned int imageSize = mode->LINE_WIDTH * mode->LINE_COUNT * 3;
        unsigned int fileSize  = imageSize + sizeof(bmp);

        memset(&bmp, 0, sizeof(bmp));

        // Use reserved byte at offset 6 to store SSTV mode ID
        bmp.magic[0]      = 'B';
        bmp.magic[1]      = 'M';
        bmp.fileSize[0]   = fileSize & 0xFF;
        bmp.fileSize[1]   = (fileSize >> 8) & 0xFF;
        bmp.fileSize[2]   = (fileSize >> 16) & 0xFF;
        bmp.fileSize[3]   = (fileSize >> 24) & 0xFF;
        bmp.reserved[0]   = mode->ID;
        bmp.dataOffset[0] = sizeof(bmp);

        bmp.dibSize[0]    = 40;
        bmp.width[0]      = mode->LINE_WIDTH & 0xFF;
        bmp.width[1]      = (mode->LINE_WIDTH >> 8) & 0xFF;
        bmp.width[2]      = (mode->LINE_WIDTH >> 16) & 0xFF;
        bmp.width[3]      = (mode->LINE_WIDTH >> 24) & 0xFF;
        bmp.height[0]     = (-mode->LINE_COUNT) & 0xFF;
        bmp.height[1]     = (-mode->LINE_COUNT >> 8) & 0xFF;
        bmp.height[2]     = (-mode->LINE_COUNT >> 16) & 0xFF;
        bmp.height[3]     = (-mode->LINE_COUNT >> 24) & 0xFF;
        bmp.planes[0]     = 1;
        bmp.bitCount[0]   = 24;
        bmp.imageSize[0]  = imageSize & 0xFF;
        bmp.imageSize[1]  = (imageSize >> 8) & 0xFF;
        bmp.imageSize[2]  = (imageSize >> 16) & 0xFF;
        bmp.imageSize[3]  = (imageSize >> 24) & 0xFF;

        // Place BMP header into the output buffer
        writeData(&bmp, sizeof(bmp));
    }
}

template <typename T>
void SstvDecoder<T>::finishFrame()
{
    if(curMode && (curState>=0))
    {
        if(curState < curMode->LINE_COUNT)
            printBmpEmptyLines(curMode, curMode->LINE_COUNT - curState);
    }

    curState = STATE_HEADER;
    curMode  = 0;
print(" [FINISH-FRAME]");
}

template <typename T>
void SstvDecoder<T>::printBmpEmptyLines(const SSTVMode *mode, unsigned int lines)
{
    unsigned int size = mode->LINE_WIDTH * 3;

    // If there is enough output buffer available...
    if(this->writer->writeable() >= lines*size)
    {
        char buf[size] = {0};
        for(int i=0 ; i<lines ; ++i) writeData(buf, size);
    }
}

template <typename T>
void SstvDecoder<T>::printDebug()
{
    // TODO: Insert periodic debug printouts here, as needed
    print(" [BUF %d at %dms]", this->reader->available(), msecs());
}

template <typename T>
void SstvDecoder<T>::print(const char *format, ...)
{
    char buf[256];

    va_list args;
    va_start(args, format);
    vsprintf(buf, format, args);
    printString(buf);
    va_end(args);
}

template <typename T>
void SstvDecoder<T>::printString(const char *buf)
{
#if 0
    // @@@ Enable to dump log into a file
    {
        FILE *F = fopen("/tmp/sstv-decoder.log", "a");
        if(F)
        {
            fprintf(F, "%08X: %s\n", (unsigned int)((unsigned long)this),buf);
            fclose(F);
        }
    }
#endif

    // If we are in debug mode, and not outputting an image, print
    if(dbgTime && (curState<0)) writeData(buf, strlen(buf));
}

template <typename T>
int SstvDecoder<T>::fftPeakFreq(fftwf_plan fft, const float *buf, unsigned int size)
{
    unsigned int xMax, j;

    // Make sure the size makes sense
    if(size<4) return(0);

    // Multiply by Hann window
    double CONST_2PI_BY_SIZE = 2.0 * 3.1415926525 / (size - 1);
    for(j=0 ; j<size ; ++j)
        fftIn[j] = buf[j] * (0.5 - 0.5 * cos(CONST_2PI_BY_SIZE * j));

    // Compute FFT
    fftwf_execute(fft);

    // Go to magnitudes, find highest magnitude bin
    // Ignore top FFT bins (Scottie does not like these)
    for(j=0, xMax=0 ; j<size/2 ; ++j)
    {
        fftIn[j] = fftOut[j][0]*fftOut[j][0] + fftOut[j][1]*fftOut[j][1];
        if(fftIn[j]>fftIn[xMax]) xMax=j;
    }

    // Interpolate peak frequency
    double vNext = fftIn[xMax<size-1? xMax+1 : size-1];
    double vPrev = fftIn[xMax>0? xMax-1:0];
    double v     = vPrev + fftIn[xMax] + vNext;

    // Can't have all three at 0
    if(v<1.0E-64) return(0);

    // Return frequency
    return(((vNext-vPrev)/v + xMax) * sampleRate / size);
}

template <typename T>
unsigned int SstvDecoder<T>::findHeader(const float *buf, unsigned int size)
{
    // Must have enough samples
    if(hdrSize>size) return(0);

    // Check buffer for the header, every 2 milliseconds
    for(unsigned int j=0 ; j<=size-hdrSize ; j+=step)
    {
        // Go to the next location if any of these checks fail
        if(abs(fftPeakFreq(fftHeader, buf + j + lead1_Start, wndSize) - 1900) >= 50) continue;
        if(abs(fftPeakFreq(fftHeader, buf + j + break_Start, wndSize) - 1200) >= 50) continue;
        if(abs(fftPeakFreq(fftHeader, buf + j + lead2_Start, wndSize) - 1900) >= 50) continue;
        if(abs(fftPeakFreq(fftHeader, buf + j + vis_Start, wndSize) - 1200) >= 50)   continue;

        // Header found
        return(j + hdrSize);
    }

    // Header not found
    return(0);
}

template <typename T>
unsigned int SstvDecoder<T>::findSync(const SSTVMode *mode, const float *buf, unsigned int size)
{
    // Must have enough samples
    if(size<3*syncSize/2) return(0);

    // Search for the sync signal
    for(unsigned int j=0 ; j+3*syncSize/2<=size ; ++j)
    {
        // Make sure we are at the sync signal
        if(abs(fftPeakFreq(fftSync, buf + j, syncSize) - 1200) >= 50)
            continue;

        // Look ahead for the end of sync
        if(fftPeakFreq(fftSync, buf + syncSize/2 + j, syncSize) > 1350)
            return(j + syncSize);
    }

    // Not found
    return(0);
}

template <typename T>
void SstvDecoder<T>::skipInput(unsigned int size)
{
    // Do not advance farther than the available data
    unsigned int avail = this->reader->available();
    size = size<avail? size : avail;

    // Advance read pointer
    this->reader->advance(size);

    // Update time
    curSamples += size;
    if(curSamples>=sampleRate)
    {
        unsigned int secs = curSamples/sampleRate;
        curSeconds += secs;
        curSamples -= secs*sampleRate;
    }
}

template <typename T>
unsigned int SstvDecoder<T>::decodeLine(const SSTVMode *mode, unsigned int line, const float *buf, unsigned int size)
{
    // Temporary output buffers
    unsigned char out[mode->CHAN_COUNT][mode->LINE_WIDTH];

    // Scanline and sync pulse size in samples
    unsigned int lineSize = round(mode->LINE_TIME * sampleRate);
    unsigned int syncSize = round(mode->SYNC_PULSE * sampleRate);

    // Must have enough input samples
    if(size < lineSize*2) return(0);

    // Starting sample to look for sync from
    int start0 = round((
        mode->CHAN_OFFSETS[mode->CHAN_SYNC] -
        mode->SYNC_PULSE - mode->SYNC_PORCH) * sampleRate);
    start0 = start0>=0? start0 : 0;

    // Find sync pulse
    int start = start0 + lineSize <= size?
        findSync(mode, buf + start0, lineSize) : 0;

    // If sync found, use it for scanline start, else skip <syncSize>
    // samples left from a previous scanline and assume start there
    if(start) start -= syncSize; else start = syncSize;

    // For each channel...
    for(unsigned int ch=0 ; ch<mode->CHAN_COUNT ; ++ch)
    {
        double pxTime, pxWindow;
        fftwf_plan fftPlan;

        // Robot mode has half-length second/third scans
        if(mode->HAS_HALF_SCAN && (ch>0))
        {
            pxTime   = mode->HALF_PIXEL_TIME;
            pxWindow = halfpSize;
            fftPlan  = fftHalfp;
        }
        else
        {
            pxTime   = mode->PIXEL_TIME;
            pxWindow = pixelSize;
            fftPlan  = fftPixel;
        }

        // Determine center window position
        double centerWindowT = (pxTime * mode->WINDOW_FACTOR) / 2.0;

        // For each pixel in line...
        for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
        {
            double pxPosT = mode->CHAN_OFFSETS[ch] + pxTime*px - centerWindowT;
            int pxPos = start + round(pxPosT * sampleRate);

            // Decode valid pixels, blank non-existant pixels
            out[ch][px] = (pxPos>=0) && (pxPos+pxWindow<=size)?
                luminance(fftPeakFreq(fftPlan, buf + pxPos, pxWindow)) : 0;
        }
    }

    // If there is enough output space available for this scanline...
    if(this->writer->writeable()>=mode->LINE_WIDTH*mode->LINE_STEP)
    {
        unsigned char *outPtr[3];
        for(int j=0 ; j<3 ; j++)
            outPtr[j] = j<mode->CHAN_COUNT? out[j] : 0;

        // R36: This is the only case where two channels are valid
        if((mode->CHAN_COUNT==2) && mode->HAS_ALT_SCAN && (mode->COLOR==COLOR_YUV))
            convertR36(mode, line, outPtr);

        // M1, M2, S1, S2, SDX: GBR color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_GBR))
            convertGBR(mode, line, outPtr);

        // PD90, PD120, ...: Interleaved YUV color
        else if((mode->CHAN_COUNT==3) && mode->HAS_ALT_SCAN && (mode->COLOR==COLOR_YUV))
            convertPD(mode, line, outPtr);

        // R72: YUV color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_YUV))
            convertYUV(mode, line, outPtr);

        // Normal RGB color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_RGB))
            convertRGB(mode, line, outPtr);

        // Unknown mode
        else
            printBmpEmptyLines(mode, 1);
    }

    // Done, return the number of input samples consumed, but leave
    // <syncSize> samples to start looking for sync in
    start += lineSize - syncSize;
    return(start<0? 0 : start<size? start : size);
}

template <typename T>
void SstvDecoder<T>::convertYUV(const SSTVMode *mode, unsigned int line, unsigned char *buf[3])
{
    unsigned char bmp[3 * mode->LINE_WIDTH];
    unsigned char *p;
    unsigned int px;

    for(p=bmp, px=0 ; px<mode->LINE_WIDTH ; ++px)
    {
        unsigned int rgb = yuv2rgb(buf[0][px], buf[2][px], buf[1][px]);
        *p++ = rgb & 0xFF;
        *p++ = (rgb >> 8) & 0xFF;
        *p++ = (rgb >> 16) & 0xFF;
    }

    writeData(bmp, sizeof(bmp));
}

template <typename T>
void SstvDecoder<T>::convertRGB(const SSTVMode *mode, unsigned int line, unsigned char *buf[3])
{
    unsigned char bmp[3 * mode->LINE_WIDTH];
    unsigned char *p;
    unsigned int px;

    for(p=bmp, px=0 ; px<mode->LINE_WIDTH ; ++px)
    {
        *p++ = buf[2][px];
        *p++ = buf[1][px];
        *p++ = buf[0][px];
    }

    writeData(bmp, sizeof(bmp));
}

template <typename T>
void SstvDecoder<T>::convertGBR(const SSTVMode *mode, unsigned int line, unsigned char *buf[3])
{
    unsigned char bmp[3 * mode->LINE_WIDTH];
    unsigned char *p;
    unsigned int px;

    for(p=bmp, px=0 ; px<mode->LINE_WIDTH ; ++px)
    {
        *p++ = buf[1][px];
        *p++ = buf[0][px];
        *p++ = buf[2][px];
    }

    writeData(bmp, sizeof(bmp));
}

template <typename T>
void SstvDecoder<T>::convertR36(const SSTVMode *mode, unsigned int line, unsigned char *buf[3])
{
    unsigned char bmp[3 * mode->LINE_WIDTH];
    unsigned char *p;
    unsigned int px;

    // V in even lines, U in odd lines
    unsigned char *u = line&1? buf[1] : linebuf[0];
    unsigned char *v = line&1? linebuf[0] : buf[1];

    for(p=bmp, px=0 ; px<mode->LINE_WIDTH ; ++px)
    {
        unsigned int rgb = yuv2rgb(buf[0][px], u[px], v[px]);
        *p++ = rgb & 0xFF;
        *p++ = (rgb >> 8) & 0xFF;
        *p++ = (rgb >> 16) & 0xFF;
    }

    // Retain U/V value until the next scanline
    memcpy(linebuf[0], buf[1], mode->LINE_WIDTH);

    writeData(bmp, sizeof(bmp));
}

template <typename T>
void SstvDecoder<T>::convertPD(const SSTVMode *mode, unsigned int line, unsigned char *buf[3])
{
    unsigned char bmp[3 * mode->LINE_WIDTH];
    unsigned char *p = bmp;
    unsigned int px;

    // Use average U/V from two scanlines
    if((line > 0) && (line < mode->LINE_COUNT-1))
    {
        // Draw first scanline
        for(px=0 ; px<mode->LINE_WIDTH ; ++px)
        {
            unsigned char u  = (linebuf[1][px] + buf[2][px]) >> 1;
            unsigned char v  = (linebuf[0][px] + buf[1][px]) >> 1;
            unsigned int rgb = yuv2rgb(buf[0][px], u, v);
            *p++ = rgb & 0xFF;
            *p++ = (rgb >> 8) & 0xFF;
            *p++ = (rgb >> 16) & 0xFF;
        }
    }

    // Write out first scanline, if drawn above
    if(p!=bmp)
    {
        writeData(bmp, sizeof(bmp));
        p = bmp;
    }

    // Draw second scanline
    unsigned char *u = line<mode->LINE_COUNT-1? buf[2] : linebuf[1];
    unsigned char *v = line<mode->LINE_COUNT-1? buf[1] : linebuf[0];
    for(px=0 ; px<mode->LINE_WIDTH ; ++px)
    {
        unsigned int rgb = yuv2rgb(buf[0][px], u[px], v[px]);
        *p++ = rgb & 0xFF;
        *p++ = (rgb >> 8) & 0xFF;
        *p++ = (rgb >> 16) & 0xFF;
    }

    // Retain U/V values until the next scanline
    memcpy(linebuf[0], buf[1], mode->LINE_WIDTH);
    memcpy(linebuf[1], buf[2], mode->LINE_WIDTH);

    writeData(bmp, sizeof(bmp));
}


template <typename T>
unsigned char SstvDecoder<T>::luminance(int freq)
{
    int lum = round((freq - 1500) / 3.1372549);
    return(lum<0? 0 : lum>255? 255 : lum);
}

template <typename T>
unsigned int SstvDecoder<T>::yuv2rgb(unsigned char y, unsigned char u, unsigned char v)
{
    int r = y + ((351 * (v-128)) >> 8);
    int g = y - ((179 * (v-128) + 86 * (u-128)) >> 8);
    int b = y + ((443 * (u-128)) >> 8);

    r = r>255? 255 : r<0? 0 : r;
    g = g>255? 255 : g<0? 0 : g;
    b = b>255? 255 : b<0? 0 : b;

    return((r << 16) | (g << 8) | b);
}

template <typename T>
bool SstvDecoder<T>::writeData(const void *buf, unsigned int size)
{
    // Must have enough writeable space
    if(this->writer->writeable()<size) return(false);

    // Write data then advance pointer
    memcpy(this->writer->getWritePointer(), buf, size);
    this->writer->advance(size);

    // Done
    return(true);
}

class Martin1: public SSTVMode
{
  public:
    Martin1()
    {
      NAME       = "Martin 1";
      ID         = 44;
      COLOR      = COLOR_GBR;
      LINE_WIDTH = 320;
      LINE_COUNT = 256;
      SCAN_TIME  = 0.146432;
      SYNC_PULSE = 0.004862;
      SYNC_PORCH = 0.000572;
      SEP_PULSE  = 0.000572;
      WINDOW_FACTOR = 2.34;

      ComputeTimings();
    }
};

class Martin2: public Martin1
{
  public:
    Martin2()
    {
      NAME       = "Martin 2";
      ID         = 40;
      LINE_WIDTH = 320;
      SCAN_TIME  = 0.073216;
      SYNC_PULSE = 0.004862;
      SYNC_PORCH = 0.000572;
      SEP_PULSE  = 0.000572;
      WINDOW_FACTOR = 4.68;

      ComputeTimings();
    }
};

class Scottie1: public SSTVMode
{
  public:
    Scottie1()
    {
      NAME       = "Scottie 1";
      ID         = 60;
      COLOR      = COLOR_GBR;
      LINE_WIDTH = 320;
      LINE_COUNT = 256;
      SCAN_TIME  = 0.138240;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.001500;
      SEP_PULSE  = 0.001500;
      CHAN_SYNC  = 2;
      WINDOW_FACTOR  = 2.48;
      HAS_START_SYNC = true;

      ComputeTimings();
    }

    void ComputeTimings()
    {
      CHAN_TIME  = SEP_PULSE + SCAN_TIME;
      LINE_TIME  = SYNC_PULSE + CHAN_COUNT*CHAN_TIME;
      PIXEL_TIME = SCAN_TIME / LINE_WIDTH;
      CHAN_OFFSETS[0] = SEP_PULSE;
      CHAN_OFFSETS[1] = SEP_PULSE + CHAN_TIME;
      CHAN_OFFSETS[2] = 2*CHAN_TIME + SYNC_PULSE + SYNC_PORCH;
    }
};

class Scottie2: public Scottie1
{
  public:
    Scottie2()
    {
      NAME = "Scottie 2";
      ID          = 56;
      LINE_WIDTH  = 320;
      SCAN_TIME   = 0.088064;
      SYNC_PULSE  = 0.009000;
      SYNC_PORCH  = 0.001500;
      SEP_PULSE   = 0.001500;
      WINDOW_FACTOR = 3.82;

      ComputeTimings();
    }
};

class ScottieDX: public Scottie2
{
  public:
    ScottieDX()
    {
      NAME       = "Scottie DX";
      ID         = 75;
      LINE_WIDTH = 320;
      SCAN_TIME  = 0.345600;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.001500;
      SEP_PULSE  = 0.001500;
      WINDOW_FACTOR = 0.98;

      ComputeTimings();
    }
};

class Robot36: public SSTVMode
{
  public:
    Robot36()
    {
      NAME       = "Robot 36";
      ID         = 8;
      COLOR      = COLOR_YUV;
      LINE_WIDTH = 320;
      LINE_COUNT = 240;
      SCAN_TIME  = 0.088000;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.003000;
      SEP_PULSE  = 0.004500;
      SEP_PORCH  = 0.001500;
      CHAN_COUNT = 2;
      HAS_HALF_SCAN  = true;
      HAS_ALT_SCAN   = true;
      WINDOW_FACTOR  = 7.70;

      ComputeTimings();
    }

    void ComputeTimings()
    {
      CHAN_TIME       = SEP_PULSE + SCAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      HALF_PIXEL_TIME = SCAN_TIME / 2.0 / LINE_WIDTH;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;

      LINE_TIME       = CHAN_OFFSETS[1] + SCAN_TIME / 2.0;
    }
};

class Robot72: public Robot36
{
  public:
    Robot72()
    {
      NAME       = "Robot 72";
      ID         = 12;
      LINE_WIDTH = 320;
      SCAN_TIME  = 0.138000;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.003000;
      SEP_PULSE  = 0.004500;
      SEP_PORCH  = 0.001500;
      CHAN_COUNT = 3;
      HAS_HALF_SCAN  = true;
      HAS_ALT_SCAN   = false;
      WINDOW_FACTOR  = 4.88;

      ComputeTimings();
    }

    void ComputeTimings()
    {
      CHAN_TIME       = SEP_PULSE + SCAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      HALF_PIXEL_TIME = SCAN_TIME / 2.0 / LINE_WIDTH;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME / 2.0 + SEP_PORCH;

      LINE_TIME       = CHAN_OFFSETS[2] + SCAN_TIME / 2.0;
    }
};

class PD50: public SSTVMode
{
  public:
    PD50()
    {
      NAME       = "PD-50";
      ID         = 93;
      COLOR      = COLOR_YUV;
      LINE_WIDTH = 320;
      LINE_COUNT = 256;
      LINE_STEP  = 2;
      SCAN_TIME  = 0.09152;
      SYNC_PULSE = 0.02;
      SYNC_PORCH = 0.00208;
      SEP_PULSE  = 0.0;
      WINDOW_FACTOR = 3.74;
      HAS_ALT_SCAN  = true;

      ComputeTimings();
    }
};

class PD90: public PD50
{
  public:
    PD90()
    {
      NAME       = "PD-90";
      ID         = 99;
      SCAN_TIME  = 0.17024;
      WINDOW_FACTOR = 2.01;

      ComputeTimings();
    }
};

class PD120: public PD50
{
  public:
    PD120()
    {
      NAME       = "PD-120";
      ID         = 95;
      LINE_WIDTH = 640;
      LINE_COUNT = 496;
      SCAN_TIME  = 0.1216;
      WINDOW_FACTOR = 2.82;

      ComputeTimings();
    }
};

class PD160: public PD50
{
  public:
    PD160()
    {
      NAME       = "PD-160";
      ID         = 98;
      LINE_WIDTH = 512;
      LINE_COUNT = 400;
      SCAN_TIME  = 0.195854;
      WINDOW_FACTOR = 1.75;

      ComputeTimings();
    }
};

class PD180: public PD120
{
  public:
    PD180()
    {
      NAME       = "PD-180";
      ID         = 96;
      SCAN_TIME  = 0.18304;
      WINDOW_FACTOR = 1.87;

      ComputeTimings();
    }
};

class PD240: public PD120
{
  public:
    PD240()
    {
      NAME       = "PD-240";
      ID         = 97;
      SCAN_TIME  = 0.24448;
      WINDOW_FACTOR = 1.40;

      ComputeTimings();
    }
};

class AVT90: public SSTVMode
{
  public:
    AVT90()
    {
      NAME       = "AVT-90";
      ID         = 68;
      COLOR      = COLOR_RGB;
      LINE_WIDTH = 256;
      LINE_COUNT = 240;
      SCAN_TIME  = 0.125;
      SYNC_PULSE = 0.0;
      SYNC_PORCH = 0.0;
      SEP_PULSE  = 0.0;
      WINDOW_FACTOR = 2.74;

      ComputeTimings();
    }
};

class SC2_60: public SSTVMode
{
  public:
    SC2_60()
    {
      NAME       = "Wraase SC2-60";
      ID         = 59;
      COLOR      = COLOR_RGB;
      LINE_WIDTH = 320;
      LINE_COUNT = 256;
      SCAN_TIME  = 0.058;
      SYNC_PULSE = 0.005;
      SYNC_PORCH = 0.0;
      SEP_PULSE  = 0.0;
      WINDOW_FACTOR = 5.91;

      ComputeTimings();

      // Channel #1 (GREEN) is twice the width + 1ms
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + 2*CHAN_TIME + 0.001;
    }
};

class SC2_30: public SC2_60
{
  public:
    SC2_30()
    {
      NAME       = "Wraase SC2-30";
      ID         = 51;
      LINE_COUNT = 128;

      ComputeTimings();

      // Channel #1 (GREEN) is twice the width + 1ms
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + 2*CHAN_TIME + 0.001;
    }
};

class SC2_120: public SC2_60
{
  public:
    SC2_120()
    {
      NAME       = "Wraase SC2-120";
      ID         = 63;
      SCAN_TIME  = 0.117;
      WINDOW_FACTOR = 2.93;

      ComputeTimings();

      // Channel #1 (GREEN) is twice the width + 1ms
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + 2*CHAN_TIME + 0.001;
    }
};

class SC2_180: public SC2_60
{
  public:
    SC2_180()
    {
      NAME       = "Wraase SC2-180";
      ID         = 55;
      SCAN_TIME  = 0.235;
      WINDOW_FACTOR = 1.46;

      // All channels are same length
      ComputeTimings();
    }
};

//
// SSTV modes with parameters
//
static Robot36   MODE_Robot36;
static Robot72   MODE_Robot72;
static Martin1   MODE_Martin1;
static Martin2   MODE_Martin2;
static Scottie1  MODE_Scottie1;
static Scottie2  MODE_Scottie2;
static ScottieDX MODE_ScottieDX;
static PD50      MODE_PD50;
static PD90      MODE_PD90;
static PD120     MODE_PD120;
static PD160     MODE_PD160;
static PD180     MODE_PD180;
static PD240     MODE_PD240;
static AVT90     MODE_AVT90;
static SC2_30    MODE_SC2_30;
static SC2_60    MODE_SC2_60;
static SC2_120   MODE_SC2_120;
static SC2_180   MODE_SC2_180;

template <typename T>
const SSTVMode *SstvDecoder<T>::decodeVIS(const float *buf, unsigned int size)
{
    unsigned int j, i, mode;

    // Verify size
    if(visSize>size) return(0);

    // Decode bits
    for(j=0, i=0, mode=0 ; j<8 ; ++j)
    {
        int peak = fftPeakFreq(fftHeader, buf + bitSize*j, wndSize);
        if(peak<=1200)
        {
            mode |= 1<<j;
            i ^= 1;
        }
    }

print(" [VIS %d %s]", mode&0x7F, i? "BAD":"OK");

    // Check parity (must be even)
    if(i) return(0);

    // Delete parity bit
    mode &= 0x7F;

    // Get mode
    switch(mode)
    {
        case 8:  return(&MODE_Robot36);
        case 12: return(&MODE_Robot72);
        case 40: return(&MODE_Martin2);
        case 44: return(&MODE_Martin1);
        case 51: return(&MODE_SC2_30);
        case 55: return(&MODE_SC2_180);
        case 56: return(&MODE_Scottie2);
        case 59: return(&MODE_SC2_60);
        case 60: return(&MODE_Scottie1);
        case 63: return(&MODE_SC2_120);
        case 68: return(&MODE_AVT90);
        case 76: return(&MODE_ScottieDX);
        case 93: return(&MODE_PD50);
        case 95: return(&MODE_PD120);
        case 96: return(&MODE_PD180);
        case 97: return(&MODE_PD240);
        case 98: return(&MODE_PD160);
        case 99: return(&MODE_PD90);

        case 0:  // Robot 12
        case 1:
        case 2:
        case 3:  // Robot BW8
        case 4:  // Robot 24
        case 5:
        case 6:
        case 7:  // Robot BW12
        case 9:
        case 10:
        case 11: // Robot BW24
        case 13:
        case 14:
        case 15: // Robot BW36
        case 32: // Martin M4
        case 36: // Martin M3
        case 41: // Martin HQ1
        case 42: // Martin HQ2
        case 48: // Scottie 4
        case 52: // Scottie 3
        case 85: // FAX480
        case 90: // FAST FM
        case 100: // Proskan J120
        case 104: // MSCAN TV-1
        case 105: // MSCAN TV-2
        case 113: // Pasokon P3
        case 114: // Pasokon P5
        case 115: // Pasokon P7
            // Not implemented yet
            break;
    }

    // Failed decoding mode
    return(0);
}

namespace Csdr {
    template class SstvDecoder<float>;
}
