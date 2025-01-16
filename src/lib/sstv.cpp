/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2024 Marat Fayzullin <luarvique@gmail.com>
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

// Decoding states
#define STATE_HEADER    (-3)
#define STATE_VIS       (-2)
#define STATE_SYNC      (-1)
#define STATE_LINE0     (0)

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

    // Create and map SSTV mode definitions, by VIS
    memset(modes, 0, sizeof(modes));
    modes[VIS_ROBOT12]   = new Robot12();
    modes[VIS_ROBOT24]   = new Robot24();
    modes[VIS_ROBOT36]   = new Robot36();
    modes[VIS_ROBOT72]   = new Robot72();
    modes[VIS_MARTIN1]   = new Martin1();
    modes[VIS_MARTIN2]   = new Martin2();
    modes[VIS_MARTIN3]   = new Martin3();
    modes[VIS_MARTIN4]   = new Martin4();
    modes[VIS_SCOTTIE1]  = new Scottie1();
    modes[VIS_SCOTTIE2]  = new Scottie2();
    modes[VIS_SCOTTIE3]  = new Scottie3();
    modes[VIS_SCOTTIE4]  = new Scottie4();
    modes[VIS_SCOTTIEDX] = new ScottieDX();
    modes[VIS_PD50]      = new PD50();
    modes[VIS_PD90]      = new PD90();
    modes[VIS_PD120]     = new PD120();
    modes[VIS_PD160]     = new PD160();
    modes[VIS_PD180]     = new PD180();
    modes[VIS_PD240]     = new PD240();
    modes[VIS_PD290]     = new PD290();
    modes[VIS_AVT90]     = new AVT90();
    modes[VIS_SC2_30]    = new SC2_30();
    modes[VIS_SC2_60]    = new SC2_60();
    modes[VIS_SC2_120]   = new SC2_120();
    modes[VIS_SC2_180]   = new SC2_180();

    // Pre-create FFT plans for each known SSTV mode
    for(int j=0 ; j<128 ; ++j)
        if(modes[j]) modes[j]->createPlans(sampleRate, fftOut, fftIn);
}

template <typename T>
SstvDecoder<T>::~SstvDecoder() {
    // Destroy all mode-specific FFT plans
    for(int j=0 ; j<128 ; ++j)
        if(modes[j]) delete modes[j];

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
            if(size<curMode->syncSize) break;
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
                skipInput(size - curMode->syncSize);
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
void SstvDecoder<T>::printBmpHeader(const SstvMode *mode)
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
        bmp.reserved[0]   = mode->ID; // SSTV mode
        bmp.reserved[1]   = 0x73;     // Identifying SSTV image
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
void SstvDecoder<T>::printBmpEmptyLines(const SstvMode *mode, unsigned int lines)
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
unsigned int SstvDecoder<T>::findSync(const SstvMode *mode, const float *buf, unsigned int size)
{
    // Must have enough samples
    if(size<3*mode->syncSize/2) return(0);

    // Search for the sync signal
    for(unsigned int j=0 ; j+3*mode->syncSize/2<=size ; ++j)
    {
        // Make sure we are at the sync signal
        if(abs(fftPeakFreq(mode->fftSync, buf + j, mode->syncSize) - 1200) >= 50)
            continue;

        // Look ahead for the end of sync
        if(fftPeakFreq(mode->fftSync, buf + mode->syncSize/2 + j, mode->syncSize) > 1350)
            return(j + mode->syncSize);
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
unsigned int SstvDecoder<T>::decodeLine(const SstvMode *mode, unsigned int line, const float *buf, unsigned int size)
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

        if((ch>0) && ((mode->ID==VIS_ROBOT36) || (mode->ID==VIS_ROBOT72) || (mode->ID==VIS_ROBOT12) || (mode->ID==VIS_ROBOT24)))
        {
            // Robot modes have half-length second/third channels
            pxTime   = mode->HALF_PIXEL_TIME;
            pxWindow = mode->halfpSize;
            fftPlan  = mode->fftHalfp;
        }
        else if((ch!=1) && ((mode->ID==VIS_SC2_30) || (mode->ID==VIS_SC2_60) || (mode->ID==VIS_SC2_120)))
        {
            // SC2 modes have half-length first/third channels
            pxTime   = mode->HALF_PIXEL_TIME;
            pxWindow = mode->halfpSize;
            fftPlan  = mode->fftHalfp;
        }
        else
        {
            pxTime   = mode->PIXEL_TIME;
            pxWindow = mode->pixelSize;
            fftPlan  = mode->fftPixel;
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

        switch(mode->ID)
        {
            case VIS_PD50:
            case VIS_PD90:
            case VIS_PD120:
            case VIS_PD160:
            case VIS_PD180:
            case VIS_PD240:
            case VIS_PD290:
                // PD90, PD120, ...: Interleaved YUV color
                convertPD(mode, line, outPtr);
                break;

            case VIS_ROBOT12:
            case VIS_ROBOT36:
                // R12, R36: This is the only case where two channels are valid
                convertR36(mode, line, outPtr);
                break;

            default:
                // Normal RGB color
                if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_RGB))
                    convertRGB(mode, line, outPtr);
                // M1, M2, S1, S2, SDX: GBR color
                else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_GBR))
                    convertGBR(mode, line, outPtr);
                // R72: YUV color
                else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_YUV))
                    convertYUV(mode, line, outPtr);
                else
                    printBmpEmptyLines(mode, 1);
                break;
        }
    }

    // Done, return the number of input samples consumed, but leave
    // <syncSize> samples to start looking for sync in
    start += lineSize - syncSize;
    return(start<0? 0 : start<size? start : size);
}

template <typename T>
void SstvDecoder<T>::convertYUV(const SstvMode *mode, unsigned int line, unsigned char *buf[3])
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
void SstvDecoder<T>::convertRGB(const SstvMode *mode, unsigned int line, unsigned char *buf[3])
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
void SstvDecoder<T>::convertGBR(const SstvMode *mode, unsigned int line, unsigned char *buf[3])
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
void SstvDecoder<T>::convertR36(const SstvMode *mode, unsigned int line, unsigned char *buf[3])
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
void SstvDecoder<T>::convertPD(const SstvMode *mode, unsigned int line, unsigned char *buf[3])
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

void SstvMode::destroyPlans()
{
    // Destroy current plans if any
    if(fftSync)  { fftwf_destroy_plan(fftSync);fftSync=0; }
    if(fftPixel) { fftwf_destroy_plan(fftPixel);fftPixel=0; }
    if(fftHalfp) { fftwf_destroy_plan(fftHalfp);fftHalfp=0; }

    // Clear parameters since plans are gone
    sampleRate = 0;
    fftOut     = 0;
    fftIn      = 0;
}

void SstvMode::createPlans(unsigned int rate, fftwf_complex *out, float *in)
{
    // Do not create plans twice
    if((sampleRate==rate) && (fftIn==in) && (fftOut==out)) return;

    // Cache parameters for later checks
    sampleRate = rate;
    fftOut     = out;
    fftIn      = in;

    // Compute scanline sync and pixel sizes
    syncSize  = round(SYNC_PULSE * 1.4 * rate);
    pixelSize = round(PIXEL_TIME * WINDOW_FACTOR * rate);
    halfpSize = round(HALF_PIXEL_TIME * WINDOW_FACTOR * rate);

    // Generate new FFT plans
    fftSync  = fftwf_plan_dft_r2c_1d(syncSize, in, out, FFTW_ESTIMATE);
    fftPixel = fftwf_plan_dft_r2c_1d(pixelSize, in, out, FFTW_ESTIMATE);
    fftHalfp = fftwf_plan_dft_r2c_1d(halfpSize, in, out, FFTW_ESTIMATE);
}

template <typename T>
SstvMode *SstvDecoder<T>::decodeVIS(const float *buf, unsigned int size)
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

    // Delete parity bit and get SSTV mode definition
    return modes[mode & 0x7F];
}

namespace Csdr {
    template class SstvDecoder<float>;
}
