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
#include <cstdio>
#include <climits>

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

// findSync() result
#define NOT_FOUND       (INT_MAX)

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
SstvDecoder<T>::SstvDecoder(unsigned int sampleRate, unsigned int targetFreq)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  curState(STATE_HEADER),
  dbgTime(0)     // Debug printout period (ms)
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

    // Allocate FFT buffers
    fftIn   = new float[wndSize*2];
    fftOut  = new fftwf_complex[wndSize*2];
}

template <typename T>
SstvDecoder<T>::~SstvDecoder() {
    if(buf) { delete[] buf;buf=0; }

    if(fftSize) fftwf_destroy_plan(fftPlan);
    fftwf_free(fftIn);
    fftwf_free(fftOut);
}

template <typename T>
bool SstvDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned int space = maxSize - curSize;
    space = space>0? space : sampleRate / 10;
    return (this->reader->available()>=space) && (this->writer->writeable()>=512);
}

template <typename T>
void SstvDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    unsigned int size = this->reader->available();
    int j, i;

    // If not enough space in the buffer...
    if(!buf || (curSize+size > maxSize))
    {
        // Extend the buffer
        float *newBuf = new float[curSize+size];
        if(!newBuf)
        {
            char msg[256];
            sprintf(msg, "Failed to allocate %d-byte buffer!", curSize+size);
            printString(msg);
            return;
        }

        if(buf)
        {
            memcpy(newBuf, buf, curSize*sizeof(buf[0]));
            delete[] buf;
        }

        buf     = newBuf;
        maxSize = curSize + size;
    }

    // Read new data into the buffer
    for(j=0 ; (j<size) && (curSize<maxSize) ; ++j)
    {
        buf[curSize++] = *(this->reader->getReadPointer());
        this->reader->advance(1);
    }

    // Depending on the current state...
    switch(curState)
    {
        case STATE_HEADER:
            // Do not detect until we have enough samples for SSTV header
//{char buf[128];sprintf(buf," [H %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<hdrSize) break;
            // Detect SSTV frame header
            i = findHeader(buf, curSize);
//if(i){char buf[128];sprintf(buf," [HDR @ %dms] [VIS @ %dms]",msecs(i-hdrSize),msecs(i));printString(buf);}
            // If header detected, decoding VIS next
            if(i) curState = STATE_VIS;
            else
            {
                // Header not found, skip input
                i = curSize>hdrSize? curSize-hdrSize : 0;
            }
            // Drop processed input data
            skipInput(i);
            // Done
            break;

        case STATE_VIS:
            // Do not decode until we have enough samples for VIS record
//{char buf[128];sprintf(buf," [V %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<visSize) break;
            // Try decoding
            curMode = decodeVIS(buf, visSize);
//{char buf[128];sprintf(buf," [MODE='%s']",curMode? curMode->NAME:"???");printString(buf);}
            // If failed, go back to header detection, else wait for scanlines
            curState = !curMode? STATE_HEADER : curMode->HAS_START_SYNC? STATE_SYNC : STATE_LINE0;
            // If succeeded decoding mode...
            if(curMode)
            {
                // Output BMP file header
                printBmpHeader(curMode);
                // Drop decoded input data
                skipInput(visSize);
            }
            // Done
            break;

        case STATE_SYNC:
            // We will need this many input samples for SYNC
            j = round(curMode->SYNC_PULSE * 1.4 * sampleRate);
            // Do not detect until we have this many
//{char buf[128];sprintf(buf," [S %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<j) break;
            // Detect SSTV frame sync
            i = findSync(curMode, buf, curSize, false);
//if(i){char buf[128];sprintf(buf," [SYNC @ %dms]",msecs(i));printString(buf);}
            // If sync detected, decoding image next
            if((i!=NOT_FOUND) && (i>=0)) curState = STATE_LINE0;
            else
            {
                // Sync not found, skip input
                i = curSize>j? curSize-j : 0;
            }
            // Drop processed input data
            skipInput(i);
            // Done
            break;

        default:
            // If invalid state or done with a frame, go back to header detection
            if(!curMode || (curState<0) || (curState>=curMode->LINE_COUNT))
            {
                curState = STATE_HEADER;
                break;
            }
            // We will need this many input samples for scanline
            j = round(curMode->LINE_TIME * sampleRate);
            // Do not detect until we have this many
//{char buf[128];sprintf(buf," [L %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<j) break;
            // Try decoding a scanline
            i = decodeLine(curMode, curState, buf, curSize);
//if(i){char buf[128];sprintf(buf," [LINE%d @ %dms]",curState,msecs());printString(buf);}
            // If decoding successful...
            if(i>0)
            {
                // Drop processed input data
                skipInput(i);
                // Next scanline
                ++curState;
            }
            break;
    }

    // Periodically print debug information, if enabled
    unsigned long millis = msecs();
    if(dbgTime && (millis-lastDebugT >= dbgTime))
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
        unsigned int fileSize  = (mode->LINE_WIDTH * mode->LINE_COUNT * 3) + sizeof(bmp);
        unsigned int imageSize = mode->LINE_WIDTH * mode->LINE_COUNT * 3;

        memset(&bmp, 0, sizeof(bmp));

        bmp.magic[0]      = 'B';
        bmp.magic[1]      = 'M';
        bmp.fileSize[0]   = fileSize & 0xFF;
        bmp.fileSize[1]   = (fileSize >> 8) & 0xFF;
        bmp.fileSize[2]   = (fileSize >> 16) & 0xFF;
        bmp.fileSize[3]   = (fileSize >> 24) & 0xFF;
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
        char *p = (char *)&bmp;
        for(int j=0 ; j<sizeof(bmp) ; ++j)
        {
            *(this->writer->getWritePointer()) = p[j];
            this->writer->advance(1);
        }
    }
}

template <typename T>
void SstvDecoder<T>::printBmpFooter(const SSTVMode *mode, unsigned int linesDone)
{
    if(linesDone<mode->LINE_COUNT)
    {
        unsigned int footerSize =
            (mode->LINE_COUNT - linesDone) * mode->LINE_WIDTH * 3;

        // If there is enough output buffer available...
        if(this->writer->writeable()>=footerSize)
        {
            for(int j=0 ; j<footerSize ; ++j)
            {
                *(this->writer->getWritePointer()) = 0x00;
                this->writer->advance(1);
            }
        }
    }
}

template <typename T>
void SstvDecoder<T>::printDebug()
{
    // @@@ TODO!
}

template <typename T>
void SstvDecoder<T>::printString(const char *buf)
{
    // If there is enough output buffer available...
    if(this->writer->writeable()>=strlen(buf))
    {
        // Place each string character into the output buffer
        for(int j=0 ; buf[j] ; ++j)
        {
            *(this->writer->getWritePointer()) = buf[j];
            this->writer->advance(1);
        }
    }
}

template <typename T>
int SstvDecoder<T>::fftPeakFreq(const float *buf, unsigned int size)
{
  unsigned int xMax, j;

  // Recreate FFT plan as needed
  if(size!=fftSize)
  {
    if(fftSize) fftwf_destroy_plan(fftPlan);
    fftPlan = fftwf_plan_dft_r2c_1d(size, fftIn, fftOut, FFTW_ESTIMATE);
    fftSize = size;
  }

  // Multiply by Hann window
  double CONST_2PI_BY_SIZE = 2.0 * 3.1415926525 / (size - 1);
  for(j=0 ; j<size ; ++j)
    fftIn[j] = buf[j] * (0.5 - 0.5 * cos(CONST_2PI_BY_SIZE * j));

  // Compute FFT
  fftwf_execute(fftPlan);

  // Go to magnitudes, find highest magnitude bin
  for(j=0, xMax=0 ; j<size ; ++j)
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
    if(abs(fftPeakFreq(buf + j + lead1_Start, wndSize) - 1900) >= 50) continue;
    if(abs(fftPeakFreq(buf + j + break_Start, wndSize) - 1200) >= 50) continue;
    if(abs(fftPeakFreq(buf + j + lead2_Start, wndSize) - 1900) >= 50) continue;
    if(abs(fftPeakFreq(buf + j + vis_Start, wndSize) - 1200) >= 50)   continue;

    // Header found
    return(j + hdrSize);
  }

  // Header not found
  return(0);
}

template <typename T>
int SstvDecoder<T>::findSync(const SSTVMode *mode, const float *buf, unsigned int size, bool startOfSync)
{
    unsigned int wndSize = round(mode->SYNC_PULSE * 1.4 * sampleRate);

    // Must have enough samples
    if(wndSize>size) return(0);

    // Search for the sync signal
unsigned int aaa = startOfSync? round(mode->SYNC_PULSE*sampleRate) - wndSize/2 : 0;
    for(unsigned int j=aaa ; j<=size-wndSize ; ++j)
    {
        if(fftPeakFreq(buf + j, wndSize)>1350)
        {
            // This is the end of sync
            j = j + wndSize/2;
            return(startOfSync? (j - round(mode->SYNC_PULSE*sampleRate)) : j);
        }
    }

    // Not found
    return(NOT_FOUND);
}

template <typename T>
void SstvDecoder<T>::skipInput(unsigned int size)
{
    // Make sure we do not skip more than we have
    size = !buf? 0 : size<curSize? size : curSize;

    // If skipping...
    if(size)
    {
        // Move data
        for(int j=0 ; j<curSize-size ; ++j) buf[j] = buf[j+size];
        curSize -= size;

        // Update time
        curSamples += size;
        if(curSamples>=sampleRate)
        {
            unsigned int secs = curSamples/sampleRate;
            curSeconds += secs;
            curSamples -= secs*sampleRate;
        }
    }
}

template <typename T>
unsigned int SstvDecoder<T>::decodeLine(const SSTVMode *mode, unsigned int line, const float *buf, unsigned int size)
{
    // Temporary output buffer
    unsigned char out[mode->CHAN_COUNT][mode->LINE_WIDTH];

    double windowFactor = mode->WINDOW_FACTOR;
    double centerWindowTime = (mode->PIXEL_TIME * windowFactor) / 2.0;
    unsigned int pxWindow = round(centerWindowTime * 2.0 * sampleRate);
    unsigned int done = 0;

    // Start on a scanline
    seqStart = 0;

    // If first line...
    if(!line)
    {
        // Align to the beginning of the previous sync pulse
// @@@ TODO!!!
//        if(mode->CHAN_SYNC>0)
//            seqStart -= round((mode->CHAN_OFFSETS[mode->CHAN_SYNC] + mode->SCAN_TIME) * sampleRate);
    }

    for(unsigned int ch=0 ; ch<mode->CHAN_COUNT ; ++ch)
    {
        // If this is a sync channel...
        if(ch==mode->CHAN_SYNC)
        {
            // Set base offset to the next line
            if((line>0) || (ch>0))
                seqStart += round(mode->LINE_TIME * sampleRate);

            // Find sync
            seqStart = findSync(mode, buf+seqStart, size-seqStart, true);

            // If no sync, do not decode yet
            if(seqStart==NOT_FOUND) return(0);

//{char buf[128];sprintf(buf," [S %dms]",msecs(seqStart));printString(buf);}
        }

        double pxTime = mode->PIXEL_TIME;
        if(mode->HAS_HALF_SCAN)
        {
            // Robot mode has half-length second/third scans
            if(ch>0) pxTime = mode->HALF_PIXEL_TIME;

            centerWindowTime = (pxTime * windowFactor) / 2.0;
            pxWindow         = round(centerWindowTime * 2.0 * sampleRate);
        }

        // For each pixel in line...
        for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
        {
            double chOffset = mode->CHAN_OFFSETS[ch];
            unsigned int pxPos = seqStart + round((chOffset + pxTime*px - centerWindowTime) * sampleRate);
            unsigned int pxEnd = pxPos + pxWindow;

            if(pxEnd>size)
            {
                char msg[256];
                sprintf(msg, "DecodeLine('%s', %d): Reached end of input!", mode->NAME, line);
                printString(msg);
                return(0);
            }

            // Decode pixel
            out[ch][px] = luminance(fftPeakFreq(buf + pxPos, pxWindow));
            done = pxEnd>done? pxEnd : done;
        }
    }

    // If there is enough output space available for this scanline...
    if(this->writer->writeable()>=3*mode->LINE_WIDTH)
    {
        // R36: This is the only case where two channels are valid
        if((mode->CHAN_COUNT==2) && mode->HAS_ALT_SCAN && (mode->COLOR==COLOR_YUV))
        {
            // @@@ TODO!!!
            for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
            {
                *(this->writer->getWritePointer()) = out[0][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = out[1][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = 0;
                this->writer->advance(1);
            }
        }

        // M1, M2, S1, S2, SDX: GBR color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_GBR))
        {
            for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
            {
                *(this->writer->getWritePointer()) = out[1][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = out[0][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = out[2][px];
                this->writer->advance(1);
            }
        }

        // R72: YUV color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_YUV))
        {
            for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
            {
                unsigned int rgb = yuv2rgb(out[0][px], out[1][px], out[2][px]);

                *(this->writer->getWritePointer()) = rgb & 0xFF;
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = (rgb >> 8) & 0xFF;
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = (rgb >> 16) & 0xFF;
                this->writer->advance(1);
            }
        }

        // Normal RGB color
        else if((mode->CHAN_COUNT==3) && (mode->COLOR==COLOR_RGB))
        {
            for(unsigned int px=0 ; px<mode->LINE_WIDTH ; ++px)
            {
                *(this->writer->getWritePointer()) = out[2][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = out[1][px];
                this->writer->advance(1);
                *(this->writer->getWritePointer()) = out[0][px];
                this->writer->advance(1);
            }
        }
    }

    // Done, return the number of input samples consumed
    return(done);
}

template <typename T>
unsigned char SstvDecoder<T>::luminance(unsigned int freq)
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

      CHAN_COUNT = 3;
      CHAN_SYNC  = 0;
      CHAN_TIME  = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME;

      LINE_TIME       = SYNC_PULSE + SYNC_PORCH + 3*CHAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 2.34;

      HAS_START_SYNC  = false;
      HAS_HALF_SCAN   = false;
      HAS_ALT_SCAN    = false;
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

      CHAN_TIME  = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME;

      LINE_TIME       = SYNC_PULSE + SYNC_PORCH + 3*CHAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 4.68;
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

      CHAN_COUNT = 3;
      CHAN_SYNC  = 2;
      CHAN_TIME  = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH + CHAN_TIME;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
      CHAN_OFFSETS[2] = SYNC_PULSE + SYNC_PORCH;

      LINE_TIME       = SYNC_PULSE + 3*CHAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 2.48;

      HAS_START_SYNC  = true;
      HAS_HALF_SCAN   = false;
      HAS_ALT_SCAN    = false;
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

      CHAN_TIME   = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH + CHAN_TIME;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
      CHAN_OFFSETS[2] = SYNC_PULSE + SYNC_PORCH;

      LINE_TIME       = SYNC_PULSE + 3*CHAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 3.82;
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

      CHAN_TIME  = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH + CHAN_TIME;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
      CHAN_OFFSETS[2] = SYNC_PULSE + SYNC_PORCH;

      LINE_TIME       = SYNC_PULSE + 3*CHAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 0.98;
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

      HALF_SCAN_TIME = 0.044000;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.003000;
      SEP_PULSE  = 0.004500;
      SEP_PORCH  = 0.001500;

      CHAN_COUNT = 2;
      CHAN_SYNC  = 0;
      CHAN_TIME  = SEP_PULSE + SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;

      LINE_TIME       = CHAN_OFFSETS[1] + HALF_SCAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      HALF_PIXEL_TIME = HALF_SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 7.70;

      HAS_START_SYNC  = false;
      HAS_HALF_SCAN   = true;
      HAS_ALT_SCAN    = true;
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
      HALF_SCAN_TIME = 0.069000;
      SYNC_PULSE = 0.009000;
      SYNC_PORCH = 0.003000;
      SEP_PULSE  = 0.004500;
      SEP_PORCH  = 0.001500;

      CHAN_COUNT = 3;
      CHAN_TIME  = SEP_PULSE + SCAN_TIME;
      HALF_CHAN_TIME = SEP_PULSE + HALF_SCAN_TIME;

      CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
      CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;
      CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + HALF_CHAN_TIME + SEP_PORCH;

      LINE_TIME       = CHAN_OFFSETS[2] + HALF_SCAN_TIME;
      PIXEL_TIME      = SCAN_TIME / LINE_WIDTH;
      HALF_PIXEL_TIME = HALF_SCAN_TIME / LINE_WIDTH;
      WINDOW_FACTOR   = 4.88;

      HAS_ALT_SCAN    = false;
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

template <typename T>
const SSTVMode *SstvDecoder<T>::decodeVIS(const float *buf, unsigned int size)
{
  unsigned int j, i, mode;

  // Verify size
  if(visSize>size)
  {
      char msg[256];
      sprintf(msg, "decodeVIS() got %d<%d samples!", visSize, size);
      printString(msg);
      return(0);
  }

  // Decode bits
  for(j=0, i=0, mode=0 ; j<8 ; ++j)
  {
    int peak = fftPeakFreq(buf + bitSize*j, wndSize);
//{char s[128];sprintf(s," [V BIT%d=%d %dms %dHz]",j,peak<=1200,msecs(bitSize*j),peak);printString(s);}

    if(peak<=1200)
    {
      mode |= 1<<j;
      i ^= 1;
    }
  }

//{char buf[128];sprintf(buf," [MID=0x%02X, P=%d]",mode,i);printString(buf);}

  // Check parity (must be even)
  if(i)
  {
      char msg[256];
      sprintf(msg, "decodeVIS() parity check failed for 0x%02X!", mode & 0x7F);
      printString(msg);
      return(0);
  }

  // Delete parity bit
  mode &= 0x7F;

  // Get mode
  switch(mode)
  {
    case 8:  return(&MODE_Robot36);
    case 12: return(&MODE_Robot72);
    case 40: return(&MODE_Martin2);
    case 44: return(&MODE_Martin1);
    case 56: return(&MODE_Scottie2);
    case 60: return(&MODE_Scottie1);
    case 76: return(&MODE_ScottieDX);
  }

  // Failed decoding mode
  char msg[256];
  sprintf(msg, "decodeVIS() unknown mode 0x%02X!", mode);
  printString(msg);
  return(0);
}

namespace Csdr {
    template class SstvDecoder<complex<float>>;
    template class SstvDecoder<float>;
}
