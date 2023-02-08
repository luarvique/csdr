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

    // Allocate FFT plan and buffers
    fftIn   = new float[wndSize];
    fftOut  = new fftwf_complex[wndSize];
    fftPlan = fftwf_plan_dft_r2c_1d(wndSize, fftIn, fftOut, FFTW_ESTIMATE);
}

template <typename T>
SstvDecoder<T>::~SstvDecoder() {
    if(buf) { delete[] buf;buf=0; }
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
    unsigned int j, i;

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
{char buf[128];sprintf(buf," [H %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<hdrSize) break;
            // Detect SSTV frame header
            i = findHeader(buf, curSize);
if(i){char buf[128];sprintf(buf," [HDR @ %dms] [VIS @ %dms]",msecs(i-hdrSize),msecs(i));printString(buf);}
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
{char buf[128];sprintf(buf," [V %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<visSize) break;
            // Try decoding
            curMode = decodeVIS(buf, visSize);
{char buf[128];sprintf(buf," [MODE='%s']",curMode? curMode->NAME:"???");printString(buf);}
            // If failed, go back to header detection, else wait for scanlines
            curState = !curMode? STATE_HEADER : curMode->HAS_START_SYNC? STATE_SYNC : STATE_LINE0;
            // Drop decoded input data
            if(curMode) skipInput(visSize);
            // Done
            break;

        case STATE_SYNC:
            // We will need this many input samples for SYNC
            j = round(curMode->SYNC_PULSE * 1.4 * sampleRate);
            // Do not detect until we have this many
{char buf[128];sprintf(buf," [S %d..%d]",msecs(),msecs(curSize));printString(buf);}
            if(curSize<j) break;
            // Detect SSTV frame sync
            i = findSync(curMode, buf, curSize);
if(i){char buf[128];sprintf(buf," [SYNC @ %dms]",msecs(i));printString(buf);}
            // If sync detected, decoding image next
            if(i) curState = STATE_LINE0;
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
{char buf[128];sprintf(buf," [L %d..%d]",msecs(),msecs(curSize));printString(buf);}
            // If invalid state or done with a frame, go back to header detection
            if(!curMode || (curState<0) || (curState>=curMode->LINE_COUNT))
            {
                curState = STATE_HEADER;
                break;
            }
            // Done, wait for the next scanline
            ++curState;
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
unsigned int SstvDecoder<T>::findSync(const SSTVMode *mode, const float *buf, unsigned int size, bool startOfSync)
{
    unsigned int wndSize = round(mode->SYNC_PULSE * 1.4 * sampleRate);

    // Must have enough samples
    if(wndSize>size) return(0);

    // Search for the sync signal
    for(unsigned int j=0 ; j<=size-wndSize ; ++j)
    {
        if(fftPeakFreq(buf + j, wndSize)>1350)
        {
            // This is the end of sync
            j = j + wndSize/2;
            return(startOfSync? (j - round(mode->SYNC_PULSE*sampleRate)) : j);
        }
    }

    // Not found
    return(0);
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
{char s[128];sprintf(s," [V BIT%d=%d %dms %dHz]",j,peak<=1200,msecs(bitSize*j),peak);printString(s);}

    if(peak<=1200)
    {
      mode |= 1<<j;
      i ^= 1;
    }
  }

{char buf[128];sprintf(buf," [MID=0x%02X, P=%d]",mode,i);printString(buf);}

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
