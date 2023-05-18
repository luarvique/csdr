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

#include "rtty.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

using namespace Csdr;

#define NUL  '\0'
#define LF   '\n'
#define CR   '\r'
#define BEL  '\007'
#define LTRS '\001'
#define FIGS '\002'
#define ENQ  '\003'

// CQ CQ RU3AMO RU3AMO RU3AMO
// CQ PSE K
//
// CQ RU3AMO =
//   C=0|01110|11 Q=0|10111|11  =0|00100|11 R=0|01010|11
//   U=0|00111|11 ^=0|11011|11 3=0|00001|11 v=0|11111|11
//   A=0|00011|11 M=0|11100|11 O=0|11000|11
//
template <typename T>
const char RttyDecoder<T>::ita2Table[2*32] =
{
  // LTRS mode
  NUL,'E',LF,'A',' ','S','I','U',CR,'D','R','J','N','F','C','K',
  'T','Z','L','W','H','Y','P','Q','O','B','G',FIGS,'M','X','V',LTRS,

  // FIGS mode
  NUL,'3',LF,'-',' ','\'','8','7',CR,ENQ,'4',BEL,',','!',':','(',
  '5','+',')','2','$','6','0','1','9','?','&',FIGS,'.','/',';',LTRS
};

static const int rev[32] =
{
  0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30,
  1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31
};

template <typename T>
RttyDecoder<T>::RttyDecoder(unsigned int sampleRate, int targetFreq, int targetWidth, double baudRate, bool reverse)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  targetWidth(targetWidth),
  baudRate(baudRate),
  reverse(reverse),
  quTime(2),        // Quantization step (ms)
  dbgTime(0),       // Debug printout period (ms)
  showRaw(false)    // TRUE: print raw data
{
    unsigned int i;

    buckets = sampleRate/50;           // Number of 50Hz FFT buckets
    i       = 1000*buckets/sampleRate;
    quTime  = quTime<i? quTime : i;    // Make quTime smaller than a bucket
    step    = quTime*sampleRate/1000;  // Quantization step in samples
    buf     = new float[buckets];      // Temporary sample buffer

    // Goertzel algorithm coefficients
    double v1 = round((double)buckets * targetFreq / sampleRate);
    double v2 = round((double)buckets * (targetFreq + targetWidth) / sampleRate);
    coeff1 = 2.0 * cos(2.0 * M_PI * v1 / buckets);
    coeff2 = 2.0 * cos(2.0 * M_PI * v2 / buckets);
}

template <typename T>
RttyDecoder<T>::~RttyDecoder() {
    if(buf) { delete[] buf;buf=0; }
}

template <typename T>
bool RttyDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=(buckets-bufPos)) && (this->writer->writeable()>=1);
}

template <typename T>
void RttyDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    unsigned int i, j;

    // Read input data into the buffer
    while(bufPos<buckets) {
        buf[bufPos++] = *(this->reader->getReadPointer());
        this->reader->advance(1);
    }

    // Process buffered data
    for(i=0 ; i+buckets<=bufPos ; i+=step) {
        // Process data
        processInternal(buf+i, buckets);

        // Update time
        curSamples += step;
        if(curSamples>=sampleRate)
        {
            unsigned int secs = curSamples/sampleRate;
            curSeconds += secs;
            curSamples -= secs*sampleRate;
        }
    }

    // Shift data
    for(j=0 ; i+j<bufPos ; ++j) buf[j]=buf[i+j];

    // Done with the data
    bufPos -= i;
}

template <typename T>
void RttyDecoder<T>::processInternal(float *data, unsigned int size) {
    unsigned long millis = msecs();
    double q10, q11, q12;
    double q20, q21, q22;
    unsigned int state, i;

    // If done with the current bit...
    if(millis-lastStartT >= 1000.0/baudRate)
    {
        // Detect ONE or ZERO bit
        state = state1>2*state0? 1 : state0>2*state1? 0 : lastState;
        code = (code<<1) | state;

        // Count MARKs and SPACEs for debugging purposes
        cnt0 += state0;
        cnt1 += state1;

        // Print current digit
        if((code>1) && showRaw)
        {
            // >: START bit
            // |: STOP bit CORRECT
            // ?: STOP bit WRONG
            *(this->writer->getWritePointer()) =
                code==2? '>' : code<0x80? '0'+(code&1) : code&1? '|' : '?';
            this->writer->advance(1);
        }

        // If current state differs from computed recent state...
        if(state!=lastState)
        {
            // Sync to when current state supposedly started
            lastStartT = lastChangeT;
            if(lastState==1) { state0 = 0; state1 = lastChange; }
            if(lastState==0) { state1 = 0; state0 = lastChange; }
        }
        else
        {
            // Done with the bit
            lastStartT = millis;
            state0 = 0;
            state1 = 0;
        }

        // If collected 5xDATA + 1xSTOP bits, decode character
        if(code>=0x80)
        {
            // Convert 5bit ITA2 code to ASCII character
            // Check that START=0 and STOP=1
            char chr = (code&0x41)==0x01? ita2char(rev[(code>>1) & 0x1F]) : '#';

            // Handle special characters
            switch(chr)
            {
                // Switch between LTRS and FIGS modes
                case LTRS: figsMode = false;break;
                case FIGS: figsMode = true;break;
            }

            // Print decoded character
            if((chr>=' ') || (chr==LF))// || (chr==CR))
            {
                *(this->writer->getWritePointer()) = chr;
                this->writer->advance(1);
            }

            // Done with the code
            code = 1;
        }
    }

    // Read samples
    for(i=0, q11=q12=q21=q22=0.0 ; i<size ; ++i)
    {
        q10 = q11 * coeff1 - q12 + data[i];
        q12 = q11;
        q11 = q10;
        q20 = q21 * coeff2 - q22 + data[i];
        q22 = q21;
        q21 = q20;
    }

    // We only need the real part
    double mag1 = sqrt(q11*q11 + q12*q12 - q11*q12*coeff1);
    double mag2 = sqrt(q21*q21 + q22*q22 - q21*q22*coeff2);
    double mag;

    // Keep track of minimal/maximal magnitude
    mag   = mag1<mag2? mag1 : mag2;
    magL += mag<magL? (mag-magL)/10.0 : (magH-magL)/1000.0;
    mag   = mag1>mag2? mag1 : mag2;
    magH += mag>magH? (mag-magH)/10.0 : (magL-magH)/1000.0;

    // Compute current state based on the magnitude
    state = mag2>mag1? (reverse? 0:1) : mag1>mag2? (reverse? 1:0) : lastState;

    // Keep count of observed states
    i = 100*fabs((mag2-mag1)/(magH-magL));
    if(state==1) state1+=i; else if(state==0) state0+=i;

    if(state==lastState)
    {
        lastChange += i;
    }
    else
    {
        lastState   = state;
        lastChangeT = millis;
        lastChange  = i;
    }

    // Sync to SPACEs
    if((code==1) && (state1>=2*state0))
    {
        state1 = 0;
        state0 = 0;
        lastStartT = millis;
    }

    // Sync to MARKs
    if((code>=0x40) && (code<0x80) && (state0>=2*state1))
    {
        state1 = 0;
        state0 = 0;
        lastStartT = millis;
    }

#if 0
for(i=0 ; i<40 ; ++i)
if(i==(int)(mag1/10.0)) printf("1");
else if(i==(int)(mag2/10.0)) printf("2");
else if(i==(int)(magL/10.0)) printf("L");
else if(i==(int)(magH/10.0)) printf("H");
else printf(".");
printf("\n");
#endif

    // Print current level (SPACE, MARK or nothing)
    if(showRaw)
    {
        *(this->writer->getWritePointer()) = state==1? '-':state==0? '_':' ';
        this->writer->advance(1);
    }

    // Periodically print debug information, if enabled
    if(dbgTime && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }
}

template <typename T>
void RttyDecoder<T>::printDebug()
{
    char buf[256];
    sprintf(buf, "[%u - %u, magL=%.3f, magH=%.3f]", cnt0, cnt1, magL, magH);
    printString(buf);
    cnt0 = cnt1 = 0;
}

template <typename T>
void RttyDecoder<T>::printString(const char *buf)
{
    unsigned int l = strlen(buf);

    // If there is enough output buffer available...
    if(this->writer->writeable()>=l)
    {
        // Write data then advance pointer
        memcpy(this->writer->getWritePointer(), buf, l);
        this->writer->advance(l);
    }
}

namespace Csdr {
    template class RttyDecoder<complex<float>>;
    template class RttyDecoder<float>;
}
