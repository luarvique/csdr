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
const char RttyDecoder::ita2Table[2*32] =
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

RttyDecoder::RttyDecoder(unsigned int sampleRate, unsigned int targetFreq, unsigned int targetWidth, double baudRate)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  targetWidth(targetWidth),
  baudRate(baudRate),
  reverse(false),
  quTime(2),//5),      // Quantization step (ms)
  dbgTime(0)  // Debug printout period (ms)
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

RttyDecoder::~RttyDecoder() {
    if(buf) { delete[] buf;buf=0; }
}

bool RttyDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (reader->available()>=(buckets-bufPos)) && (writer->writeable()>=1);
}

void RttyDecoder::process() {
    unsigned int i, j;

    // Read input data into the buffer
    while(bufPos<buckets) {
        buf[bufPos++] = *(reader->getReadPointer());
        reader->advance(1);
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

void RttyDecoder::processInternal(float *data, unsigned int size) {
    unsigned long millis = msecs();
    double q10, q11, q12;
    double q20, q21, q22;
    unsigned int state, i;

    // If done with the current bit...
    if(millis-lastStartT >= 1000.0/baudRate)
    {
        // Detect ONE or ZERO bit
        state = state1>state0? 1 : 0;
        code = (code<<1) | state;

        // Print current digit
        if(code>1)
        {
//            *(writer->getWritePointer()) = code==2? '>' : '0'+(code&1);
//            writer->advance(1);
        }

        // If current state differs from computed recent state...
        if(state!=lastState)
        {
            // Sync to when current state supposedly started
            lastStartT = lastChangeT;
            if(lastState==1) state0 = 0;
            if(lastState==0) state1 = 0;
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
            char chr = ita2char(rev[(code>>1) & 0x1F]);

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
                *(writer->getWritePointer()) = chr;
                writer->advance(1);
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

    // Compute current state based on the magnitude
    state = lastState;
    if((mag1>(magL+(magH-magL)*0.6)) && (mag2<(magL+(magH-magL)*0.4))) state = reverse? 1:0;
    if((mag2>(magL+(magH-magL)*0.6)) && (mag1<(magL+(magH-magL)*0.4))) state = reverse? 0:1;
    if(state==1) state1++; else if(state==0) state0++;

    if(state!=lastState)
    {
        lastChangeT = millis;
        lastState = state;
    }

    // Sync to SPACEs
    if((code==1) && (state1>=state0))
    {
        state1 = state==1? 1:0;
        state0 = state==0? 1:0;
        lastStartT = millis;
    }

    // Print current level (SPACE, MARK or nothing)
//    *(writer->getWritePointer()) = state==1? '-':state==0? '_':' ';
//    writer->advance(1);

    // Keep track of minimal/maximal magnitude
    if(mag1>mag2) { double mag=mag1;mag1=mag2;mag2=mag1; }
    magL += mag1<magL? (mag1-magL)/10.0 : (magH-magL)/1000.0;
    magH += mag2>magH? (mag2-magH)/10.0 : (magL-magH)/1000.0;

    // Periodically print debug information, if enabled
    if(dbgTime && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }
}

void RttyDecoder::printDebug()
{
    char buf[256];
    int j;

    // @@@ WRITE CODE HERE!
    buf[0] = '\0';
    sprintf(buf, "%lu: magL=%.3f, magH=%.3f, [0]x%d, [1]x%d\n", msecs(), magL, magH, state0, state1);

    // If there is enough output buffer available...
    if(writer->writeable()>=strlen(buf))
    {
        // Place each string character into the output buffer
        for(j=0 ; buf[j] ; ++j)
        {
            *(writer->getWritePointer()) = buf[j];
            writer->advance(1);
        }
    }
}
