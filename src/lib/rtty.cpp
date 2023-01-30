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

const char RttyDecoder::ita2Table[2*32] =
{
  // LTRS mode
  NUL,'E',LF,'A',' ','S','I','U',CR,'D','R','J','N','F','C','K',
  'T','Z','L','W','H','Y','P','Q','O','B','G',FIGS,'M','X','V',LTRS,

  // FIGS mode
  NUL,'3',LF,'-',' ','\'','8','7',CR,ENQ,'4',BEL,',','!',':','(',
  '5','+',')','2','$','6','0','1','9','?','&',FIGS,'.','/',';',LTRS
};

RttyDecoder::RttyDecoder(unsigned int sampleRate, unsigned int targetFreq, unsigned int targetWidth, double baudRate)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  targetWidth(targetWidth),
  baudRate(baudRate),
  quTime(5),      // Quantization step (ms)
  dbgTime(0)      // Debug printout period (ms)
{
    buckets = sampleRate/targetWidth; // Number of FFT buckets
    step    = quTime*sampleRate/1000; // Quantization step in samples
    buf     = new float[buckets];     // Temporary sample buffer

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
    unsigned int i, j;

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
    double mag  = mag1>mag2? mag1 : mag2;

    // Keep track of minimal/maximal magnitude
    magL += mag<magL? (mag-magL)/10.0 : (magH-magL)/1000.0;
    magH += mag>magH? (mag-magH)/10.0 : (magL-magH)/1000.0;

    // Compute current state based on the magnitude
    if((mag1>(magL+(magH-magL)*0.6)) && (mag2<(magL+(magH-magL)*0.4))) ++state0;
    if((mag2>(magL+(magH-magL)*0.6)) && (mag1<(magL+(magH-magL)*0.4))) ++state1;

    // If done with the current bit...
    if(millis-lastStartT >= 1000/baudRate)
    {
        // Detect ONE or ZERO bit, reset code if none detected
        if(state0>state1*2)
            code = (code<<1) | 0;
        else if(state1>state0*2)
            code = (code<<1) | 1;
        else
            code = 1;

        // Done with the bit
        lastStartT = millis;
        state0 = 0;
        state1 = 0;

        // If we collected enough bits, decode character
        if(code>=0x100)
        {
            // Verify start=0 and stop=11 bits
            if(!(code & 0x80) && (code & 0x02) && (code & 0x01))
            {
                // Convert ITA2 code to ASCII character
                char chr = ita2char((code>>2) & 0x1F);
                switch(chr)
                {
                    // Switch between LTRS and FIGS modes
                    case LTRS: figsMode = false;break;
                    case FIGS: figsMode = true;break;

                    default:
                        // If it is a printable character...
                        if(chr>=' ')
                        {
                            // Print character
                            *(writer->getWritePointer()) = chr;
                            writer->advance(1);
                        }
                        break;
                }
            }

            // Done with the code
            code = 1;
        }
    }

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
