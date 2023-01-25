/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2021 Jakob Ketterl <jakob.ketterl@gmx.de>
Copyright (c) 2022 Marat Fayzullin <luarvique@gmail.com>
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

#include "cw.hpp"
#include <cmath>
#include <cstring>

using namespace Csdr;

const char CwDecoder::cwTable[] =
    "##TEMNAIOGKDWRUS" // 00000000
    "##QZYCXBJP#L#FVH"
    "09#8#<#7#(###/-6" // <AR>
    "1######&2###3#45"
    "#######:####,###" // 01000000
    "##)#!;########-#"
    "#'###@####.#####"
    "###?######{#####" // <SK>
    "################" // 10000000
    "################"
    "################"
    "################"
    "################" // 11000000
    "################"
    "################"
    "######$#########";

CwDecoder::CwDecoder(unsigned int sampleRate, unsigned int targetFreq, unsigned int targetWidth)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  quTime(5),
  nbTime(20),
  dbgTime(30000),
  buckets(sampleRate/targetWidth),
  step(quTime*sampleRate/1000),
  magL(1000.0),
  magH(0.0),
  realState0(0),
  filtState0(0),
  lastStartT(0),
  startTimeL(0),
  startTimeH(0),
  avgTimeH(50),
  avgTimeL(50),
  code(1),
  stop(0),
  wpm(0),
  curSeconds(0),
  curSamples(0),
  bufPos(0),
  histCntH(0),
  histCntL(0),
  lastDebugT(0)
{
    double v = round((double)buckets * targetFreq / sampleRate);

    coeff = 2.0 * cos(2.0 * M_PI * v / buckets);
    buf   = new float[buckets];

    memset(histH, 0, sizeof(histH));
    memset(histL, 0, sizeof(histL));
}

CwDecoder::~CwDecoder() {
    if(buf) { delete[] buf;buf=0; }
}

bool CwDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (reader->available()>=(buckets-bufPos)) && (writer->writeable()>=2);
}

void CwDecoder::process() {
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

void CwDecoder::processInternal(float *data, unsigned int size) {
    unsigned long millis = msecs();
    double q0, q1, q2;
    unsigned int i, j;

    // Read samples
    for(i=0, q1=q2=0.0 ; i<size ; ++i)
    {
        q0 = q1 * coeff - q2 + data[i];
        q2 = q1;
        q1 = q0;
    }

    // We only need the real part
    double magnitude = sqrt(q1*q1 + q2*q2 - q1*q2*coeff);

    // Keep track of minimal/maximal magnitude
    magL += magnitude<magL? (magnitude-magL)/10.0 : (magH-magL)/1000.0;
    magH += magnitude>magH? (magnitude-magH)/10.0 : (magL-magH)/1000.0;

    // Compute current state based on the magnitude
    unsigned int realState =
        magnitude>(magL+(magH-magL)*0.6)? 1 :
        magnitude<(magL+(magH-magL)*0.4)? 0 :
        realState0;

    // Filter out jitter based on nbTime
    if(realState!=realState0) lastStartT = millis;
    unsigned int filtState = (millis-lastStartT)>nbTime? realState : filtState0;

    // If signal state changed...
    if(filtState!=filtState0)
    {
        // Mark change in signal state
        stop = 0;

        if(filtState)
        {
            // Ending a LOW state...

            // Compute LOW duration
            startTimeH = millis;
            durationL  = millis - startTimeL;

            // Accumulate histogram data
            i = sizeof(histL) / sizeof(histL[0]);
            j = durationL / 10;
            histL[j<i? j:i-1]++;
            histCntL++;

            // If we have got some dits or dahs...
            if(code>1)
            {
                // If letter space...
                if((durationL>2*avgTimeL) && (durationL<5*avgTimeL))
                {
                    // Letter space...
                    *(writer->getWritePointer()) = cw2char(code);
                    writer->advance(1);
                    // Start new character
                    code = 1;
                }
                else if(durationL>=5*avgTimeL)
                {
                    // Word space
                    *(writer->getWritePointer()) = cw2char(code);
                    writer->advance(1);
                    *(writer->getWritePointer()) = ' ';
                    writer->advance(1);
                    // Start new character
                    code = 1;
                }
            }

            // Keep track of the average LOW duration
            if((durationL>20) && (durationL<2*avgTimeL))
                avgTimeL += (int)(durationL - avgTimeL)/10;
            else if((durationL>3*avgTimeL) && (durationL<6*avgTimeL))
                avgTimeL += (int)(durationL/3 - avgTimeL)/10;
        }
        else
        {
            // Ending a HIGH state...

            // Compute HIGH duration
            startTimeL = millis;
            durationH  = millis - startTimeH;

            // Accumulate histogram data
            i = sizeof(histH) / sizeof(histH[0]);
            j = durationH / 10;
            histH[j<i? j:i-1]++;
            histCntH++;

            // 0.6 to filter out false dits
            if((durationH<=2*avgTimeH) && (durationH>0.6*avgTimeH))
            {
                // Print a dit
//                *(writer->getWritePointer()) = '.';
//                writer->advance(1);
                // Add a dit to the code
                code = (code<<1) | 1;
            }
            else if((durationH>2*avgTimeH) && (durationH<6*avgTimeH))
            {
                // Print a dah
//                *(writer->getWritePointer()) = '-';
//                writer->advance(1);
                // Add a dah to the code
                code = (code<<1) | 0;
                // Try computing WPM
                wpm = (wpm + (1200/(durationH/3)))/2;
            }

            // Keep track of the average HIGH duration
            if((durationH>20) && (durationH<2*avgTimeH))
                avgTimeH += (int)(durationH - avgTimeH)/10;
            else if((durationH>3*avgTimeH) && (durationH<250))
                avgTimeH += (int)(durationH/3 - avgTimeH)/10;
        }
    }

    // Write if no more letters
    if(((millis-startTimeL)>6*durationH) && !stop)
    {
        if(code>1)
        {
            *(writer->getWritePointer()) = cw2char(code);
            writer->advance(1);
            *(writer->getWritePointer()) = ' ';
            writer->advance(1);
            // Start new character
            code = 1;
        }

        stop = 1;
    }

    // Periodically print debug information
    if(millis-lastDebugT >= dbgTime)
    {
        lastDebugT = millis;
        printDebug();
    }

    // Update state
    realState0 = realState;
    filtState0 = filtState;
}

void CwDecoder::printDebug()
{
    char buf[256];
    int i, j;

    i = sizeof(histH) / sizeof(histH[0]);
    for(j=0 ; j<i ; ++j)
    {
        int h = 10 * histH[j] / (histCntH+1);
        int l = 10 * histL[j] / (histCntL+1);

        buf[j+2]   = h>9? '*' : h>0? '0'+h : histH[j]>0? '0' : '-';
        buf[j+i+3] = l>9? '*' : l>0? '0'+l : histL[j]>0? '0' : '-';
        histH[j] = histL[j] = 0;
    }

    histCntH = histCntL = 0;
    buf[0]   = '\n';
    buf[1]   = '[';
    buf[i+2] = '|';

    sprintf(buf+2*i+3, "] [%d-%d %dms|%dms WPM%d]\n", (int)magL, (int)magH, avgTimeH, avgTimeL, wpm);
    for(j=0 ; buf[j] ; ++j)
    {
        *(writer->getWritePointer()) = buf[j];
        writer->advance(1);
    }
}
