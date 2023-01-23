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
  buckets(sampleRate/targetWidth),
  step(quTime*sampleRate/1000),
  magL(1000.0),
  magH(0.0),
  realState0(0),
  filtState0(0),
  lastStartT(0),
  startTimeL(0),
  startTimeH(0),
  avgTimeH(30),
  avgTimeL(30),
  code(1),
  stop(0),
  wpm(0),
  curSeconds(0),
  curSamples(0),
  bufPos(0)
{
    double v = round((double)buckets * targetFreq / sampleRate);

    coeff = 2.0 * cos(2.0 * M_PI * v / buckets);
    buf   = new float[buckets];
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
    unsigned int i;

static unsigned int HistH[32] = {0};
static unsigned int HistL[32] = {0};
static unsigned int CountH = 0;
static unsigned int CountL = 0;

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
        magnitude>(magL+(magH-magL)*0.5)? 1 :
        magnitude<(magL+(magH-magL)*0.5)? 0 :
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

HistL[durationL/10<32? durationL/10:31]++;
CountL++;

            // At high speeds we have to have a little more pause
            double m = wpm>35? 1.5 : wpm>30? 1.2 : wpm>25? 1.0 : 1.0;
            unsigned int pauseTime = (unsigned int)(m * avgTimeL);
pauseTime = avgTimeL;

            // If we have got some dits or dahs...
            if(code>1)
            {
                // If letter space...
                if((durationL>2*pauseTime) && (durationL<5*pauseTime))
                {
                    // Letter space...
                    *(writer->getWritePointer()) = cw2char(code);
                    writer->advance(1);
                    // Start new character
                    code = 1;
                }
                else if(durationL>=5*pauseTime)
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
            if((durationL>=10) && (durationL<2*avgTimeL))
                avgTimeL += (int)(durationL-avgTimeL)/3;
        }
        else
        {
            // Ending a HIGH state...

            // Compute HIGH duration
            startTimeL = millis;
            durationH  = millis - startTimeH;

HistH[durationH/10<32? durationH/10:31]++;
CountH++;

            // 0.6 to filter out false dits
            if((durationH<=2*avgTimeH) && (durationH>0.6*avgTimeH))
            {
                // Print a dit
                *(writer->getWritePointer()) = '.';
                writer->advance(1);
                // Add a dit to the code
                code = (code<<1) | 1;
            }
            else if((durationH>2*avgTimeH) && (durationH<6*avgTimeH))
            {
                // Print a dah
                *(writer->getWritePointer()) = '-';
                writer->advance(1);
                // Add a dah to the code
                code = (code<<1) | 0;
                // Try computing WPM
                wpm = (wpm + (1200/(durationH/3)))/2;
            }

            // Keep track of the average HIGH duration
            if((durationH>=10) && (durationH<2*avgTimeH))
                avgTimeH += (int)(durationH - avgTimeH)/3;
            else if((durationH>=3*avgTimeH) && (durationH<300))
                avgTimeH += (int)(durationH/3 - avgTimeH)/3;
        }


#if 1
{
char buf[256];
static int aaa=0;
if(++aaa>100){
aaa=0;
for(int j=0;j<32;++j) {
i = 10*HistH[j]/(CountH+1);
buf[j+1]=i<1? '-':i<10? '0'+i:'*';
i = 10*HistL[j]/(CountL+1);
buf[j+34]=i<1? '-':i<10? '0'+i:'*';
}
buf[0]='\n';
buf[33]='|';
memset(HistL,0,sizeof(HistL));
memset(HistH,0,sizeof(HistH));
CountL=CountH=0;
sprintf(buf+33+33, "[%d-%d %dms|%dms WPM%d]\n", (int)magL, (int)magH, avgTimeH, avgTimeL, wpm);
for(int j=0;buf[j];++j) {
  *(writer->getWritePointer()) = buf[j];
  writer->advance(1);
}
}}
#endif
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

    // Update state
    realState0 = realState;
    filtState0 = filtState;
}
