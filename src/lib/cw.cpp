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

using namespace Csdr;

CwDecoder::CwDecoder(unsigned int sampleRate, unsigned int dahTime, unsigned int ditTime)
: sampleRate(sampleRate),
  avgLevel(0.5*MAX_HISTORY),
  dahTime(dahTime),
  ditTime(ditTime),
  hisPtr(0),
  curSignal(0),
  curTime(0),
  iniTime(0),
  data(1)
{
    for(int i=0; i<MAX_HISTORY ; ++i)
    {
        hisLevel[i] = 0.5;
    }
}

bool CwDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= ms2smp(QUANTUM_MSEC) && writer->writeable() > 0;
}

void CwDecoder::process() {
    unsigned int n = ms2smp(QUANTUM_MSEC);
    unsigned int j;
    double curLevel;
    char out;

    std::lock_guard<std::mutex> lock(this->processMutex);

    // Compute average signal level in the input
    for(j=0, curLevel=0.0; (j<n) && reader->available(); ++j)
    {
        curLevel += fabs(*(reader->getReadPointer()));
        reader->advance(1);
    }

    if(!j) return;

    // Determine if we have signal or not
    curLevel /= j;
    j = curLevel>1.25*avgLevel/MAX_HISTORY? 1:0;

    // Keep track of the long-term average signal level
    avgLevel += curLevel - hisLevel[hisPtr];
    hisLevel[hisPtr] = curLevel;
    hisPtr = hisPtr<MAX_HISTORY-1? hisPtr+1 : 0;

#if 0
{
static char logBuf[256];
static unsigned int count = 0;
if(++count>=100)
{
count=0;
sprintf(logBuf, " %dHz %.03f %dms %dms ", sampleRate, avgLevel/MAX_HISTORY, ditTime, dahTime);
for(int k=0; logBuf[k]; ++k)
{
*(writer->getWritePointer()) = logBuf[k];
writer->advance(1);
}
}
}
#endif

    // Inject current signal status into the parser
    out = parseSignal(j, QUANTUM_MSEC);

    // If got a character, output it
    if(out)
    {
        *(writer->getWritePointer()) = out;
        writer->advance(1);
    }
}

char CwDecoder::parseSignal(unsigned int signal, unsigned int msec)
{
    static const char cw2char[] =
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

    char result = '\0';

    // Eliminate jitter
    if((signal!=curSignal) && (iniTime+msec<=QUANTUM_MSEC))
    {
        iniTime += msec;
        return(result);
    }
    else
    {
        msec += iniTime;
        iniTime = 0;
    }

    // When signal level is low for a while, decode character
    if(!curSignal && (curTime>dahTime))
    {
        if(data>1)
        {
            result = data<256? cw2char[data] : '#';
            curTime = 0;
            data = 1;
        }
        else if(signal && (curTime>(2*dahTime)))
        {
            result = ' ';
        }
    }

    // If signal level changed...
    if(signal!=curSignal)
    {
        if(curSignal)
        {
            // Parse dit or dah
            data = (data << 1) | (curTime<((ditTime+dahTime)>>1)? 1:0);

            if(curTime<((ditTime+dahTime)>>1))
            {
                ditTime = (curTime + ditTime) >> 1;
                dahTime = (3 * ditTime) >> 1;
            }
            else
            {
                dahTime = (dahTime + curTime) >> 1;
                ditTime = (dahTime << 1) / 3;
            }
        }

        // Reset time and signal level
        curSignal = signal;
        curTime = 0;
    }

    // Update signal and time
    curTime += msec;
    return(result);
}
