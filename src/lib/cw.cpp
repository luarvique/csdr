
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

CwDecoder::CwDecoder(unsigned int sampleRate, unsigned int dahTime, unsigned int ditTime)
: sampleRate(sampleRate),
  MagLimit(0.5),
  MagLimitL(0.5),
  RealState(0),
  RealState0(0),
  FiltState0(0),
  FiltState(0),
  NBTime(6),
  Code(1),
  Stop(0),
  WPM(0),
  LastStartT(0),
  StartTimeL(0),
  StartTimeH(0),
  AvgTimeH(0),
  quantum(48),
  curTime(0),
  curSamples(0),
  targetFreq(992)
{
    unsigned int J = (int)((double)(quantum * targetFreq) / sampleRate + 0.5);
    Coeff = 2.0 * cos((2.0 * M_PI * J) / quantum);
}

bool CwDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= quantum && writer->writeable() >= 2;
}

void CwDecoder::process() {
    unsigned long millis = msecs();
    double Q0, Q1, Q2;
    unsigned int I;

    // Read samples
    for(I=0, Q1=Q2=0.0 ; I<quantum ; ++I)
    {
        Q0 = Q1 * Coeff - Q2 + (*(reader->getReadPointer()));
        Q2 = Q1;
        Q1 = Q0;

        reader->advance(1);
    }

    // We only need the real part
    double Magnitude = sqrt(Q1*Q1 + Q2*Q2 - Q1*Q2*Coeff);

    // Try to set the automatic magnitude limit
    if(Magnitude>MagLimitL) MagLimit += (Magnitude - MagLimit) / 6.0;
    if(MagLimit<MagLimitL)  MagLimit = MagLimitL;

    // Check the magnitude
    RealState = Magnitude>MagLimit*0.6? 1 : 0;

    // Clean up the state with a noise blanker
    if(RealState!=RealState0) LastStartT = millis;
    if((millis-LastStartT)>NBTime) FiltState = RealState;

    // If signal state changed...
    if(FiltState!=FiltState0)
    {
        // Compute high / low durations
        if(FiltState)
        {
            StartTimeH = millis;
            DurationL  = millis - StartTimeL;
        }
        else
        {
            StartTimeL = millis;
            DurationH  = millis - StartTimeH;

            // Now we know average dit time (rolling 3 average)
            if((DurationH<2*AvgTimeH) || (AvgTimeH==0))
                AvgTimeH = (DurationH+AvgTimeH+AvgTimeH)/3;

            // If speed decreases fast...
            if(DurationH>5*AvgTimeH)
                AvgTimeH = DurationH + AvgTimeH;


#if 0
{
char buf[256];
sprintf(buf, "%d / %d", DurationH, AvgTimeH);
for(int j=0;buf[j];++j) {
  *(writer->getWritePointer()) = buf[j];
  writer->advance(1);
}}
#endif

        }

        // now we will check which kind of baud we have - dit or dah
        // and what kind of pause we do have 1 - 3 or 7 pause
        // we think that AvgTimeH = 1 bit
        Stop = 0;

        // If ending a HIGH state...
        if(!FiltState)
        {
            // 0.6 to filter out false dits
            if((DurationH<2*AvgTimeH) && (DurationH>0.6*AvgTimeH))
            {
              Code = (Code<<1) | 1;
//*(writer->getWritePointer()) = '.';
//writer->advance(1);
            }
            else if((DurationH>2*AvgTimeH) && (DurationH<6*AvgTimeH))
            {
              // The most precise we can do
              WPM  = (WPM + (1200/(DurationH/3)))/2;
              Code = (Code<<1) | 0;
//*(writer->getWritePointer()) = '-';
//writer->advance(1);
            }
        }
        else
        {
            // Ending a LOW state...
            // At high speeds we have to have a little more pause before
            // new letter or new word
            double LackTime = WPM>35? 1.5 : WPM>30? 1.2 : WPM>25? 1.0 : 1.0;

            // If letter space...
            if((Code>1) && (DurationL>2*LackTime*AvgTimeH) && (DurationL<5*LackTime*AvgTimeH))
            {
                // Letter space...
                *(writer->getWritePointer()) = cw2char(Code);
                writer->advance(1);
                // Start new character
                Code = 1;
            }
            else if(DurationL>=5*LackTime*AvgTimeH)
            {
                // Word space
                if(Code>1)
                {
                    *(writer->getWritePointer()) = cw2char(Code);
                    writer->advance(1);
                }
                *(writer->getWritePointer()) = ' ';
                writer->advance(1);
                // Start new character
                Code = 1;
            }
        }
    }

    // Write if no more letters
    if(((millis-StartTimeL)>6*DurationH) && !Stop)
    {
        if(Code>1)
        {
            *(writer->getWritePointer()) = cw2char(Code);
            writer->advance(1);
            // Start new character
            Code = 1;
        }
        Stop = 1;
    }

    // Update state
    RealState0 = RealState;
    FiltState0 = FiltState;

    // Update time
    curSamples += quantum;
    if(curSamples>=sampleRate)
    {
        unsigned int secs = curSamples/sampleRate;
        curTime += secs;
        curSamples -= secs*sampleRate;
    }
}
