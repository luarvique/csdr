
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

CwDecoder::CwDecoder(unsigned int sampleRate, unsigned int targetFreq, unsigned int buckets)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  buckets(buckets),
  MagLimit(10.0),
  MagLimitL(10.0),
  MagTotal(0.0),
  RealState0(0),
  FiltState0(0),
  NBTime(10),
  Code(1),
  Stop(0),
  WPM(0),
  LastStartT(0),
  StartTimeL(0),
  StartTimeH(0),
  AvgTimeH(30),
  curTime(0),
  curSamples(0)
{
    double V = round((double)buckets * targetFreq / sampleRate);
    Coeff = 2.0 * cos(2.0 * M_PI * V / buckets);
}

bool CwDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= buckets && writer->writeable() >= 2;
}

void CwDecoder::process() {
    unsigned long millis = msecs();
    double Q0, Q1, Q2;
    unsigned int I;

    // Read samples
    for(I=0, Q1=Q2=0.0 ; I<buckets ; ++I)
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

    // Check the magnitude
    unsigned int RealState = Magnitude>MagLimit*0.6? 1 : 0;

    // Clean up the state with a noise blanker
    if(RealState!=RealState0) LastStartT = millis;
    unsigned int FiltState = (millis-LastStartT)>NBTime? RealState : FiltState0;

    // If signal state changed...
    if(FiltState!=FiltState0)
    {
        // now we will check which kind of baud we have - dit or dah
        // and what kind of pause we do have 1 - 3 or 7 pause
        // we think that AvgTimeH = 1 bit
        Stop = 0;

        if(FiltState)
        {
            // Ending a LOW state...

            // Compute LOW duration
            StartTimeH = millis;
            DurationL  = millis - StartTimeL;

if(DurationL>=3*AvgTimeH)
{
    MagTotal /= DurationL / (1000 * buckets / sampleRate);
    MagLimit = (MagLimit + MagTotal*2.0)/2.0;
}

            // At high speeds we have to have a little more pause
            double M = WPM>35? 1.5 : WPM>30? 1.2 : WPM>25? 1.0 : 1.0;
            unsigned int PauseTime = (unsigned int)(M*AvgTimeH);

            // If we have got some dits or dahs...
            if(Code>1)
            {
                // If letter space...
                if((DurationL>2*PauseTime) && (DurationL<5*PauseTime))
                {
                    // Letter space...
                    *(writer->getWritePointer()) = cw2char(Code);
                    writer->advance(1);
                    // Start new character
                    Code = 1;
                }
                else if(DurationL>=5*PauseTime)
                {
                    // Word space
                    *(writer->getWritePointer()) = cw2char(Code);
                    writer->advance(1);
                    *(writer->getWritePointer()) = ' ';
                    writer->advance(1);
                    // Start new character
                    Code = 1;
                }
            }
        }
        else
        {
            // Ending a HIGH state...

            // Compute HIGH duration
            StartTimeL = millis;
            DurationH  = millis - StartTimeH;

if(DurationH>=3*AvgTimeH)
{
    MagTotal /= DurationH / (1000 * buckets / sampleRate);
    MagLimit = (MagLimit + MagTotal)/2.0;
}

            if((DurationH>=10) && (DurationH<2*AvgTimeH))
                AvgTimeH = (DurationH+AvgTimeH)/2;
            else if((DurationH>=3*AvgTimeH) && (DurationH<500))
                AvgTimeH = (DurationH/3+AvgTimeH)/2;

            // Now we know average dit time (rolling 3 average)
//            if((DurationH<2*AvgTimeH) || (AvgTimeH<=NBTime*3))
//                AvgTimeH = (DurationH+AvgTimeH+AvgTimeH)/3;

            // If speed decreases fast...
//            if(DurationH>5*AvgTimeH)
//                AvgTimeH = DurationH + AvgTimeH;

            // 0.6 to filter out false dits
            if((DurationH<2*AvgTimeH) && (DurationH>0.6*AvgTimeH))
            {
                // Print a dit
//                *(writer->getWritePointer()) = '.';
//                writer->advance(1);
                // Add a dit
                Code = (Code<<1) | 1;
            }
            else if((DurationH>2*AvgTimeH) && (DurationH<6*AvgTimeH))
            {
                // Print a dah
//                *(writer->getWritePointer()) = '-';
//                writer->advance(1);
                // Add a dash
                Code = (Code<<1) | 0;
                // Try computing WPM
                WPM  = (WPM + (1200/(DurationH/3)))/2;
            }
        }


#if 0
{
char buf[256];
static int aaa=0;
if(++aaa>100){
aaa=0;
sprintf(buf, "[%d %dms WPM%d]", (int)MagLimit, AvgTimeH, WPM);
for(int j=0;buf[j];++j) {
  *(writer->getWritePointer()) = buf[j];
  writer->advance(1);
}}}
#endif

        // Compute new total magnitude
        MagTotal = 0.0;
    }

    // Write if no more letters
    if(((millis-StartTimeL)>6*DurationH) && !Stop)
    {
        if(Code>1)
        {
            *(writer->getWritePointer()) = cw2char(Code);
            writer->advance(1);
            *(writer->getWritePointer()) = ' ';
            writer->advance(1);
            // Start new character
            Code = 1;
        }
        Stop = 1;
    }

    // Update state
    RealState0 = RealState;
    FiltState0 = FiltState;
    MagTotal  += Magnitude;

    // Update time
    curSamples += buckets;
    if(curSamples>=sampleRate)
    {
        unsigned int secs = curSamples/sampleRate;
        curTime += secs;
        curSamples -= secs*sampleRate;
    }
}