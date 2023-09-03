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

#include "cw.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

using namespace Csdr;

template <typename T>
const char CwDecoder<T>::cwTable[] =
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

template <typename T>
CwDecoder<T>::CwDecoder(unsigned int sampleRate, unsigned int targetFreq, unsigned int targetWidth)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  quTime(5),      // Quantization step (ms)
  nbTime(20),     // Noise blanking width (ms)
  dbgTime(0),     // Debug printout period (ms)
  showCw(false)   // TRUE: print DITs/DAHs
{
    buckets = sampleRate/targetWidth; // Number of FFT buckets
    step    = quTime*sampleRate/1000; // Quantization step in samples

    // Goertzel algorithm coefficient
    double v = round((double)buckets * targetFreq / sampleRate);
    coeff = 2.0 * cos(2.0 * M_PI * v / buckets);
}

template <typename T>
bool CwDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=buckets) && (this->writer->writeable()>=2);
}

template <typename T>
void CwDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    // Until we run out of input data...
    for(; this->reader->available()>=buckets ; this->reader->advance(step)) {
        // Process data
        processInternal(this->reader->getReadPointer(), buckets);

        // Update time
        curSamples += step;
        if(curSamples>=sampleRate)
        {
            unsigned int secs = curSamples/sampleRate;
            curSeconds += secs;
            curSamples -= secs*sampleRate;
        }
    }
}

template <typename T>
void CwDecoder<T>::processInternal(const T *data, unsigned int size) {
    unsigned long millis = msecs();
    double q0, q1, q2;
    unsigned int i, j;

    // Read samples
    for(i=0, q1=q2=0.0 ; i<size ; ++i)
    {
        q0 = q1 * coeff - q2 + sample2double(data[i]);
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

            // If we have got some DITs or DAHs...
            if(code>1)
            {
                // If a letter BREAK...
                if((durationL>=5*avgBrkT/2) && (durationL<5*avgBrkT))
                {
                    // Print character
                    *(this->writer->getWritePointer()) = cw2char(code);
                    this->writer->advance(1);

                    // Start new character
                    code = 1;
                }
                // If a word BREAK...
                else if(durationL>=5*avgBrkT)
                {
                    // Print character
                    *(this->writer->getWritePointer()) = cw2char(code);
                    this->writer->advance(1);

                    // Print word BREAK
                    *(this->writer->getWritePointer()) = ' ';
                    this->writer->advance(1);

                    // Start new character
                    code = 1;
                }
            }

            // Keep track of the average small BREAK duration
            if((durationL>20) && (durationL<3*avgDitT/2) && (durationL>2*avgDitT/3))
                avgBrkT += (int)(durationL - avgBrkT)/10;
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

            // 2/3 to filter out false DITs
            if((durationH<3*avgDitT/2) && (durationH>avgDitT/2))
            {
                // Add a DIT to the code
                code = (code<<1) | 1;

                // Print a DIT
                if(showCw)
                {
                    *(this->writer->getWritePointer()) = '.';
                    this->writer->advance(1);
                }
            }
            else if((durationH<3*avgDahT) && (durationH>2*avgDahT/3))
            {
                // Add a DAH to the code
                code = (code<<1) | 0;

                // Try computing WPM
                wpm = (wpm + (1200/(durationH/3)))/2;

                // Print a DAH
                if(showCw)
                {
                    *(this->writer->getWritePointer()) = '-';
                    this->writer->advance(1);
                }
            }

            // Keep track of the average DIT duration
            if((durationH>20) && (durationH<2*avgDitT))
                avgDitT += (int)(durationH - avgDitT)/10;

            // Keep track of the average DAH duration
            if((durationH<300) && (durationH>5*avgDitT/2))
                avgDahT += (int)(durationH - avgDahT)/10;
        }
    }

    // If no more characters...
    if(((millis-startTimeL)>6*durationH) && !stop)
    {
        // If there is a buffered code...
        if(code>1)
        {
            // Print character
            *(this->writer->getWritePointer()) = cw2char(code);
            this->writer->advance(1);

            // Print word break
            *(this->writer->getWritePointer()) = ' ';
            this->writer->advance(1);

            // Start new character
            code = 1;
        }

        stop = 1;
    }

    // Periodically print debug information, if enabled
    if(dbgTime && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }

    // Update state
    realState0 = realState;
    filtState0 = filtState;
}

template <typename T>
void CwDecoder<T>::printDebug()
{
    char buf[256];
    int i, j;

    // Number of histogram entries
    i = sizeof(histH) / sizeof(histH[0]);

    // Draw HIGH/LOW duration histograms
    for(j=0 ; j<i ; ++j)
    {
        int h = 10 * histH[j] / (histCntH+1);
        int l = 10 * histL[j] / (histCntL+1);

        buf[j+2]   = h>9? '*' : h>0? '0'+h : histH[j]>0? '0' : '-';
        buf[j+i+3] = l>9? '*' : l>0? '0'+l : histL[j]>0? '0' : '-';
        histH[j] = histL[j] = 0;
    }

    // Complete histograms
    histCntH = histCntL = 0;
    buf[0]   = '\n';
    buf[1]   = '[';
    buf[i+2] = '|';

    // Create complete string to print
    sprintf(buf+2*i+3, "] [%d-%d .%ld -%ld _%ldms WPM%d]\n", (int)magL, (int)magH, avgDitT, avgDahT, avgBrkT, wpm);

    // Print
    printString(buf);
}

template <typename T>
void CwDecoder<T>::printString(const char *buf)
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

template <>
inline double CwDecoder<complex<float>>::sample2double(complex<float> input)
{
    return input.i();
}

template<>
inline double CwDecoder<float>::sample2double(float input)
{
    return input;
}

namespace Csdr {
    template class CwDecoder<complex<float>>;
    template class CwDecoder<float>;
}
