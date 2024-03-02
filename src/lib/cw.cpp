/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2024 Marat Fayzullin <luarvique@gmail.com>
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
    "__TEMNAIOGKDWRUS" // 00000000
    "__QZYCXBJP_L_FVH"
    "09_8_<_7_(___/-6" // <AR>
    "1______&2___3_45"
    "_______:____,___" // 01000000
    "__)_!;________-_"
    "_'___@____._____"
    "___?______{_____" // <SK>
    "________________" // 10000000
    "________________"
    "________________"
    "________________"
    "________________" // 11000000
    "________________"
    "________________"
    "______$_________";

template <typename T>
CwDecoder<T>::CwDecoder(unsigned int sampleRate, bool showCw)
: sampleRate(sampleRate),
  quTime(5),      // Quantization step (ms)
  nbTime(20),     // Noise blanking width (ms)
  dbgTime(0),     // Debug printout period (ms)
  showCw(showCw)  // TRUE: print DITs/DAHs
{
    // Minimal number of samples to process
    quStep = quTime * sampleRate / 1000;
}

template <typename T>
void CwDecoder<T>::reset() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    // Input signal characteristics
    magL = 1000.0;  // Minimal observed magnitude
    magH = 0.0;     // Maximal observed magnitude
    realState0 = 0; // Last unfiltered signal state (0/1)
    filtState0 = 0; // Last filtered signal state (0/1)

    // HIGH / LOW timing
    lastStartT = 0; // Time of the last signal change (ms)
    startTimeH = 0; // Time HIGH signal started (ms)
    durationH  = 0; // Duration of the HIGH signal (ms)
    startTimeL = 0; // Time LOW signal started (ms)
    durationL  = 0; // Duration of the LOW signal (ms)

    // DIT / DAH / BREAK timing
    avgDitT = 50;   // Average DIT signal duration (ms)
    avgDahT = 100;  // Average DAH signal duration (ms)
    avgBrkT = 50;   // Average BREAK duration (ms)

    // Current CW code
    code = 1;       // Currently accumulated CW code or 1
    stop = 0;       // 1 if there is a code pending
    wpm  = 0;       // Current CW speed (in wpm)
}

template <typename T>
bool CwDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=quStep) && (this->writer->writeable()>=2);
}

template <typename T>
void CwDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    // Until we run out of input data...
    for(; this->reader->available()>=quStep ; this->reader->advance(quStep)) {
        // Process data
        processInternal(this->reader->getReadPointer(), quStep);

        // Update time
        curSamples += quStep;
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
    unsigned int i, j;
    double magnitude;

    // Compute overall magnitude
    for(i=0, magnitude=0.0 ; i<size ; ++i)
        magnitude += sample2level(data[i]);
    magnitude /= size;

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
inline double CwDecoder<complex<float>>::sample2level(complex<float> input)
{
    return sqrt((input.i() * input.i()) + (input.q() * input.q()));
}

template<>
inline double CwDecoder<float>::sample2level(float input)
{
    return fabs(input);
}

namespace Csdr {
    template class CwDecoder<complex<float>>;
    template class CwDecoder<float>;
}
