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

#include "sstv.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

using namespace Csdr;

template <typename T>
SstvDecoder<T>::SstvDecoder(unsigned int sampleRate, unsigned int targetFreq)
: sampleRate(sampleRate),
  targetFreq(targetFreq),
  dbgTime(0),     // Debug printout period (ms)
{
    buckets = sampleRate/targetWidth; // Number of FFT buckets
    step    = quTime*sampleRate/1000; // Quantization step in samples
    buf     = new float[buckets];     // Temporary sample buffer
}

template <typename T>
SstvDecoder<T>::~SstvDecoder() {
    if(buf) { delete[] buf;buf=0; }
}

template <typename T>
bool SstvDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=(buckets-bufPos)) && (this->writer->writeable()>=2);
}

template <typename T>
void SstvDecoder<T>::process() {
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
void SstvDecoder<T>::processInternal(float *data, unsigned int size) {
    unsigned long millis = msecs();

    // @@@ TODO!

    // Periodically print debug information, if enabled
    if(dbgTime && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }
}

template <typename T>
void SstvDecoder<T>::printDebug()
{
    // @@@ TODO!
}

template <typename T>
void SstvDecoder<T>::printString(const char *buf)
{
    // If there is enough output buffer available...
    if(this->writer->writeable()>=strlen(buf))
    {
        // Place each string character into the output buffer
        for(int j=0 ; buf[j] ; ++j)
        {
            *(this->writer->getWritePointer()) = buf[j];
            this->writer->advance(1);
        }
    }
}

namespace Csdr {
    template class SstvDecoder<complex<float>>;
    template class SstvDecoder<float>;
}
