/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2025 Marat Fayzullin <luarvique@gmail.com>
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

#include "smartsquelch.hpp"
#include <cmath>
#include <cstring>

using namespace Csdr;

template <typename T>
SmartSquelch<T>::SmartSquelch(size_t length, size_t flushLength)
: length(length),
  flushLength(flushLength)
{
    unsigned int i;

    buckets = 4;

    // Goertzel algorithm coefficients
    double v1 = round((double)buckets * targetFreq / sampleRate);
    double v2 = round((double)buckets * (targetFreq + targetWidth) / sampleRate);
    coeff1 = 2.0 * cos(2.0 * M_PI * v1 / buckets);
    coeff2 = 2.0 * cos(2.0 * M_PI * v2 / buckets);
}

template <typename T>
bool SmartSquelch<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=length) && (this->writer->writeable()>=length);
}

template <typename T>
void SmartSquelch<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    // Process input data
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

namespace Csdr {
    template class SmartSquelch<complex<float>>;
    template class SmartSquelch<float>;
}
