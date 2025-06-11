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

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

template <typename T>
SmartSquelch<T>::SmartSquelch(size_t length, size_t hangLength, size_t flushLength, std::function<void(float)> callback)
: length(length),
  hangLength(hangLength),
  flushLength(flushLength),
  callback(std::move(callback)),
  hangCount(0),
  flushCount(0)
{
    fftInput  = fftwf_alloc_complex(length);
    fftOutput = fftwf_alloc_complex(length);
    fftPlan   = fftwf_plan_dft_1d(length, fftInput, fftOutput, FFTW_FORWARD, CSDR_FFTW_FLAGS);
}

template<typename T>
SmartSquelch<T>::~SmartSquelch() {
    fftwf_destroy_plan(fftPlan);
    fftwf_free(fftInput);
    fftwf_free(fftOutput);
}

template <typename T>
void SmartSquelch<T>::setSquelch(float squelchLevel) {
    this->squelchLevel = squelchLevel;
}

template <typename T>
bool SmartSquelch<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=length) && (this->writer->writeable()>=length);
}

template <typename T>
void SmartSquelch<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    const T *in = this->reader->getReadPointer();
    T *out = this->writer->getWritePointer();
    float avg, peak;
    size_t j;

    // Copy data into the input buffer, computing power
    auto* data = (complex<float>*) fftInput;
    for (j=0 ; j < length ; ++j) data[j] = in[j];

    // Calculate FFT on input buffer
    fftwf_execute(fftPlan);

    for (avg=peak=0.0, j=0 ; j < length ; ++j) {
        float v = fftOutput[j][0]*fftOutput[j][0] + fftOutput[j][1]*fftOutput[j][1];
        avg += v;
        peak = std::max(v, peak);
    }

    // Compute average and peak power
    avg  /= length;
    peak -= avg;

    // Report power
    if (callback) callback(peak);

    bool squelchOpen = (squelchLevel == 0.0) || (peak >= squelchLevel);

    // Hang with open squelch for a while to prevent dropouts
    if(squelchOpen) hangCount = 0;
    else if(hangCount < hangLength)
    {
      hangCount += length;
      squelchOpen = true;
    }

    // If squelch is open...
    if (squelchOpen) {
        memcpy(out, in, length * sizeof(T));
        this->writer->advance(length);
        flushCount = 0;
    // Squelch closed, flushing some zeros...
    } else if (flushCount < flushLength) {
        size_t l = std::min(length, flushLength - flushCount);
        memset(out, 0, l * sizeof(T));
        this->writer->advance(l);
        flushCount += l;
    }

    // Done with input
    this->reader->advance(length);
}

namespace Csdr {
    template class SmartSquelch<complex<float>>;
    template class SmartSquelch<float>;
}
