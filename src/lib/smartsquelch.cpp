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
SmartSquelch<T>::SmartSquelch(size_t length, size_t flushLength, std::function<void(float)> callback)
: length(length),
  flushLength(flushLength),
  callback(std::move(callback))
{
    // Not flushing anything yet
    flushCount = 0;

    // Goertzel algorithm coefficients
    coeff1 = 2.0 * cos(2.0 * M_PI * 0.0); // bucket 0/2
    coeff2 = 2.0 * cos(2.0 * M_PI * 0.5); // bucket 1/2
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

    double q10, q11, q12;
    double q20, q21, q22;
    float power = 0.0f;

    // Compute power and bucket values
    for (size_t j=0, q11=q12=q21=q22=0.0 ; j < length ; ++j) {
        power += std::norm(in[j]);

        q10 = q11 * coeff1 - q12 + std::norm(in[j]);
        q12 = q11;
        q11 = q10;
        q20 = q21 * coeff2 - q22 + std::norm(in[j]);
        q22 = q21;
        q21 = q20;
    }

    // Report average power
    if (callback) callback(power / length);

    // We only need the real part
    double mag1 = q11*q11 + q12*q12 - q11*q12*coeff1;
    double mag2 = q21*q21 + q22*q22 - q21*q22*coeff2;

    // Open squelch if one bucket is much higher than the other
    bool squelchOpen = (mag1 > 4.0*mag2) || (mag2 > 4.0*mag1);

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
