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
#include <cstdio>
#include <cmath>

using namespace Csdr;

template <typename T>
SmartSquelch<T>::SmartSquelch(size_t length, size_t hangLength, size_t flushLength, std::function<void(float)> callback)
: length(length),
  hangLength(hangLength),
  flushLength(flushLength),
  callback(std::move(callback))
{
    // Not flushing anything yet
    flushCount = hangCount = 0;

    // Goertzel algorithm coefficients
    coeff1 = 2.0 * cos(2.0 * M_PI * 0.1); // bucket 3/5
    coeff2 = 2.0 * cos(2.0 * M_PI * 0.4); // bucket 4/5
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

    float q10, q11 = 0.0, q12 = 0.0;
    float q20, q21 = 0.0, q22 = 0.0;
    float q30, q31 = 0.0, q32 = 0.0;
    float power = 0.0;

    float coeff1 = 2.0 * cos(2.0 * M_PI * 0.08);
    float coeff2 = 2.0 * cos(2.0 * M_PI * 0.14);
    float coeff3 = 2.0 * cos(2.0 * M_PI * 0.20);

    // Compute power and bucket values
    for (size_t j=0 ; j < length ; ++j) {
        power += std::norm(in[j]);

        q10 = std::norm(in[j]) + q11 * coeff1 - q12;
        q12 = q11;
        q11 = q10;

        q20 = std::norm(in[j]) + q21 * coeff2 - q22;
        q22 = q21;
        q21 = q20;

        q30 = std::norm(in[j]) + q31 * coeff3 - q32;
        q32 = q31;
        q31 = q30;
    }

    // Report average power
    if (callback) callback(power / length);

    // We only need the real part
    float mag1 = q11*q11 + q12*q12 - q11*q12*coeff1;
    float mag2 = q21*q21 + q22*q22 - q21*q22*coeff2;
    float mag3 = q31*q31 + q32*q32 - q31*q32*coeff3;

    bool squelchOpen =
        (mag1 > 20.0 * (mag2 + mag3)) ||
        (mag2 > 20.0 * (mag1 + mag3)) ||
        (mag3 > 20.0 * (mag1 + mag2));

    // Open squelch if one bucket is much higher than the other
    //bool squelchOpen = (mag1 > 100000.0*mag2) || (mag2 > 100000.0*mag1);

//fprintf(stderr, "power = %f, %f vs %f => %f => %s\n",
//  log10(power/length),
//  log10(mag1), log10(mag2),
//  mag1/mag2,
//  squelchOpen? "ON":"OFF"
//);

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
