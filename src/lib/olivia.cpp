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

#include "olivia.hpp"
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cstdio>

using namespace Csdr;

template <typename T>
OliviaDecoder<T>::OliviaDecoder(unsigned int sampleRate, unsigned int centerFreq, unsigned int symbols, unsigned int bandwidth)
: sampleRate(sampleRate),
  centerFreq(centerFreq),
  symbols(symbols),
  bandwidth(bandwidth)
{
    baud = bandwidth / symbols;
    wlen = (sampleRate + baud - 1) / baud;
}

template <typename T>
OliviaDecoder<T>::~OliviaDecoder() {
}

template <typename T>
bool OliviaDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return this->reader->available() >= wlen;
}

template <typename T>
void OliviaDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    const T *ptr = this->reader->getReadPointer();
    unsigned int size = this->reader->available();
    unsigned int j, i, k, symbol;

    // Must have at least wlen samples
    if (size < wlen) return;

    // @@@ Do FFT here!
    fft = ???;

    // Find the most likely symbol
    i = centerFreq - bandwidth / 2 + baud / 2;
    for (j=0, k=0, symbol=0 ; j<symbols ; ++j, i+=baud) {
        unsigned int v = abs(fft[i * wlen / sampleRate]);
        if (v > k) { k = v; symbol = j; }
    }

    // Decode and append symbol
    addSymbol(degray(maxJ));

    // Done with input data
    this->reader->advanceReadPointer(wlen);
}

template <typename T>
bool OliviaDecoder<T>::decodeBlock(const unsigned int *buf) {



}

template <typename T>
void OliviaDecoder<T>::addSymbol(unsigned int v) {
  buf[bufCount++] = v;
  if (bufCount>=64) {
      if (decodeBlock(buf)) {
          // Buffer decoded successfully
          bufCount = 0;
      } else {
          // Try rolling
          memcpy(&buf[0], &buf[1], (bufCount - 1) * sizeof(buf[0]));
          bufCount -= 1;
      }
  }

}

template <typename T>
unsigned int OliviaDecoder<T>::degray(unsigned int v) {
    unsigned int mask;
    for (mask=v ; mask ; v^=mask) mask>>=1;
    return v;
}

template <>
inline double OliviaDecoder<complex<float>>::sample2double(complex<float> input)
{
    return input.i();
}

template<>
inline double OliviaDecoder<float>::sample2double(float input)
{
    return input;
}

namespace Csdr {
    template class OliviaDecoder<complex<float>>;
    template class OliviaDecoder<float>;
}
