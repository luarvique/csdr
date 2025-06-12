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

#include "snr.hpp"
#include <cstring>
#include <cmath>

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

template <typename T>
Snr<T>::Snr(size_t length, size_t fftSize, std::function<void(float)> callback)
: callback(std::move(callback))
{
    this->fftSize = fftSize = fftSize >= 64? fftSize: 64;
    this->length  = length >= fftSize? length : fftSize;

    fftInput  = fftwf_alloc_complex(fftSize);
    fftOutput = fftwf_alloc_complex(fftSize);
    fftPlan   = fftwf_plan_dft_1d(fftSize, fftInput, fftOutput, FFTW_FORWARD, CSDR_FFTW_FLAGS);
}

template<typename T>
Snr<T>::~Snr() {
    fftwf_destroy_plan(fftPlan);
    fftwf_free(fftInput);
    fftwf_free(fftOutput);
}

template <typename T>
bool Snr<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    size_t length = this->getLength();
    return (this->reader->available() > length && this->writer->writeable() > length);
}

template <typename T>
void Snr<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    T *input = this->reader->getReadPointer();
    float avg, snr;
    size_t j;

    // Copy data into the input buffer
    auto* data = (complex<float>*) fftInput;
    for (j=0 ; j < fftSize ; ++j) data[j] = input[j];

    // Calculate FFT on input buffer
    fftwf_execute(fftPlan);

    for (avg=snr=0.0, j=0 ; j < fftSize ; ++j) {
        float v = fftOutput[j][0]*fftOutput[j][0] + fftOutput[j][1]*fftOutput[j][1];
        snr  = std::max(v, snr);
        avg += v;
    }

    // Compute average and peak power
    avg /= fftSize;
    snr /= avg;

    // Report peak power over average
    if (callback) callback(snr);

    // Pass data
    forwardData(input, snr);

    // Advance input
    this->reader->advance(length);
}

template <typename T>
size_t Snr<T>::getLength() {
    return length;
}

template <typename T>
void Snr<T>::forwardData(T* input, float snr) {
    T* output = this->writer->getWritePointer();
    size_t length = this->getLength();
    std::memcpy(output, input, length * sizeof(T));
    this->writer->advance(length);
}

template <typename T>
SnrSquelch<T>::SnrSquelch(size_t length, size_t fftSize, size_t hangLength, size_t flushLength, std::function<void(float)> callback)
: Snr<T>(length, fftSize, callback),
  hangLength(hangLength),
  flushLength(flushLength),
  callback(std::move(callback))
{}

template <typename T>
void SnrSquelch<T>::setSquelch(float squelchLevel) {
    this->squelchLevel = squelchLevel;
}

template <typename T>
void SnrSquelch<T>::forwardData(T *input, float snr) {
printf("@@@ SNR = %f, SQL = %f\n", snr, squelchLevel);
fflush(stdout);
    if (squelchLevel == 0.0f || snr >= squelchLevel) {
        Snr<T>::forwardData(input, snr);
        flushCounter = hangCounter = 0;
    } else if (hangCounter < hangLength) {
        // Keep forwarding for a while in case signal comes back
        Snr<T>::forwardData(input, snr);
        hangCounter += this->getLength();
    } else if (flushCounter < flushLength) {
        // Produce some 0s to flush any subsequent modules
        // if they have any overhead (e.g. FIR filter delays)
        T* output = this->writer->getWritePointer();
        size_t length = std::min(this->getLength(), flushLength - flushCounter);
        std::memset(output, 0, sizeof(T) * length);
        this->writer->advance(length);
        flushCounter += length;
    }
}

namespace Csdr {
    template class Snr<complex<float>>;
    template class Snr<float>;
    template class SnrSquelch<complex<float>>;
    template class SnrSquelch<float>;
}
