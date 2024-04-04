/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2021 Jakob Ketterl <jakob.ketterl@gmx.de>
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

#include "fftfilter.hpp"
#include "fir.hpp"

#include <cstring>

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

template <typename T>
FftFilter<T>::FftFilter(size_t fftSize):
    fftSize(fftSize),
    forwardInput(fftwf_alloc_complex(fftSize)),
    forwardOutput(fftwf_alloc_complex(fftSize)),
    forwardPlan(fftwf_plan_dft_1d(fftSize, forwardInput, forwardOutput, FFTW_FORWARD, CSDR_FFTW_FLAGS)),
    inverseInput(fftwf_alloc_complex(fftSize)),
    inverseOutput(fftwf_alloc_complex(fftSize)),
    inversePlan(fftwf_plan_dft_1d(fftSize, inverseInput, inverseOutput, FFTW_BACKWARD, CSDR_FFTW_FLAGS)),
    overlap((T*) calloc(sizeof(T), fftSize))
{
    // fill with zeros so that the padding works
    for (size_t i = 0; i < fftSize; i++) {
        forwardInput[i][0] = 0.0f;
        forwardInput[i][1] = 0.0f;
    };

    vkBackend = new VkFFTBackend(fftSize);
}

template <typename T>
FftFilter<T>::FftFilter(size_t fftSize, complex<float> *taps, size_t taps_length): FftFilter(fftSize) {
    this->taps = taps;
    this->taps_length = taps_length;
    inputSize = fftSize - taps_length + 1;
}

template<typename T>
FftFilter<T>::~FftFilter() {
    free(taps);
    fftwf_destroy_plan(forwardPlan);
    fftwf_free(forwardInput);
    fftwf_free(forwardOutput);
    fftwf_destroy_plan(inversePlan);
    fftwf_free(inverseInput);
    fftwf_free(inverseOutput);
    free(overlap);

    delete vkBackend;
}

template<typename T>
size_t FftFilter<T>::apply(T *input, T *output, size_t size) {
    // use the overlap & add method for filtering

    // copy input but only partially fill fft input
    std::memcpy(forwardInput, input, sizeof(T) * inputSize);

    // calculate FFT on input buffer
    if (vkBackend->isReady()) {
        VkFFTResult  res = vkBackend->fft(forwardInput, forwardOutput, -1);
        if (res != VKFFT_SUCCESS) {
            std::cerr << "vkfft failed " << res << "\n";
        }
    } else {
        fftwf_execute(forwardPlan);
    }

    auto* in = (complex<float>*) forwardOutput;
    auto* out = (complex<float>*) inverseInput;

    // multiply the filter and the input
    for (size_t i = 0; i < fftSize; i++) {
        out[i] = in[i] * taps[i];
    }

    // calculate inverse FFT on multiplied buffer
    if (vkBackend->isReady()) {
        VkFFTResult  res = vkBackend->fft(inverseInput, inverseOutput, 1);
        if (res != VKFFT_SUCCESS) {
            std::cerr << "inverse vkfft failed " << res << "\n";
        }
    } else {
        fftwf_execute(inversePlan);
    }

    // add the overlap of the previous segment
    auto result = (complex<float>*) inverseOutput;

    for (size_t i = 0; i < fftSize; i++) {
        result[i] /= fftSize;
    }

    for (size_t i = 0; i < taps_length - 1; i++) {
        result[i] = result[i] + overlap[i];
    }

    std::memcpy(overlap, result + inputSize, sizeof(complex<float>) * (taps_length - 1));

    // copy input but only partially fill fft input
    std::memcpy(output, result, sizeof(T) * inputSize);

    return inputSize;
}

template <typename T>
size_t FftFilter<T>::filterLength(float transition) {
    size_t result = 4.0 / transition;
    if (result % 2 == 0) result++; //number of symmetric FIR filter taps should be odd
    return result;
}

template <typename T>
size_t FftFilter<T>::getFftSize(size_t taps_length) {
    size_t fft_size = 1;
    while (fft_size < taps_length) fft_size <<= 1;
    //the number of padding zeros is the number of output samples we will be able to take away after every processing step, and it looks sane to check if it is large enough.
    while (fft_size - taps_length < 200) fft_size <<= 1;
    return fft_size;
}

FftBandPassFilter::FftBandPassFilter(float lowcut, float highcut, float transition, Window* window):
    FftFilter<complex<float>>(FftBandPassFilter::getFftSize(FftBandPassFilter::filterLength(transition)))
{
    taps_length = FftBandPassFilter::filterLength(transition);
    auto generator = new BandPassTapGenerator(lowcut, highcut, window);
    taps = generator->generateFftTaps(taps_length, fftSize);
    delete generator;
    inputSize = fftSize - taps_length + 1;
}

namespace Csdr {
    template class FftFilter<complex<float>>;
}
