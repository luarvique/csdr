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

#pragma once

#include "filter.hpp"
#include "complex.hpp"
#include "window.hpp"
#include "vkfft.hpp"

#include <fftw3.h>

namespace Csdr {

    template <typename T>
    class FftFilter: public Filter<T> {
        public:
            FftFilter(size_t fftSize, complex<float>* taps, size_t taps_length);
            ~FftFilter() override;
            size_t apply(T* input, T* output, size_t size) override;
            size_t getMinProcessingSize() override { return inputSize; }
        protected:
            explicit FftFilter(size_t fftSize);
            static size_t filterLength(float transition);
            static size_t getFftSize(size_t taps_length);
            complex<float>* taps;
            size_t taps_length;
            size_t fftSize;
            size_t inputSize;
        private:
            fftwf_complex* forwardInput;
            fftwf_complex* forwardOutput;
            fftwf_plan forwardPlan;
            fftwf_complex* inverseInput;
            fftwf_complex* inverseOutput;
            fftwf_plan inversePlan;
            T* overlap;

            VkFFTBackend* vkBackend;
    };

    class FftBandPassFilter: public FftFilter<complex<float>> {
        public:
            FftBandPassFilter(float lowcut, float highcut, float transition, Window* window);
    };

}