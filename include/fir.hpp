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

#include "window.hpp"
#include "complex.hpp"
#include "module.hpp"
#include "filter.hpp"
#include "fftfilter.hpp"

namespace Csdr {

    template <typename T, typename U>
    class FirFilter: public SampleFilter<T> {
        public:
            FirFilter(U* taps, size_t length);
            ~FirFilter();
            T processSample(T* data, size_t index) override;
            T processSample_fmv(T* data, size_t index);
            size_t getOverhead() override;
        protected:
            explicit FirFilter(size_t length);
            static size_t filterLength(float transition);
            void allocateTaps(size_t length);
            U* taps;
            size_t taps_length;
    };

    template <typename T>
    class TapGenerator {
        public:
            explicit TapGenerator(Window* window);
            virtual T* generateTaps(size_t length) = 0;
            complex<float>* generateFftTaps(size_t length, size_t fftSize);
        protected:
            void normalize(T* taps, size_t length);
            Window* window;
    };

    class LowPassTapGenerator: public TapGenerator<float> {
        public:
            LowPassTapGenerator(float cutoff, Window* window);
            float* generateTaps(size_t length) override;
        private:
            float cutoff;
    };

    template <typename T>
    class LowPassFilter: public FirFilter<T, float> {
        public:
            LowPassFilter(float cutoff, float transition, Window* window);
    };

    class BandPassTapGenerator: public TapGenerator<complex<float>> {
        public:
            BandPassTapGenerator(float lowcut, float highcut, Window* window);
            complex<float>* generateTaps(size_t length) override;
        private:
            float lowcut;
            float highcut;
    };

    template <typename T>
    class BandPassFilter: public FirFilter<T, complex<float>> {
        public:
            BandPassFilter(float lowcut, float highcut, float transition, Window* window);
    };
}