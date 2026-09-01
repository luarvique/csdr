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

#pragma once

#include <functional>
#include "module.hpp"
#include <fftw3.h>

namespace Csdr {

    template <typename T>
    class Snr: public Module<T, T> {
        public:
            Snr(size_t length, size_t fftSize = 0, std::function<void(float)> callback = 0);
            ~Snr() override;

            size_t getLength();
            bool canProcess() override;
            void process() override;

            void setAttackDecay(float attack, float decay);

        protected:
            // to be overridden by the squelch implementation
            virtual void forwardData(T* input, float snr);

        private:
            size_t length;        // Length of data to measure over
            size_t fftSize;       // Number of FFT bins (<= length)
            size_t wndSize;       // FFT window width (<= fftSize/2)

            std::function<void(float)> callback;

            double attack = 0.3;  // Noise floor and signal peak attack rate
            double decay  = 0.05; // Noise floor and signal peak decay rate
            double peak   = 0.0;  // Current signal peak
            double floor  = 0.0;  // Current noise floor

            fftwf_complex* fftInput;
            fftwf_complex* fftOutput;
            fftwf_plan fftPlan;
            float *inputWindow;
    };

    template <typename T>
    class SnrSquelch: public Snr<T> {
        public:
            SnrSquelch(size_t length, size_t fftSize = 0, size_t hangLength = 0, size_t flushLength = 0, std::function<void(float)> callback = 0);
            void setThreshold(float dBthreshold);

        protected:
            void forwardData(T* input, float snr) override;

        private:
            size_t hangLength;  // Number of samples to keep after signal stops
            size_t flushLength; // Number of empty samples after signal stops

            float  threshold    = 0.0f; // SNR level that opens squelch
            size_t hangCounter  = 0;
            size_t flushCounter = 0;
    };
}
