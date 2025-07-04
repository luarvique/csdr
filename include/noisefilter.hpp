/*
Copyright (c) 2022 Marat Fayzullin <luarvique@gmail.com>

This file is part of libcsdr.

libcsdr is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libcsdr is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with libcsdr.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "filter.hpp"
#include "complex.hpp"

#include <fftw3.h>

namespace Csdr {

    template <typename T>
    class NoiseFilter: public Filter<T> {
        public:
            NoiseFilter(size_t fftSize = 1024, size_t wndSize = 16, unsigned int decay = 10, unsigned int attack = 2);
            ~NoiseFilter() override;

            size_t apply(T* input, T* output, size_t size) override;
            size_t getMinProcessingSize() override { return fftSize-ovrSize; }

            void setThreshold(int dBthreshold);

        protected:
            size_t fftSize;   // Size of the FFT
            size_t wndSize;   // Actually, half-a-window
            size_t ovrSize;   // Usually 1/64th of fftSize
            double threshold; // Filtering threshold
            double avgPower;  // Power over multiple FFTs
            double attack;    // Attack rate measuring avgPower
            double decay;     // Decay rate measuring avgPower

        private:
            fftwf_complex* forwardInput;
            fftwf_complex* forwardOutput;
            fftwf_plan forwardPlan;
            fftwf_complex* inverseInput;
            fftwf_complex* inverseOutput;
            fftwf_plan inversePlan;
            fftwf_complex* overlapBuf;

            // Convert output complex into a sample
            inline T complex2sample(complex<float> input);
    };

    class AFNoiseFilter: public NoiseFilter<float> {
        public:
            AFNoiseFilter(size_t fftSize = 1024, size_t wndSize = 16, unsigned int decay = 10, unsigned int attack = 2):
                NoiseFilter<float>(fftSize, wndSize, decay, attack) {}
    };
}
