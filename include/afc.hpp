/*
Copyright (c) 2023 Marat Fayzullin <luarvique@gmail.com>

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

#include "module.hpp"
#include "shift.hpp"

#include <fftw3.h>

namespace Csdr {

    class Afc: public ShiftAddfast {
        public:
            Afc(unsigned int updatePeriod = 4);
            ~Afc();

        protected:
            void process(complex<float>* input, complex<float>* output) override;

        private:
            // Configuration
            unsigned int updatePeriod; // Update period

            // State
            unsigned int updateCount;  // Update counter
            double curShift;           // Current frequency correction

            // FFT setup
            fftwf_complex *fftIn;
            fftwf_complex *fftOut;
            fftwf_plan fftPlan;
    };
}
