/*
Copyright (c) 2023 Jakob Ketterl <jakob.ketterl@gmx.de>

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
#include "complex.hpp"

namespace Csdr {

    class PhaseDemod: public AnyLengthModule<complex<float>, float> {
        public:
            void process(complex<float>* input, float* output, size_t work_size) override;
        protected:
            size_t maxLength() override { return buffer_size; }
        private:
            size_t buffer_size = 1024;
    };

}