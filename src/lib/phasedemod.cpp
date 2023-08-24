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

#include "phasedemod.hpp"

using namespace Csdr;

void PhaseDemod::process(complex<float> *input, float *output, size_t work_size) {
    for (size_t i = 0; i < work_size; i++) {
        output[i] = std::arg(input[i]);
    }

    for (size_t i = 0; i < work_size; i++) {
        if (std::isnan(output[i])) output[i] = 0.0f;
    }
}