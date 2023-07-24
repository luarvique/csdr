/*
Copyright (c) 2021 Jakob Ketterl <jakob.ketterl@gmx.de>

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

#include "converter.hpp"
#include "complex.hpp"

#include <climits>

using namespace Csdr;

template <> void Converter<float, short>::process(float* input, short* output, size_t length) {
    for (int i = 0; i < length; i++) {
         output[i] = input[i] * SHRT_MAX;
    }
}

template <> void Converter<short, float>::process(short* input, float* output, size_t length) {
    for (int i = 0; i < length; i++) {
        output[i] = (float) input[i] / SHRT_MAX;
    }
}

template <>
void Converter<complex<float>, complex<short>>::process(complex<float> *input, complex<short> *output, size_t length) {
    for (int i = 0; i < length * 2; i++) {
        ((short*) output)[i] = ((float*) input)[i] * SHRT_MAX;
    }
}

template <>
void Converter<complex<short>, complex<float>>::process(complex<short> *input, complex<float>* output, size_t length) {
    for (int i = 0; i < length * 2; i++) {
        ((float*) output)[i] = (float) ((short*) input)[i] / SHRT_MAX;
    }
}

template <> void Converter<float, unsigned char>::process(float* input, unsigned char* output, size_t length) {
    for (int i = 0; i < length; i++) {
         output[i] = (input[i] + 1.0) * UCHAR_MAX / 2;
    }
}

template <>
void Converter<complex<float>, complex<unsigned char>>::process(complex<float> *input, complex<unsigned char> *output, size_t length) {
    for (int i = 0; i < length * 2; i++) {
        ((unsigned char*) output)[i] = (((float*) input)[i] + 1.0) * UCHAR_MAX / 2;
    }
}
