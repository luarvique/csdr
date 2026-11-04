
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

#include "limit.hpp"
#include <cmath>

using namespace Csdr;

template <typename T>
Limit<T>::Limit(float maxAmplitude): maxAmplitude(maxAmplitude) {}

template <typename T>
void Limit<T>::process(T *input, T *output, size_t size) {
    for (size_t i = 0; i < size; i++)  output[i] = limit(input[i]);
}

template <>
float Limit<float>::limit(float in) {
    return std::min(maxAmplitude, std::max(-maxAmplitude, in));
}

template <>
complex<float> Limit<complex<float>>::limit(complex<float> in) {
    return complex<float>(std::complex<float>(
        std::min(maxAmplitude, std::max(-maxAmplitude, in.i())),
        std::min(maxAmplitude, std::max(-maxAmplitude, in.q()))
    ));
}

namespace Csdr {
    template class Limit<float>;
    template class Limit<complex<float>>;
}
