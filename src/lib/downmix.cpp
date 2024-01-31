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

#include "downmix.hpp"

using namespace Csdr;

template <typename T>
Downmix<T>::Downmix(unsigned int channels): channels(channels) {}

template <typename T>
bool Downmix<T>::canProcess() {
    return this->reader->available() >= channels && this->writer->writeable() > 0;
}

template <typename T>
void Downmix<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    size_t numSamples = std::min(this->reader->available() / channels, this->writer->writeable());
    T* input = this->reader->getReadPointer();
    T* output = this->writer->getWritePointer();
    for (size_t i = 0; i < numSamples; i++) {
        T sum = 0;
        for (unsigned int k = 0; k < channels; k++) {
            sum += input[i * channels + k] / channels;
        }
        output[i] = sum;
    }
    this->reader->advance(numSamples * channels);
    this->writer->advance(numSamples);
}

namespace Csdr {
    template class Downmix<short>;
    template class Downmix<float>;
}