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

#include "power.hpp"

#include <cstring>
#include <utility>

using namespace Csdr;

template <typename T>
Power<T>::Power(size_t length, unsigned int decimation, std::function<void(float)> callback):
    length(length),
    decimation(decimation),
    callback(std::move(callback))
{}

template <typename T>
bool Power<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    size_t length = this->getLength();
    return (this->reader->available() > length && this->writer->writeable() > length);
}

template <typename T>
void Power<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    T* input = this->reader->getReadPointer();
    size_t length = this->getLength();

    float power = 0.0f;
    for (size_t i = 0; i < length; i += decimation) {
        power += std::norm(input[i]);
    }

    // compute average power
    power /= ceilf((float) length / decimation);

    // report power
    callback(power);

    // pass data
    forwardData(input, power);

    // advance input
    this->reader->advance(length);
}

template <typename T>
size_t Power<T>::getLength() {
    return length;
}

template <typename T>
void Power<T>::forwardData(T* input, float power) {
    T* output = this->writer->getWritePointer();
    size_t length = this->getLength();
    std::memcpy(output, input, length * sizeof(T));
    this->writer->advance(length);
}

template <typename T>
void Squelch<T>::setSquelch(float squelchLevel) {
    this->squelchLevel = squelchLevel;
}

template <typename T>
void Squelch<T>::forwardData(T *input, float power) {
    if (squelchLevel == 0.0f || power >= squelchLevel) {
        Power<T>::forwardData(input, power);
        flushCounter = 0;
    } else if (flushCounter < 5) {
        // produce some 0s to flush any subsequent modules if they have any overhead (e.g. FIR filter delays)
        T* output = this->writer->getWritePointer();
        size_t length = this->getLength();
        std::memset(output, 0, sizeof(T) * length);
        this->writer->advance(length);
        // increment inside because an unsigned char would overflow soon...
        flushCounter++;
    }
}

namespace Csdr {
    template class Power<float>;
    template class Power<complex<float>>;
    template class Squelch<float>;
    template class Squelch<complex<float>>;
}
