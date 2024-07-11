/*
Copyright (c) 2024 Jakob Ketterl <jakob.ketterl@gmx.de>

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

#include "throttle.hpp"

#include <iostream>
#include <cstring>

using namespace Csdr;

template <typename T>
Throttle<T>::Throttle(size_t rate, size_t chunkSize):
    Module<T, T>(),
    rate(rate),
    chunkSize(chunkSize),
    nominalDuration(chunkSize * 1E6 / rate)
{}

template <typename T>
Throttle<T>::~Throttle<T>() {
    if (worker != nullptr) {
        std::thread* old = worker;
        worker = nullptr;
        run = false;
        old->join();
        delete(old);
    }
}

template <typename T>
void Throttle<T>::setReader(Reader<T> *reader) {
    Module<T, T>::setReader(reader);
    std::lock_guard<std::mutex> lock(this->processMutex);
    if (worker == nullptr && this->reader != nullptr && this->writer != nullptr) {
        worker = new std::thread([this] { loop(); });
    }
}

template <typename T>
void Throttle<T>::setWriter(Writer<T>* writer) {
    Module<T, T>::setWriter(writer);
    std::lock_guard<std::mutex> lock(this->processMutex);
    if (worker == nullptr && this->reader != nullptr && this->writer != nullptr) {
        worker = new std::thread([this] { loop(); });
    }
}

template <typename T>
void Throttle<T>::loop() {
    while (run) {
        {
            std::lock_guard<std::mutex> lock(this->processMutex);
            if (this->reader->available() < chunkSize) {
//                std::cerr << "Csdr::Throttle buffer under-run" << std::endl;
            } else if (this->writer->writeable() < chunkSize) {
//                std::cerr << "Csdr::Throttle buffer over-run" << std::endl;
            } else {
                std::memcpy(this->writer->getWritePointer(), this->reader->getReadPointer(), sizeof(T) * chunkSize);
                this->reader->advance(chunkSize);
                this->writer->advance(chunkSize);
            }
        }


        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        if (nextScheduledExecution != std::chrono::system_clock::time_point()) {
            nextScheduledExecution += nominalDuration;
        } else {
            nextScheduledExecution = now + nominalDuration;
        }

        std::chrono::duration<float, std::micro> toSleep = nextScheduledExecution - now;

        if (toSleep > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(toSleep);
        }
    }
}

namespace Csdr {
    template class Throttle<short>;
    template class Throttle<float>;
}
