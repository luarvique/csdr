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

#pragma once

#include "module.hpp"

#include <thread>
#include <chrono>

namespace Csdr {
    template <typename T>
    class Throttle: public Module<T, T> {
        public:
            Throttle(size_t rate, size_t chunkSize = 8096);
            ~Throttle() override;
            // processing is asynchronous, so this will always be false
            bool canProcess() override { return false; };
            // this doesn't do anything either. the worker thread does the work.
            void process() override {};
            void setReader(Reader<T>* reader) override;
            void setWriter(Writer<T>* writer) override;
        private:
            size_t rate;
            size_t chunkSize;
            std::chrono::duration<double, std::micro> nominalDuration;
            std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double, std::micro>> nextScheduledExecution;

            bool run = true;
            std::thread* worker = nullptr;
            void loop();
    };
}