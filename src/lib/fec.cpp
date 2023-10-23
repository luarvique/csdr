/*
Copyright (c) 2023 Marat Fayzullin <luarvique@gmail.com>
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

#include "fec.hpp"

using namespace Csdr;

bool FecDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (reader->available() > fecSize) && (writer->available() > 0);
}

void FecDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char* input = reader->getReadPointer();
    size_t length = reader->available();
    for (size_t i = 0; i < length - fecSize; i++) {
        unsigned char c1 = input[i];
        unsigned char c2 = input[i + fecSize];
        * (writer->getWritePointer()) = c1==c2? c1 : '#';
        writer->advance(1);
    }
    reader->advance(length - fecSize);
}
