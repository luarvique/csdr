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

#include "dsc.hpp"
#include "ccir493.hpp"

using namespace Csdr;

bool DscDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 10;
}

void DscDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float *data = reader->getReadPointer();
    unsigned int output = 0;
    unsigned int marks = 0;
    int i;

    // Get ten input bits
    for (i = 0; i < 10; i++) {
        unsigned int bit = toBit(data[i]);
        output |= (bit << i);
        if (i<7) marks += bit;
    }

    // Last three bits indicate the number of zeros in the first seven
    if ((output & 7) != (7 - marks)) {
        // Skip a bit
        reader->advance(1);
    } else {
        // Output received character
        *writer->getWritePointer() = output >> 3;
        reader->advance(10);
        writer->advance(1);
    }
}

bool DscDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
