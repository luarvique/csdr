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

#include "sitor.hpp"
#include "ccir476.hpp"

using namespace Csdr;

bool SitorDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 7 + jitter;
}

void SitorDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float* data = reader->getReadPointer();
    unsigned int output = 0;

    for (int i = 0; i < 7 + jitter; i++) {
        output |= (toBit(data[i]) << i);
        if (i>=7) {
            unsigned char c = (output >> (i - 7));
            if ((c==CCIR476_SIA) /*|| (c==CCIR476_RPT)*/) {
                reader->advance(i - 7);
                output >>= (i - 7);
                break;
            }
        }
    }

    * writer->getWritePointer() = output & 0x7F;
    reader->advance(7);
    writer->advance(1);
}

bool SitorDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
