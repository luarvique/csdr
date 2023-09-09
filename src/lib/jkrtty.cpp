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

#include "jkrtty.hpp"

using namespace Csdr;

JKRttyDecoder::JKRttyDecoder(bool invert):
    Module<float, unsigned char>(),
    invert(invert)
{}

JKRttyDecoder::JKRttyDecoder():
    JKRttyDecoder(false)
{}

bool JKRttyDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 8;
}

void JKRttyDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float* data = reader->getReadPointer();
    if (!toBit(data[0]) && toBit(data[6])) {
        unsigned char output = 0;
        for (int i = 0; i < 5; i++) {
            bool bit = toBit(data[5 - i]);
            output = (output << 1) | bit;
        }
        reader->advance(7);
        *(writer->getWritePointer()) = output;
        writer->advance(1);
    } else {
        reader->advance(1);
    }
}

bool JKRttyDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
