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

using namespace Csdr;

SitorDecoder::SitorDecoder(bool invert):
    Module<float, unsigned char>(),
    invert(invert)
{}

SitorDecoder::SitorDecoder():
    SitorDecoder(false)
{}

bool SitorDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 8;
}

void SitorDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float* data = reader->getReadPointer();
    unsigned char output = 0;
    for (int i = 0; i < 7; i++) {
        bool bit = toBit(data[7 - i]);
        output = (output << 1) | bit;
    }
    reader->advance(7);
    *(writer->getWritePointer()) = output;
    writer->advance(1);
}

bool SitorDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
