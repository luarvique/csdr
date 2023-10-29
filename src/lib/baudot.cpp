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

#include "baudot.hpp"

using namespace Csdr;

bool BaudotDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0;
}

void BaudotDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char* input = reader->getReadPointer();
    size_t length = reader->available();
    for (size_t i = 0; i < length; i++) {
        unsigned char c = input[i];
        switch (c) {
            case BAUDOT_FIG_SHIFT:
                mode = 1;
                break;
            case BAUDOT_LTR_SHIFT:
                mode = 0;
                break;
            default:
                // Debug mode: plain ASCII characters mapped to 128..255 range
                c = c>127? c-128 : c>31? '\0' : mode? BAUDOT_FIGURES[c] : BAUDOT_LETTERS[c];
                * (writer->getWritePointer()) = c? c : '_';
                writer->advance(1);
                break;
        }
    }
    reader->advance(length);
}
