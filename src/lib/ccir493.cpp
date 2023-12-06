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

#include "ccir493.hpp"

using namespace Csdr;

bool Ccir493Decoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0;
}

void Ccir493Decoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char *input = reader->getReadPointer();
    size_t length = reader->available();
    for (size_t i = 0; i < length; i++) {
        unsigned char c = useFec? fec(input[i]) : input[i];
        switch (c) {
            default:
                c = ascii(c);
                *(writer->getWritePointer()) = c? c : '_';
                writer->advance(1);
                break;
        }
    }
    reader->advance(length);
}

unsigned char Ccir493Decoder::ascii(unsigned short code) {
    // @@@ TODO!!!
    return code>1023? '\0' : CCIR493_CODES[code];
}

bool Ccir493Decoder::isValid(unsigned short code) {
    // @@@ TODO!!!
    return true;
}

unsigned char Ccir493Decoder::fec(unsigned short code) {
    // @@@ TODO!!!
    return code;
}
