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
        unsigned short c = useFec? fec(input[i]) : input[i];
        switch (c) {
            case 0:
                break;
            default:
                *(writer->getWritePointer()) = ascii(c);
                writer->advance(1);
                break;
        }
    }
    reader->advance(length);
}

signed char Ccir493Decoder::ascii(unsigned short code) {
    return code < sizeof(CCIR493_CODES)? CCIR493_CODES[code] : -1;
}

bool Ccir493Decoder::isValid(unsigned short code) {
    return (code < sizeof(CCIR493_CODES)) && (CCIR493_CODES[code] >= 0);
}

unsigned short Ccir493Decoder::fec(unsigned short code) {
    signed char c = ascii(code);
    switch (c) {
        case CCIR493_PHASE_DX:
            alpha = 0;
            break;
        case CCIR493_PHASE_RX0:
        case CCIR493_PHASE_RX0 + 1:
        case CCIR493_PHASE_RX0 + 2:
        case CCIR493_PHASE_RX0 + 3:
        case CCIR493_PHASE_RX0 + 4:
        case CCIR493_PHASE_RX0 + 5:
        case CCIR493_PHASE_RX0 + 6:
        case CCIR493_PHASE_RX0 + 7:
            alpha = 1;
            break;
    }

    if (alpha) {
        errors = c1==code? 0 : errors + 1;
        code = c1==code? code
             : errors>errorsAllowed? 0
             : isValid(code)? code
             : isValid(c1)? c1
             : isValid(c1|code)? (c1|code)
             : isValid(c1&code)? (c1&code)
             : 0;
    } else {
        c1 = c2;
        c2 = c3;
        c3 = code;
        code = 0;
    }

    alpha = !alpha;
    return code;
}
