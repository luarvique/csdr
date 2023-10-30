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

#include "ccir476.hpp"

using namespace Csdr;

bool Ccir476Decoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0;
}

void Ccir476Decoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char* input = reader->getReadPointer();
    size_t length = reader->available();

    for (size_t i = 0; i < length; i++) {
        // Debug mode: plain ASCII characters mapped to 128..255 range
        unsigned char c = useFec && (input[i]<128)? fec(input[i]) : input[i];

        switch (c) {
            case '\0':
            case CCIR476_SIA:
            case CCIR476_SIB:
            case CCIR476_RPT:
            case CCIR476_BLK:
                break;
            case CCIR476_FIG_SHIFT:
                mode = 1;
                break;
            case CCIR476_LTR_SHIFT:
                mode = 0;
                break;
            default:
                c = ascii(c);
                * (writer->getWritePointer()) = c? c : '_';
                writer->advance(1);
                break;
        }
    }

    reader->advance(length);
}

unsigned char Ccir476Decoder::ascii(unsigned char code) {
    // Debug mode: plain ASCII characters mapped to 128..255 range
    return code>=128? code-128 : mode? CCIR476_FIGURES[code] : CCIR476_LETTERS[code];
}

bool Ccir476Decoder::isValid(unsigned char code) {
    // Debug mode: plain ASCII characters mapped to 128..255 range
    if (code>=128) return true;

    int j;
    for (j=0; code; code>>=1) j += (code&1);
    return j==4;
}

unsigned char Ccir476Decoder::fec(unsigned char code) {
    switch (code) {
        case CCIR476_SIA:
            alpha = 1;
            break;
        case CCIR476_RPT:
            alpha = 0;
            break;
    }

    if (alpha) {
        // Detect phasing
        if((code==CCIR476_SIA) && (c1==CCIR476_RPT)) {
            code = c1 = CCIR476_SIA;
            mode = 0;

            * (writer->getWritePointer()) = '!';
            writer->advance(1);
        }

        errors = c1==code? 0 : errors + 1;
        if (errors==errorsAllowed) {
            * (writer->getWritePointer()) = '?';
            writer->advance(1);
        }

        code = c1==code? code
             : errors>errorsAllowed? '\0'
             : isValid(code)? code
             : isValid(c1)? c1
             : isValid(c1|code)? (c1|code)
             : isValid(c1&code)? (c1&code)
             : 128;
    } else {
        c1 = c2;
        c2 = c3;
        c3 = code;
        code = '\0';
    }

    alpha = !alpha;
    return code;
}
