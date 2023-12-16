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

#include "sitorb.hpp"
#include "ccir476.hpp"

using namespace Csdr;

bool SitorBDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 7 + jitter;
}

void SitorBDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float *data = reader->getReadPointer();
    unsigned int output = 0;
    unsigned int marks = 0;
    int i;

    // Get seven input bits AND some jitter bits
    for (i = 0; i < 7 + jitter; i++) {
        unsigned int bit = toBit(data[i]);
        output |= (bit << i);
        if (i<7) marks += bit;
    }

    // Try aligning input stream to get a valid CCIR476 character
    for (i = 0; (marks != 4) && (i < jitter); i++) {
        marks += ((output>>(i+7)) & 1) - ((output>>i) & 1);
    }

    // If a valid character found, use it, aligning input stream
    if((i > 0) && (marks == 4)) {
        reader->advance(i);
        output >>= i;
    // If no valid character and too many errors, keep looking
    } else if (errors>errorsAllowed) {
        reader->advance(jitter + 1);
        return;
    }

    // Process and output received character
    output = fec(output & 0x7F);
    if (output) {
        *(writer->getWritePointer()) = output;
        writer->advance(1);
    }

    // Advance input
    reader->advance(7);
}

bool SitorBDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}

bool SitorBDecoder::isValid(unsigned char code) {
    return (code < 128) && CCIR476_VALID[code];
}

unsigned char SitorBDecoder::fec(unsigned char code) {
    switch (code) {
        case CCIR476_SIA:
            // This symbol is always received in DX phase
            rxPhase = false;
            errors  = 0;
            break;
        case CCIR476_RPT:
            // The DX phase had to have CCIR476_SIA
            code = c1==CCIR476_SIA? c1 : '\0';
            // DX phase next
            rxPhase = false;
            errors  = code? 0 : errors + 1;
            return code;
    }

    if (rxPhase) {
        errors = c1==code? 0 : errors + 1;
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

    rxPhase = !rxPhase;
    return code;
}
