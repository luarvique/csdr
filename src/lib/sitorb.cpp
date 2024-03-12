/*
Copyright (c) 2023-2024 Marat Fayzullin <luarvique@gmail.com>

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
    return reader->available() >= 7;
}

void SitorBDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float *data = reader->getReadPointer();
    unsigned int output = 0;
    int i;

    // Get seven input bits
    for (i = 0; i < 7; i++) {
        output |= toBit(data[i])? (1 << i) : 0;
    }

    // Resync after several repeated errors
    if (!isValid(output) && (errors > errorsAllowed)) {
        // Skip a bit
        reader->advance(1);
    } else {
        // Count repeating errors
        if(isValid(output)) errors = 0; else errors++;
        // Process and output received character
        output = fec(output);
        if (output) {
            *(writer->getWritePointer()) = output;
            writer->advance(1);
        }
        // Skip 7 bits
        reader->advance(7);
    }
}

bool SitorBDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}

bool SitorBDecoder::isValid(unsigned char code) {
    return (code < 128) && (CCIR476_ZEROCOUNT[code] == 3);
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
        code = c1==code? code
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
