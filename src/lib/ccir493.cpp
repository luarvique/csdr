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
#include <string.h>
#include <stdio.h>

using namespace Csdr;

bool Ccir493Decoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 10;
}

void Ccir493Decoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float *data = reader->getReadPointer();
    unsigned int output = 0;
    int i;

    // Get ten input bits
    for (i = 0; i < 10; i++) {
        output |= toBit(data[i])? (1 << i) : 0;
    }

    // Make sure top three bits are in the correct order
    // (they indicate the number of zeros in the bottom seven)
    output = (output & 0x17F) | ((output & 0x200) >> 2) | ((output & 0x080) << 2);

    // Resync after several repeated errors
    if (!isValid(output) && (errors > 2)) {
        // Skip a bit
        reader->advance(1);
    } else {
        // Count repeating errors
        if(isValid(output)) errors = 0; else errors++;
        // Pass received character through FEC
        output = fec(output);
        // Output received character
        if (isValid(output)) {
            *(writer->getWritePointer()) = toCode(output);
            writer->advance(1);
        }
        // Skip 10 bits
        reader->advance(10);
    }
}

bool Ccir493Decoder::toBit(float sample) {
    return (sample > 0) == invert;
}

bool Ccir493Decoder::isValid(unsigned short code) {
    return (code < 0x400) && ((code >> 7) == CCIR493_ZEROCOUNT[code & 0x07F]);
}

char Ccir493Decoder::toCode(unsigned short code) {
    return code & 0x07F;
}

unsigned short Ccir493Decoder::fec(unsigned short code) {
    // This symbol is always received in DX phase
    if (toCode(code)==CCIR493_PHASE_DX) rxPhase = false;

    if (rxPhase) {
        code = toCode(c1)==CCIR493_PHASE_DX? code
             : c1==code? code
             : isValid(code)? code
             : isValid(c1)? c1
             : 0xFFFF;
    } else {
        c1 = c2;
        c2 = c3;
        c3 = code;
        code = 0;
    }

    rxPhase = !rxPhase;
    return code;
}
