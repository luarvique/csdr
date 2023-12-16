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
    unsigned int marks = 0;
    int i;

    // Get ten input bits
    for (i = 0; i < 10; i++) {
        unsigned int bit = toBit(data[i]);
        output |= (bit << i);
        if (i<7) marks += bit;
    }

    // Last three bits indicate the number of zeros in the first seven
    if (!isValid(output)) {
        // Skip a bit
        reader->advance(1);
    } else {
        // Pass received character through FEC
        output = fec(output);
        // Output received character
        if (isValid(output)) {
if(output) {
char s[256];
sprintf(s, " %d", toCode(output));
if(writer->writeable() >= strlen(s)) {
    memcpy(writer->getWritePointer(), s, strlen(s));
    writer->advance(strlen(s));
}
}


//            *writer->getWritePointer() = toCode(output);
//            writer->advance(1);
        }
        // Skip 10 bits
        reader->advance(10);
    }
}

bool Ccir493Decoder::toBit(float sample) {
    return (sample > 0) != invert;
}

bool Ccir493Decoder::isValid(unsigned short code) {
    return (code < 0x400) && ((code & 7) == CCIR493_BITCOUNT[code >> 3]);
}

char Ccir493Decoder::toCode(unsigned short code) {
    return code >> 3;
}

unsigned short Ccir493Decoder::fec(unsigned short code) {
    switch (code) {
        case CCIR493_PHASE_DX:
            // This symbol is always received in DX phase
            rxPhase = false;
            break;
        case CCIR493_PHASE_RX0:
        case CCIR493_PHASE_RX1:
        case CCIR493_PHASE_RX2:
        case CCIR493_PHASE_RX3:
        case CCIR493_PHASE_RX4:
        case CCIR493_PHASE_RX5:
        case CCIR493_PHASE_RX6:
        case CCIR493_PHASE_RX7:
            // The DX phase had to have CCIR493_PHASE_DX
            code = c1==CCIR493_PHASE_DX? code : 0xFFFF;
            // DX phase next
            rxPhase = false;
            return code;
    }

    if (rxPhase) {
        code = c1==code? code
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
