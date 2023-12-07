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
    if ((output & 7) != (7 - marks)) {
        // Skip a bit
        reader->advance(1);
    } else {
        // Pass received character through FEC
        output >>= 3;
        output = useFec? fec(output) : output;
        // Output received character
//        if (output != 255) {
        if (output < 128) {
            *writer->getWritePointer() = output;
            writer->advance(1);
        }
        // Skip 10 bits
        reader->advance(10);
    }
}

bool Ccir493Decoder::toBit(float sample) {
    return (sample > 0) != invert;
}

bool Ccir493Decoder::isValid(unsigned char code) {
    return code < 128;
}

unsigned char Ccir493Decoder::fec(unsigned char code) {
    switch (code) {
        case CCIR493_PHASE_DX:
            rxPhase = false;
            break;
        case CCIR493_PHASE_RX0:
        case CCIR493_PHASE_RX0 + 1:
        case CCIR493_PHASE_RX0 + 2:
        case CCIR493_PHASE_RX0 + 3:
        case CCIR493_PHASE_RX0 + 4:
        case CCIR493_PHASE_RX0 + 5:
        case CCIR493_PHASE_RX0 + 6:
        case CCIR493_PHASE_RX0 + 7:
            rxPhase = true;
            break;
    }

    if (rxPhase) {
        code = c1==code? code
/*             : isValid(code)? code
             : isValid(c1)? c1 */
             : 128;
    } else {
        c1 = c2;
        c2 = c3;
        c3 = code;
        code = 255;
    }

    rxPhase = !rxPhase;
    return code;
}
