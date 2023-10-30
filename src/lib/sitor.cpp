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

bool SitorDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 14;
}

static int phase = 0;
static int flip = 0;
static int errors = 0;

void SitorDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    float* data = reader->getReadPointer();
    unsigned char* out = writer->getWritePointer();
    unsigned int output = 0;
    unsigned int marks = 0;
    int i;

    // Get seven input bits AND some jitter bits
    for (i = 0; i < 14; i++) {
        unsigned int bit = toBit(data[i]);
        output |= (bit << i);
        if (i<7) marks += bit;
    }

    // If phasing header of RPT->SIA found...
    if (output == 0b00011111100110) {
        phase  = 1;
        errors = 0;
        flip   = 0;
    }

    switch (phase) {
        case 1:
            // RPT of the phasing header
            phase = 2;
            *out++ = '>'|0x80;
            writer->advance(1);
            break;
        case 2:
            // SIA of the phasing header
            phase = 3;
            *out++ = '*'|0x80;
            writer->advance(1);
            break;
        case 3:
            // Determine if we are using inverted CCIR476 characters
            phase = (marks==3) || (marks==4)? 4 : 0;
            flip  = marks==3? 1 : 0;
            *out++ = (marks + '0')|0x80;
            writer->advance(1);
            break;
    }

    // Invert CCIR476 character if enabled
    if (flip) { output = ~output; marks = 7 - marks; }

    if (!phase && (marks != 4)) {
        // Keep searching for the correct phase
        reader->advance(1);
    } else {
        // Try correcting phase by one step
        if (marks != 4) {
            marks += ((output>>7) & 1) - (output & 1);
            if (marks == 4) {
                reader->advance(1);
                output >>= 1;
            }
        }

        // Output received character
        *out++ = output & 0x7F;
        reader->advance(7);
        writer->advance(1);

        // Track errors balance
        errors = marks!=4? (errors + 1) : errors>0? (errors - 1) : 0;

        // Revert to phasing if there are too many errors
        if (errors > 16) {
            phase = 0;
            *out++ = '<'|0x80;
            writer->advance(1);
        }
    }
}

bool SitorDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
