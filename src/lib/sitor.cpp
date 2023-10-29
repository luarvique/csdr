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

    switch (phase) {
        case 0:
            // If phasing header of RPT->SIA found
            if (output == 0b00011111100110)
            {
              flip = 0;
              phase++;

              *out++ = '>'|0x80;
              writer->advance(1);
            }
            break;
        case 1:
            // SIA of the phasing header
            phase++;
            *out++ = '<'|0x80;
            writer->advance(1);
            break;
        case 2:
            // Determine if we are using inverted CCIR476 characters
            flip = marks==3;
            phase++;
            *out++ = (flip? '^':'v')|0x80;
            writer->advance(1);
            break;
        case 3:
            // Invert CCIR476 character if enabled
            if (flip) { output = ~output; marks = 7 - marks; }
            break;
    }

    if (phase || (marks == 4)) {
        // Reset back to phasing if there are too many errors
        errors = marks==4? 0 : errors + 1;
        if (errors > 16) phase = 0;
        // Output received character
        *out++ = output & 0x7F;
        reader->advance(7);
        writer->advance(1);
    } else {
        // Keep searching for the correct phase
        reader->advance(1);
    }
}

bool SitorDecoder::toBit(float sample) {
    return (sample > 0) != invert;
}
