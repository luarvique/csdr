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

#include "navtex.hpp"
#include <cstring>

using namespace Csdr;

bool NavtexDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 11;
}

void NavtexDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char *in  = reader->getReadPointer();
    size_t length = reader->available();
    size_t i;

    // If looking for a message...
    if (!receiving) {
        // Need at least 11 characters
        if (length < 11) return;

        // Look for a message start
        receiving =
            (in[0]=='Z') && (in[1]=='C') && (in[2]=='Z') && (in[3]=='C') &&
            (in[4]==' ') && (in[9]=='\r') && (in[10]=='\n');

        // Reset character counter when starting a message
        if (receiving) {
            received = 0;
        } else {
            reader->advance(1);
            return;
        }
    }

    //
    // ... RECEIVING NAVTEX MESSAGE ...
    //

    // Look for a message end
    for (i = 0 ; i + 7 <= length ; i++) {
        unsigned char *p = in + i;
        receiving = !(
            (p[0]=='N') && (p[1]=='N') && (p[2]=='N') && (p[3]=='N') &&
            (p[4]=='\r') && (p[5]=='\n') && (p[6]=='\n')
        );

        if (!receiving) {
            i += 7;
            break;
        }
    }

    // Copy received message content to the output
    memcpy(writer->getWritePointer(), in, i);
    writer->advance(i);
    reader->advance(i);
    received += i;

    // Limit the number of characters per message
    if (received >= NAVTEX_MAX_CHARS) receiving = false;
}
