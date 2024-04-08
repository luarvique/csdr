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
    return reader->available() >= receiving? 7 : 11;
}

void NavtexDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char *in  = reader->getReadPointer();
    size_t length = reader->available();
    size_t i;

    // Look for a message start
    for (i = 0 ; !receiving && (i + 11 <= length) ; i++) {
        unsigned char *p = in + i;
        receiving =
            (p[0]=='Z') && (p[1]=='C') && (p[2]=='Z') && (p[3]=='C') &&
            (p[4]==' ') && (p[9]=='\r') && (p[10]=='\n');
        // Reset character counter when starting a message
        if (receiving) received = 0;
    }

    // If still not receiving...
    if (!receiving) {
        // Skip non-message characters and drop out
        if (length > 10) reader->advance(length - 10);
        return;
    }

    //
    // ... RECEIVING NAVTEX MESSAGE ...
    //

    // Move to the first message character, if any
    if (i > 1) {
        i -= 1;
        reader->advance(i);
        length -= i;
        in += i;
    }

    // Look for a message end
    for (i = 0 ; receiving && (i + 7 <= length) ; i++) {
        unsigned char *p = in + i;
        receiving = !(
            (p[0]=='N') && (p[1]=='N') && (p[2]=='N') && (p[3]=='N') &&
            (p[4]=='\r') && (p[5]=='\n') && (p[6]=='\n')
        );
    }

    // Determine number of characters to copy
    i = !receiving? i + 6 : length > 6? length - 6 : 0;

    // Copy received message content to the output
    memcpy(writer->getWritePointer(), in, i);
    writer->advance(i);
    received += i;

    // Limit the number of characters per message
    if (received >= NAVTEXT_MAX_CHARS) receiving = false;
}
