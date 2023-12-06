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
    return reader->available() > 0;
}

void Ccir493Decoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char *input = reader->getReadPointer();
    size_t length = reader->available();
    for (size_t i = 0; i < length; i++) {
        unsigned char c = useFec? fec(input[i]) : input[i];
        if (isValid(c)) {
            *(writer->getWritePointer()) = c;
            writer->advance(1);
        }
    }
    reader->advance(length);
}

bool Ccir493Decoder::isValid(unsigned char code) {
    return code < 128;
}

unsigned char Ccir493Decoder::fec(unsigned char code) {
    switch (code) {
        case CCIR493_PHASE_DX:
            alpha = 0;
            break;
        case CCIR493_PHASE_RX0:
        case CCIR493_PHASE_RX0 + 1:
        case CCIR493_PHASE_RX0 + 2:
        case CCIR493_PHASE_RX0 + 3:
        case CCIR493_PHASE_RX0 + 4:
        case CCIR493_PHASE_RX0 + 5:
        case CCIR493_PHASE_RX0 + 6:
        case CCIR493_PHASE_RX0 + 7:
            alpha = 1;
            break;
    }

    if (alpha) {
        errors = c1==code? 0 : errors + 1;
        code = c1==code? code
             : errors>errorsAllowed? 0xFF
             : isValid(code)? code
             : isValid(c1)? c1
             : 0xFF;
    } else {
        c1 = c2;
        c2 = c3;
        c3 = code;
        code = 0xFF;
    }

    alpha = !alpha;
    return code;
}

bool Ccir493Decoder::parseMessage(const unsigned char *data, int size) {
    char dst[16]  = ""; // Destination ID or location
    char src[16]  = ""; // Source ID or location
    char id[16]   = ""; // Distress ID
    char loc[16]  = ""; // Distress location
    char time[16] = ""; // Time
    char freq[16] = ""; // Frequency or duration
    char num[16]  = ""; // Number

    const char *category = 0;
    const char *distress = 0;
    const char *next = 0;
    const char *cmd1 = 0;
    const char *cmd2 = 0;

    // Index within message
    int i = 2;
    int j;

    // Must have repeated format specifier
    if ((i>size) || (data[0]!=data[1])) return false;

    // Depending on message format...
    switch (data[0]) {

        case DSC_FMT_DISTRESS:
            // Parse source address
            j = parseAddress(src, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(data[i++]) : 0;
            if (!distress) return false;
            // Parse distress location
            j = parseLocation(loc, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse time
            j = parseTime(time, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse subsequent comms
            next = i<size? parseNext(data[i++]) : 0;
            if (!next) return false;
            break;

        case DSC_FMT_ALLSHIPS:
            // Parse category
            category = i<size? parseCategory(data[i++]) : 0;
            if (!category) return false;
            // Parse source address
            j = parseAddress(src, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse telecommand
            cmd1 = i<size? parseCommand(data[i++]) : 0;
            if (!cmd1) return false;
            // Parse distress address
            j = parseAddress(id, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(data[i++]) : 0;
            if (!distress) return false;
            // Parse distress location
            j = parseLocation(loc, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse time
            j = parseTime(time, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse subsequent comms
            next = i<size? parseNext(data[i++]) : 0;
            if (!next) return false;
            break;

        case DSC_FMT_AREACALL:
        case DSC_FMT_GROUPCALL:
        case DSC_FMT_SELCALL:
            if (data[0] == DSC_FMT_AREACALL) {
                // Parse destination area
                j = parseArea(dst, data + i, size - i);
            } else {
                // Parse destination address
                j = parseAddress(dst, data + i, size - i);
            }
            if (!j) return false; else i += j;
            // Parse category
            category = i<size? parseCategory(data[i++]) : 0;
            if (!category) return false;
            // Parse source address
            j = parseAddress(src, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse telecommand
            cmd1 = i<size? parseCommand(data[i++]) : 0;
            if (!cmd1) return false;
            // Parse distress address
            j = parseAddress(id, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(data[i++]) : 0;
            if (!distress) return false;
            // Parse distress location
            j = parseLocation(loc, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse time
            j = parseTime(time, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse subsequent comms
            next = i<size? parseNext(data[i++]) : 0;
            if (!next) return false;
            break;

        case DSC_FMT_AUTOCALL:
            // Parse destination address
            j = parseAddress(dst, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse category
            category = i<size? parseCategory(data[i++]) : 0;
            if (!category) return false;
            // Parse source address
            j = parseAddress(src, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse telecommands
            cmd1 = i<size? parseCommand(data[i++]) : 0;
            if (!cmd1) return false;
            cmd2 = i<size? parseCommand(data[i++]) : 0;
            if (!cmd2) return false;
            // Parse frequency or duration
            j = parseFrequency(freq, data + i, size - i);
            if (!j) return false; else i += j;
            // Parse number
            j = parseNumber(num, data + i, size - i);
            if (!j) return false; else i += j;
            break;

        default:
            // Unknown message format
            return false;
    }

    // Write out accumulated data
    startJson(data[0]);
    if(*src)  outputJson("src", src);
    if(*dst)  outputJson("dst", dst);
    if(*id)   outputJson("id", loc);
    if(*loc)  outputJson("loc", loc);
    if(*time) outputJson("time", time);
    if(*freq) outputJson("freq", freq);
    if(*num)  outputJson("num", num);
    if(category) outputJson("category", category);
    if(distress) outputJson("distress", distress);
    if(next)     outputJson("next", next);
    if(cmd1)     outputJson("cmd1", cmd1);
    if(cmd2)     outputJson("cmd2", cmd2);
    endJson();

    // Done
    return true;
}

void Ccir493Decoder::startJson(unsigned char type) {
    char buf[256];
    sprintf(buf, "{ \"type\": \"%s\"", parseType(type));
    printString(buf);
}

void Ccir493Decoder::outputJson(const char *name, const char *value) {
    char buf[256];
    sprintf(buf, ", \"%s\": \"%s\"", name, value);
    printString(buf);
}

void Ccir493Decoder::endJson() {
    printString(" }");
}

int Ccir493Decoder::parseAddress(char *out, const unsigned char *in, int size) {
    int i, j;

    // Return empty string on errors
    out[0] = '\0';

    // Must have ten decimal digits
    if (size<5) return 0;

    // Output digits, drop out on errors
    for (i=j=0 ; (j<5) && (in[j]<100) ; ++j) {
        out[i++] = in[j] / 10;
        out[i++] = in[j] % 10;
    }

    // Return empty string on errors
    out[j==5? i:0] = '\0';

    // Done
    return j==5? 5 : 0;
}

int Ccir493Decoder::parseLocation(char *out, const unsigned char *in, int size) {
    // Return empty string on errors
    out[0] = '\0';

    // Must have ten decimal digits
    if (size<5) return 0;

    // Check for "unknown location" (5 x 99)
    if ((in[0]==99) && (in[1]==99) && (in[2]==99) && (in[3]==99) && (in[4]==99)) {
        strcpy(out, "???");
        return 5;
    }

    // Verify QY.Yy.yX.XX.xx format
    if ((in[0]>39) || (in[1]>99) || (in[2]>99) || (in[3]>99) || (in[4]>99)) return 0;

    // Get quadrant, latitude, longitude
    unsigned int quad = in[0] / 10;
    unsigned int latD = ((in[0] % 10) * 10) + (in[1] / 10);
    unsigned int latM = ((in[1] % 10) * 10) + (in[2] / 10);
    unsigned int lonD = ((in[2] % 10) * 100) + in[3];
    unsigned int lonM = in[4];

    // Verify latitude and longitude
    if ((latD>180) || (latM>59) || (lonD>90) || (lonM>59)) return 0;

    // Print location
    sprintf(out, "%.3f%c%.3f%c",
        latD + latM / 60.0, quad&2? 'S' : 'N',
        lonD + lonM / 60.0, quad&1? 'W' : 'E'
    );

    // Done
    return 5;
}

int Ccir493Decoder::parseArea(char *out, const unsigned char *in, int size) {
    // Return empty string on errors
    out[0] = '\0';

    // Must have ten decimal digits
    if (size<5) return 0;

    // Check for "unknown location" (5 x 99)
    if ((in[0]==99) && (in[1]==99) && (in[2]==99) && (in[3]==99) && (in[4]==99)) {
        strcpy(out, "???");
        return 5;
    }

    // Verify QY.YX.XX.HH.WW format
    if ((in[0]>39) || (in[1]>99) || (in[2]>99) || (in[3]>99) || (in[4]>99)) return 0;

    // Get quadrant, latitude, longitude
    unsigned int quad = in[0] / 10;
    unsigned int latD = ((in[0] % 10) * 10) + (in[1] / 10);
    unsigned int lonD = ((in[1] % 10) * 100) + in[2];
    unsigned int latH = in[4];
    unsigned int lonW = in[5];

    // Verify latitude and longitude
    if ((latD>180) || (lonD>90)) return 0;

    // Print location
    sprintf(out, "%d%c%d%c+%d+%d",
        latD, quad&2? 'S' : 'N',
        lonD, quad&1? 'W' : 'E',
        latH, lonW
    );

    // Done
    return 5;
}

int Ccir493Decoder::parseTime(char *out, const unsigned char *in, int size) {
    // Return empty string on errors
    out[0] = '\0';

    // Must have four decimal digits
    if (size<2) return 0;

    // Check for "unknown time" (4 x 88)
    if ((in[0]==88) && (in[1]==88)) {
        strcpy(out, "???");
        return 2;
    }

    // Validate hours and minutes
    if ((in[0]>23) || (in[1]>59)) return 0;

    // Output time
    out[0] = '0' + (in[0] / 10);
    out[1] = '0' + (in[0] % 10);
    out[2] = '0' + (in[1] / 10);
    out[3] = '0' + (in[1] % 10);
    out[4] = '\0';

    // Done
    return 2;
}

const char *Ccir493Decoder::parseType(unsigned char code) {
    switch (code) {
        case DSC_FMT_DISTRESS:  return "distress";
        case DSC_FMT_ALLSHIPS:  return "allships";
        case DSC_FMT_GROUPCALL: return "groupcall";
        case DSC_FMT_SELCALL:   return "selcall";
        case DSC_FMT_AREACALL:  return "areacall";
        case DSC_FMT_AUTOCALL:  return "autocall";
    }

    return 0;
}

const char *Ccir493Decoder::parseCategory(unsigned char code) {
    switch (code) {
        case DSC_CAT_ROUTINE:  return "routine";
        case DSC_CAT_SAFETY:   return "safety";
        case DSC_CAT_URGENCY:  return "urgency";
        case DSC_CAT_DISTRESS: return "distress";
    }

    return 0;
}

const char *Ccir493Decoder::parseDistress(unsigned char code) {
    switch (code) {
        case DSC_DIS_FIRE:       return "fire / explosion";
        case DSC_DIS_FLOODING:   return "flooding";
        case DSC_DIS_COLLISION:  return "collision";
        case DSC_DIS_GROUNDING:  return "grounding";
        case DSC_DIS_LISTING:    return "listing / may capsize";
        case DSC_DIS_SINKING:    return "sinking";
        case DSC_DIS_DISABLED:   return "disabled / adrift";
        case DSC_DIS_UNDEFINED:  return "undefined";
        case DSC_DIS_ABANDONING: return "abandoning ship";
        case DSC_DIS_PIRACY:     return "piracy / robbery";
        case DSC_DIS_MANOVERB:   return "man overboard";
        case DSC_DIS_EPIRB:      return "EPIRB emission";
    }

    return 0;
}

const char *Ccir493Decoder::parseCommand(unsigned char code) {
    // @@@ TODO!!!
    return "CMD";
}

const char *Ccir493Decoder::parseNext(unsigned char code) {
    // @@@ TODO!!!
    return "NXT";
}

int Ccir493Decoder::parseFrequency(char *out, const unsigned char *in, int size) {
    // @@@ TODO!!!
    out[0] = '\0';
    return 0;
}

int Ccir493Decoder::parseNumber(char *out, const unsigned char *in, int size) {
    // @@@ TODO!!!
    out[0] = '\0';
    return 0;
}

void Ccir493Decoder::printString(const char *buf) {
    unsigned int l = strlen(buf);

    // If there is enough output buffer available...
    if(this->writer->writeable()>=l) {
        // Write data then advance pointer
        memcpy(this->writer->getWritePointer(), buf, l);
        this->writer->advance(l);
    }
}
