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

#include "dsc.hpp"
#include <string.h>
#include <stdio.h>

using namespace Csdr;

bool DscDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (reader->available() > 2) && (writer->writeable() > 8);
}

void DscDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    int todo, done;

    // @@@ TODO: REMOVE THIS!!!
    unsigned char c = *reader->getReadPointer();
    sprintf((char *)writer->getWritePointer(), " %d", c);
    writer->advance(strlen((const char *)writer->getWritePointer()));
    reader->advance(1);
    return;


    // Try obtaining complete DSC message from the input
    do {
      todo = reader->available();
      done = todo>0? parseMessage(reader->getReadPointer(), todo) : 0;
      done = done>0? done : todo>32? 1 : 0;

      // Advance input
      if (done>0) reader->advance(done);
    } while(done);
}

bool DscDecoder::isValid(unsigned char code) {
    return code < 128;
}

int DscDecoder::parseMessage(const unsigned char *in, int size) {
    char dst[16]  = ""; // Destination ID or location
    char src[16]  = ""; // Source ID or location
    char id[16]   = ""; // Distress ID
    char loc[16]  = ""; // Distress location
    char time[16] = ""; // Time
    char freq[16] = ""; // Frequency or duration
    char num[16]  = ""; // Number
    char next[16] = ""; // Subsequent communication type
    char cmd1[16] = ""; // Telecommand #1
    char cmd2[16] = ""; // Telecommand #2

    const char *category = 0;
    const char *distress = 0;

    // Index within message
    int i = 2;
    int j;

    // Must have repeated format specifier
    if (i>size) return 0;
    if (in[0]!=in[1]) return 1;

    // Depending on message format...
    switch (in[0]) {

        case DSC_FMT_DISTRESS:
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(in[i++]) : 0;
            if (!distress) return 0;
            // Parse distress location
            j = parseLocation(loc, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse time
            j = parseTime(time, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse subsequent comms
            if((i>=size) || !parseNext(next, in[i++])) return 0;
            break;

        case DSC_FMT_ALLSHIPS:
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return 0;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse telecommand
            if((i>=size) || !parseCommand(cmd1, in[i++])) return 0;
            // Parse distress address
            j = parseAddress(id, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(in[i++]) : 0;
            if (!distress) return 0;
            // Parse distress location
            j = parseLocation(loc, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse time
            j = parseTime(time, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse subsequent comms
            if((i>=size) || !parseNext(next, in[i++])) return 0;
            break;

        case DSC_FMT_AREACALL:
        case DSC_FMT_GROUPCALL:
        case DSC_FMT_SELCALL:
            if (in[0] == DSC_FMT_AREACALL) {
                // Parse destination area
                j = parseArea(dst, in + i, size - i);
            } else {
                // Parse destination address
                j = parseAddress(dst, in + i, size - i);
            }
            if (!j) return 0; else i += j;
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return 0;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse telecommand
            if((i>=size) || !parseCommand(cmd1, in[i++])) return 0;
            // Parse distress address
            j = parseAddress(id, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(in[i++]) : 0;
            if (!distress) return 0;
            // Parse distress location
            j = parseLocation(loc, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse time
            j = parseTime(time, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse subsequent comms
            if((i>=size) || !parseNext(next, in[i++])) return 0;
            break;

        case DSC_FMT_AUTOCALL:
            // Parse destination address
            j = parseAddress(dst, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return 0;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse telecommands
            if((i>=size) || !parseCommand(cmd1, in[i++])) return 0;
            if((i>=size) || !parseCommand(cmd2, in[i++])) return 0;
            // Parse frequency or duration
            j = parseFrequency(freq, in + i, size - i);
            if (!j) return 0; else i += j;
            // Parse number
            j = parseNumber(num, in + i, size - i);
            if (!j) return 0; else i += j;
            break;

        default:
            // May not be a message, keep scanning
            return 2;
    }

    // Write out accumulated data
    startJson(in[0]);
    if(*src)  outputJson("src", src);
    if(*dst)  outputJson("dst", dst);
    if(*id)   outputJson("id", loc);
    if(*loc)  outputJson("loc", loc);
    if(*time) outputJson("time", time);
    if(*freq) outputJson("freq", freq);
    if(*num)  outputJson("num", num);
    if(category) outputJson("category", category);
    if(distress) outputJson("distress", distress);
    if(*next)    outputJson("next", next);
    if(*cmd1)    outputJson("cmd1", cmd1);
    if(*cmd2)    outputJson("cmd2", cmd2);
    endJson();

    // Done
    return i;
}

void DscDecoder::startJson(unsigned char type) {
    char buf[256];
    sprintf(buf, "{ \"type\": \"%s\"", parseType(type));
    printString(buf);
}

void DscDecoder::outputJson(const char *name, const char *value) {
    char buf[256];
    sprintf(buf, ", \"%s\": \"%s\"", name, value);
    printString(buf);
}

void DscDecoder::endJson() {
    printString(" }");
}

int DscDecoder::parseAddress(char *out, const unsigned char *in, int size) {
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

int DscDecoder::parseLocation(char *out, const unsigned char *in, int size) {
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

int DscDecoder::parseArea(char *out, const unsigned char *in, int size) {
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

int DscDecoder::parseTime(char *out, const unsigned char *in, int size) {
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

const char *DscDecoder::parseType(unsigned char code) {
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

const char *DscDecoder::parseCategory(unsigned char code) {
    switch (code) {
        case DSC_CAT_ROUTINE:  return "routine";
        case DSC_CAT_SAFETY:   return "safety";
        case DSC_CAT_URGENCY:  return "urgency";
        case DSC_CAT_DISTRESS: return "distress";
    }

    return 0;
}

const char *DscDecoder::parseDistress(unsigned char code) {
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

const char *DscDecoder::parseCommand(char *out, unsigned char code) {
    sprintf(out, "CMD-%d", code);
    return out;
}

const char *DscDecoder::parseNext(char *out, unsigned char code) {
    sprintf(out, "NEXT-%d", code);
    return out;
}

int DscDecoder::parseFrequency(char *out, const unsigned char *in, int size) {
    // @@@ TODO!!!
    out[0] = '\0';
    return 0;
}

int DscDecoder::parseNumber(char *out, const unsigned char *in, int size) {
    // @@@ TODO!!!
    out[0] = '\0';
    return 0;
}

void DscDecoder::printString(const char *buf) {
    unsigned int l = strlen(buf);

    // If there is enough output buffer available...
    if(writer->writeable()>=l) {
        // Write data then advance pointer
        memcpy(writer->getWritePointer(), buf, l);
        writer->advance(l);
    }
}
