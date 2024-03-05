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

#include "dsc.hpp"
#include <string.h>
#include <stdio.h>
#include <time.h>

using namespace Csdr;

bool DscDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (reader->available() >= 32) && (writer->writeable() >= 256);
}

void DscDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    int todo, done;

    // Try obtaining complete DSC message from the input
    do {
      unsigned char *in  = reader->getReadPointer();
      unsigned char *out = writer->getWritePointer();

      todo = reader->available();
      done = todo>0? parseMessage(in, todo) : 0;
      done = done>0? done : todo>32? 1 : 0;

      // If failed to decode a message, output numeric data for debugging
      if ((done>=4) && (writer->getWritePointer()==out)) {
          startJson(DSC_FMT_ERROR);
          outputJson("data", in, todo>32? 32 : todo, done);
          outputJson("timestamp", time(NULL));
          endJson();
      }

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
    char msgt[16] = ""; // Time
    char rxfq[16] = ""; // RX frequency or duration
    char txfq[16] = ""; // TX frequency or duration
    char num[16]  = ""; // Number
    int format    = -1; // Message format
    int next      = -1; // Subsequent communication type
    int cmd1      = -1; // Telecommand #1
    int cmd2      = -1; // Telecommand #2
    int ecc       = -1; // Checksum

    const char *category = 0; // Message category
    const char *distress = 0; // Distress reason
    const char *eos      = 0; // End-of-sequence code

    int i, j, k, start;

    // Collect enough input first
    if (size < 32) return 0;

    // Check for sequence of phasing characters
    for (i=0, j=DSC_PHASE_RX7+1 ; (i<size-2) && (in[i]<j) && (in[i]>=DSC_PHASE_RX0) ; i++) {
        j = in[i];
    }

    // Must have at least two phasing characters
    if (i < 2) return i;

    // Must have repeated format specifier
    if (in[i] != in[i+1]) return i + 1;

    // Get message format and advance pointer
    format = in[i];
    start  = i + 1;
    i += 2;

    // Get timestamp
    time_t ts = time(NULL);

    // Depending on message format...
    switch (format) {

        case DSC_FMT_DISTRESS:
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(in[i++]) : 0;
            if (!distress) return i;
            // Parse distress location
            j = parseLocation(loc, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse time
            j = parseTime(msgt, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse subsequent comms
            if((i>=size) || !parseNext(&next, in[i++])) return i;
            break;

        case DSC_FMT_ALLSHIPS:
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return i;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse telecommand
            if((i>=size) || !parseCommand(&cmd1, in[i++])) return i;
            // Parse distress address
            j = parseAddress(id, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse distress type
            distress = i<size? parseDistress(in[i++]) : 0;
            if (!distress) return i;
            // Parse distress location
            j = parseLocation(loc, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse time
            j = parseTime(msgt, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse subsequent comms
            if((i>=size) || !parseNext(&next, in[i++])) return i;
            break;

        case DSC_FMT_AREACALL:
        case DSC_FMT_GROUPCALL:
        case DSC_FMT_SELCALL:
            if (format == DSC_FMT_AREACALL) {
                // Parse destination area
                j = parseArea(dst, in + i, size - i);
            } else {
                // Parse destination address
                j = parseAddress(dst, in + i, size - i);
            }
            if (!j) return i; else i += j;
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return i;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse telecommands
            if((i>=size) || !parseCommand(&cmd1, in[i++])) return i;
            if((i>=size) || !parseCommand(&cmd2, in[i++])) return i;
            // Parse frequencies or position (@@@ TODO: UTC)
            if((i<size) && (in[i]==55)) {
                ++i;
                j = parseLocation(loc, in + i, size - i);
                if (!j) return i; else i += j;
            } else {
                j = parseFrequency(rxfq, in + i, size - i);
                if (!j) return i; else i += j;
                j = parseFrequency(txfq, in + i, size - i);
                if (!j) return i; else i += j;
            }
            break;

        case DSC_FMT_AUTOCALL:
            // Parse destination address
            j = parseAddress(dst, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse category
            category = i<size? parseCategory(in[i++]) : 0;
            if (!category) return i;
            // Parse source address
            j = parseAddress(src, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse telecommands
            if((i>=size) || !parseCommand(&cmd1, in[i++])) return i;
            if((i>=size) || !parseCommand(&cmd2, in[i++])) return i;
            // Parse frequencies or duration
            j = parseFrequency(rxfq, in + i, size - i);
            if (!j) return i; else i += j;
            j = parseFrequency(txfq, in + i, size - i);
            if (!j) return i; else i += j;
            // Parse phone number
            j = parsePhone(num, in + i, size - i);
            if (!j) return i; else i += j;
            break;

        default:
            // May not be a message, keep scanning
            return i;
    }

    // Verify end-of-sequence
    if ((i+3>size) || (in[i] != in[i+2])) return i;

    // Advance to the end of the message
    eos = parseEos(in[i]);
    ecc = in[i+1];

    // Must have valid EOS
    if (!eos) return i + 3;

    // Compute actual ECC
    for (k=0, j=start ; j<i+1 ; ++j) k ^= in[j];

    // Write out accumulated data
    startJson(format);
    if(*src)     outputJson("src", src);
    if(*dst)     outputJson("dst", dst);
    if(*id)      outputJson("id", loc);
    if(*loc)     outputJson("loc", loc);
    if(*msgt)    outputJson("time", msgt);
    if(*rxfq)    outputJson("rxfreq", rxfq);
    if(*txfq)    outputJson("txfreq", txfq);
    if(*num)     outputJson("num", num);
    if(category) outputJson("category", category);
    if(distress) outputJson("distress", distress);
    if(next>=0)  outputJson("next", next);
    if(cmd1>=0)  outputJson("cmd1", cmd1);
    if(cmd2>=0)  outputJson("cmd2", cmd2);
    if(eos)      outputJson("eos", eos);
    outputJson("ecc", ecc==k);
    outputJson("timestamp", ts);
    endJson();

    // Done
    return i + 3;
}

void DscDecoder::startJson(unsigned char type) {
    char buf[256];
    sprintf(buf, "{ \"format\": \"%s\"", parseType(type));
    printString(buf);
}

void DscDecoder::outputJson(const char *name, const unsigned char *value, unsigned int length, int mark) {
    char buf[256];
    sprintf(buf, ", \"%s\": \"", name);
    printString(buf);

    // Print a string of decimal values
    for(unsigned int j=0 ; j<length ; ++j) {
        sprintf(buf, "%s%d", j==mark? "|":j? " ":"", value[j]);
        printString(buf);
    }

    sprintf(buf, "\"");
    printString(buf);
}

void DscDecoder::outputJson(const char *name, const char *value) {
    char buf[256];
    sprintf(buf, ", \"%s\": \"%s\"", name, value);
    printString(buf);
}

void DscDecoder::outputJson(const char *name, int value) {
    char buf[64];
    sprintf(buf, ", \"%s\": %d", name, value);
    printString(buf);
}

void DscDecoder::outputJson(const char *name, const time_t &value) {
    char buf[64];
    sprintf(buf, ", \"%s\": %lld", name, (long long int)value);
    printString(buf);
}

void DscDecoder::outputJson(const char *name, bool value) {
    char buf[64];
    sprintf(buf, ", \"%s\": %s", name, value? "true":"false");
    printString(buf);
}

void DscDecoder::endJson() {
    printString(" }\n");
}

int DscDecoder::parseAddress(char *out, const unsigned char *in, int size) {
    int i, j;

    // Return empty string on errors
    out[0] = '\0';

    // Must have ten decimal digits
    if (size<5) return 0;

    // Output digits, drop out on errors
    for (i=j=0 ; j<5 ; ++j) {
        if (in[j]<100) {
            out[i++] = '0' + (in[j] / 10);
            out[i++] = '0' + (in[j] % 10);
        } else {
            out[i++] = out[i++] = '-';
        }
    }

    // Terminate string, skipping the last digit (MMSIs are 9 digits)
    out[i-1] = '\0';

    // Done
    return j==5? j : 0;
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
        case DSC_FMT_ERROR:     return "error";
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

const char *DscDecoder::parseEos(unsigned char code) {
    switch (code) {
        case DSC_ACK_RQ: return "arq";
        case DSC_ACK_BQ: return "abq";
        case DSC_EOS:    return "done";
    }

    return 0;
}

bool DscDecoder::parseCommand(int *out, unsigned char code) {
    *out = code;
    return out;
}

bool DscDecoder::parseNext(int *out, unsigned char code) {
    *out = code;
    return out;
}

int DscDecoder::parseFrequency(char *out, const unsigned char *in, int size) {
    int i, j;

    // Will need 3-4 characters
    if (size<4) return 0;

    // Check if frequency is blank
    for (i=0 ; (i<3) && (in[i]==DSC_EMPTY) ; ++i);
    if (i==3) return i;

    // Starting output from index 0
    i = 0;
    j = 0;

    switch (in[0] / 10) {
        case 0:
        case 1:
        case 2:
            // Frequency in 100Hz increments
            for (i=0 ; i<3 ; ++i) {
                if (in[i] > 99) {
                    out[j++] = out[j++] = '-';
                } else if ((in[i]>0) || (j>0)) {
                    out[j++] = '0' + in[i] / 10;
                    out[j++] = '0' + in[i] % 10;
                }
            }
            if (j>0) out[j++] = '0';
            out[j++] = '0';
            out[j++] = '\0';
            break;
        case 3:
        case 8:
        case 9:
            // Only 3x, 8x, 90 are valid cases
            if (in[0]>90) return 0;
            out[j++] = 'C';
            out[j++] = 'H';
            if(in[0] % 10) out[j++] = '0' + in[0] % 10;
            // Channel number
            for (i=1 ; i<3 ; ++i) {
                if (in[i] > 99) {
                    out[j++] = out[j++] = '-';
                } else if ((in[i]>0) || (j>2)) {
                    out[j++] = '0' + in[i] / 10;
                    out[j++] = '0' + in[i] % 10;
                }
            }
            if (j==2) out[j++] = '0';
            out[j++] = '\0';
            break;
        case 4:
            // Only 40, 41, 42 are valid cases
            if (in[0]>42) return 0;
            if(in[0] % 10) out[j++] = '0' + in[0] % 10;
            // Frequency in 10Hz increments
            for (i=1 ; i<4 ; ++i) {
                if (in[i] > 99) {
                    out[j++] = out[j++] = '-';
                } else if ((in[i]>0) || (j>0)) {
                    out[j++] = '0' + in[i] / 10;
                    out[j++] = '0' + in[i] % 10;
                }
            }
            out[j++] = '0';
            out[j++] = '\0';
            break;
        default:
            // Anything else is not a frequency!
            return 0;
    }

    // Done
    return i;
}

int DscDecoder::parseNumber(char *out, const unsigned char *in, int size) {
    int i, j;

    // Going to assume 5 characters
    if (size<5) return 0;

    for (i=0, j=0 ; i<5 ; ++i) {
        if (in[i] > 99) {
            out[j++] = out[j++] = '-';
        } else if ((in[i]>0) || (j>0)) {
            out[j++] = '0' + in[i] / 10;
            out[j++] = '0' + in[i] % 10;
        }
    }

    // Terminate number
    out[j] = '\0';

    // Parsed 5 characters
    return i;
}

int DscDecoder::parsePhone(char *out, const unsigned char *in, int size) {
    int i, j;

    // Must have odd/even code + 4 characters characters
    if ((size<2) || ((in[0]!=106) && (in[0]!=105))) return 0;

    // Use even number of digits for 106, odd for 105
    j = 0;
    if (in[0]==106) out[j++] = in[1]>99? '-' : '0' + in[1] / 10;
    out[j++] = in[1]>99? '-' : '0' + in[1] % 10;

    // Parse numeric characters
    for (i=2, j=0 ; (i<size) && (in[i]<100) ; ++i) {
        out[j++] = '0' + in[i] / 10;
        out[j++] = '0' + in[i] % 10;
    }

    // Terminate number
    out[j] = '\0';

    // Parsed until non-numeric character
    return i;
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
