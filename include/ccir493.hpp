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

#pragma once

#include "module.hpp"

namespace Csdr {

    const unsigned char CCIR493_PHASE_RX0 = 104;
    const unsigned char CCIR493_PHASE_DX  = 125;
    const unsigned char CCIR493_ACK_RQ    = 117;
    const unsigned char CCIR493_ACK_BQ    = 122;
    const unsigned char CCIR493_EMPTY     = 126;
    const unsigned char CCIR493_EOS       = 127;

    const unsigned char DSC_FMT_DISTRESS  = 112;
    const unsigned char DSC_FMT_ALLSHIPS  = 116;
    const unsigned char DSC_FMT_GROUPCALL = 114;
    const unsigned char DSC_FMT_SELCALL   = 120;
    const unsigned char DSC_FMT_AREACALL  = 102;
    const unsigned char DSC_FMT_AUTOCALL  = 123;

    const unsigned char DSC_CAT_ROUTINE   = 100;
    const unsigned char DSC_CAT_SAFETY    = 108;
    const unsigned char DSC_CAT_URGENCY   = 110;
    const unsigned char DSC_CAT_DISTRESS  = 112;

    const unsigned char DSC_DIS_FIRE      = 100;
    const unsigned char DSC_DIS_FLOODING  = 101;
    const unsigned char DSC_DIS_COLLISION = 102;
    const unsigned char DSC_DIS_GROUNDING = 103;
    const unsigned char DSC_DIS_LISTING   = 104;
    const unsigned char DSC_DIS_SINKING   = 105;
    const unsigned char DSC_DIS_DISABLED  = 106;
    const unsigned char DSC_DIS_UNDEFINED = 107;
    const unsigned char DSC_DIS_ABANDONING = 108;
    const unsigned char DSC_DIS_PIRACY    = 109;
    const unsigned char DSC_DIS_MANOVERB  = 110;
    const unsigned char DSC_DIS_EPIRB     = 112;

    class Ccir493Decoder: public Module<unsigned char, unsigned char> {
        public:
            Ccir493Decoder(bool fec = true, unsigned int errorsAllowed = 16)
            : useFec(fec), errorsAllowed(errorsAllowed) {}

            bool canProcess() override;
            void process() override;

        private:
            unsigned char c1 = 0, c2 = 0, c3 = 0;
            unsigned int errorsAllowed;
            unsigned int errors = 0;
            int alpha = 0;
            bool useFec;

            unsigned char fec(unsigned char code);
            bool isValid(unsigned char code);

            bool parseMessage(const unsigned char *data, int size);

            int parseAddress(char *out, const unsigned char *in, int size);
            int parseLocation(char *out, const unsigned char *in, int size);
            int parseArea(char *out, const unsigned char *in, int size);
            int parseTime(char *out, const unsigned char *in, int size);
            int parseFrequency(char *out, const unsigned char *in, int size);
            int parseNumber(char *out, const unsigned char *in, int size);

            const char *parseType(unsigned char code);
            const char *parseCategory(unsigned char code);
            const char *parseDistress(unsigned char code);
            const char *parseCommand(unsigned char code);
            const char *parseNext(unsigned char code);

            void startJson(unsigned char type);
            void outputJson(const char *name, const char *value);
            void endJson();

            void printString(const char *buf);
    };
}
