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

    const unsigned char CCIR476_LTR_SHIFT = 90;
    const unsigned char CCIR476_FIG_SHIFT = 54;
    const unsigned char CCIR476_SIA = 15;
    const unsigned char CCIR476_SIB = 51;
    const unsigned char CCIR476_RPT = 102;
    const unsigned char CCIR476_BLK = 106;
    const unsigned char CCIR476_BEL_FIG = 23;
    const unsigned char CCIR476_ENQ_FIG = 83;

    const unsigned char CCIR476_LETTERS[] = {
            '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  '>',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  'J',
            '\0', '\0', '\0',  'F', '\0',  'C',  'K', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  'W',
            '\0', '\0', '\0',  'Y', '\0',  'P',  'Q', '\0',
            '\0', '\0', '\0', '\0', '\0',  'G', '\0', '\0',
            '\0',  'M',  'X', '\0',  'V', '\0', '\0', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  'A',
            '\0', '\0', '\0',  'S', '\0',  'I',  'U', '\0',
            '\0', '\0', '\0',  'D', '\0',  'R',  'E', '\0',
            '\0',  'N', '\0', '\0',  ' ', '\0', '\0', '\0',
            '\0', '\0', '\0',  'Z', '\0',  'L',  '<', '\0',
            '\0',  'H', '\0', '\0', '\n', '\0', '\0', '\0',
            '\0',  'O',  'B', '\0',  'T', '\0', '\0', '\0',
            '\r', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
    };

    const unsigned char CCIR476_FIGURES[] = {
            '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  '>',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\a',
            '\0', '\0', '\0',  '!', '\0',  ':',  '(', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  '2',
            '\0', '\0', '\0',  '6', '\0',  '0',  '1', '\0',
            '\0', '\0', '\0', '\0', '\0',  '&', '\0', '\0',
            '\0',  '.',  '/', '\0',  '=', '\0', '\0', '\0',
            '\0', '\0', '\0', '\0', '\0', '\0', '\0',  '-',
            '\0', '\0', '\0', '\'', '\0',  '8',  '7', '\0',
            '\0', '\0', '\0', '\0', '\0',  '4',  '3', '\0',
            '\0',  ',', '\0', '\0',  ' ', '\0', '\0', '\0',
            '\0', '\0', '\0',  '+', '\0',  ')',  '<', '\0',
            '\0',  '#', '\0', '\0', '\n', '\0', '\0', '\0',
            '\0',  '9',  '?', '\0',  '5', '\0', '\0', '\0',
            '\r', '\0', '\0', '\0', '\0', '\0', '\0', '\0'
    };

    class Ccir476Decoder: public Module<unsigned char, unsigned char> {
        public:
            Ccir476Decoder(bool fec = true, bool strict = true)
            : useFec(fec), strict(strict) {}

            bool canProcess() override;
            void process() override;

        private:
            unsigned char c1 = '\0', c2 = '\0', c3 = '\0';
            int alpha = 0;
            int mode = 0;
            bool useFec;
            bool strict;

            unsigned char fec(unsigned char code);
            unsigned char ascii(unsigned char code);
            bool isValid(unsigned char code);
    };
}
