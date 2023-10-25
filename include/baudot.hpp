/*
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

    const unsigned char BAUDOT_LTR_SHIFT = 31;
    const unsigned char BAUDOT_FIG_SHIFT = 27;

    const unsigned char BAUDOT_LETTERS[] = {
            '\0', 'E', '\n',  'A', ' ', 'S', 'I', 'U',
            '\r', 'D',  'R',  'J', 'N', 'F', 'C', 'K',
             'T', 'Z',  'L',  'W', 'H', 'Y', 'P', 'Q',
             'O', 'B',  'G', '\0', 'M', 'X', 'V', '\0'
    };

    const unsigned char BAUDOT_FIGURES[] = {
            '\0',  '3', '\n',  '-', ' ', '\'', '8', '7',
            '\r',  '$',  '4', '\a', ',',  '!', ':', '(',
             '5',  '+',  ')',  '2', '#',  '6', '0', '1',
             '9',  '?',  '&', '\0', '.',  '/', '=', '\0'
    };

    class BaudotDecoder: public Module<unsigned char, unsigned char> {
        public:
            bool canProcess() override;
            void process() override;
        private:
            int mode = 0;
    };

}
