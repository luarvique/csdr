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

    const unsigned char CCIR493_BITCOUNT[128] = {
        7, 6, 6, 5, 6, 5, 5, 4, 6, 5, 5, 4, 5, 4, 4, 3, // 0
        6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 16
        6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 32
        5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // 48
        6, 5, 5, 4, 5, 4, 4, 3, 5, 4, 4, 3, 4, 3, 3, 2, // 64
        5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // 80
        5, 4, 4, 3, 4, 3, 3, 2, 4, 3, 3, 2, 3, 2, 2, 1, // 96
        4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0  // 112
    };

    const unsigned short CCIR493_PHASE_RX0 = (104<<3) + 4;
    const unsigned short CCIR493_PHASE_RX1 = (105<<3) + 3;
    const unsigned short CCIR493_PHASE_RX2 = (106<<3) + 3;
    const unsigned short CCIR493_PHASE_RX3 = (107<<3) + 2;
    const unsigned short CCIR493_PHASE_RX4 = (108<<3) + 3;
    const unsigned short CCIR493_PHASE_RX5 = (109<<3) + 2;
    const unsigned short CCIR493_PHASE_RX6 = (110<<3) + 2;
    const unsigned short CCIR493_PHASE_RX7 = (111<<3) + 1;
    const unsigned short CCIR493_PHASE_DX  = (125<<3) + 1;
    const unsigned short CCIR493_ACK_RQ    = (117<<3) + 2;
    const unsigned short CCIR493_ACK_BQ    = (122<<3) + 2;
    const unsigned short CCIR493_EMPTY     = (126<<3) + 1;
    const unsigned short CCIR493_EOS       = (127<<3) + 0;

    class Ccir493Decoder: public Module<float, unsigned char> {
        public:
            explicit Ccir493Decoder(bool invert = false)
            : invert(invert) {}

            bool canProcess() override;
            void process() override;

        private:
            unsigned short c1 = 0, c2 = 0, c3 = 0;
            bool rxPhase = false;
            bool invert;

            bool toBit(float sample);
            bool isValid(unsigned short code);
            unsigned short fec(unsigned short code);
            char toCode(unsigned short code);
    };
}
