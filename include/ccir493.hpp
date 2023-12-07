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

    class Ccir493Decoder: public Module<float, unsigned char> {
        public:
            explicit Ccir493Decoder(bool fec = true, bool invert = false)
            : useFec(fec), invert(invert) {}

            bool canProcess() override;
            void process() override;

        private:
            unsigned char c1 = 0, c2 = 0, c3 = 0;
            bool rxPhase = false;
            bool useFec;
            bool invert;

            bool toBit(float sample);
            bool isValid(unsigned char code);
            unsigned char fec(unsigned char code);
    };

}
