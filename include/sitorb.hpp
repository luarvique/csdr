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

    class SitorBDecoder: public Module<float, unsigned char> {
        public:
            explicit SitorBDecoder(unsigned int jitter = 1, unsigned int errorsAllowed = 16, bool invert = false)
            : jitter(jitter<=6? jitter : 6), errorsAllowed(errorsAllowed), invert(invert) {}

            bool canProcess() override;
            void process() override;

        private:
            unsigned int jitter;
            unsigned int errorsAllowed;
            bool invert;

            unsigned char c1 = '\0', c2 = '\0', c3 = '\0';
            unsigned int errors = 0;
            bool rxPhase = false;

            bool toBit(float sample);
            unsigned char fec(unsigned char code);
            bool isValid(unsigned char code);
    };

}
