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

#pragma once

#include "module.hpp"

namespace Csdr {
    // 15 minutes * 100 baud / 10 bit / 2 characters
    const size_t NAVTEXT_MAX_CHARS = 15 * 60 * 100 / 10 / 2;

    class NavtexDecoder: public Module<unsigned char, unsigned char> {
        public:
            bool canProcess() override;
            void process() override;

        private:
            bool receiving = false;
            size_t received;
    };
}
