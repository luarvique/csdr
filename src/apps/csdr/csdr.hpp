/*
Copyright (c) 2021-2023 Jakob Ketterl <jakob.ketterl@gmx.de>

This file is part of csdr.

csdr is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

csdr is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with csdr.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "module.hpp"

namespace Csdr {

    class Cli {
        public:
            int main(int argc, char** argv);
        private:
            template <typename T, typename U>
            void runModule(Module<T, U>* module);
    };

}