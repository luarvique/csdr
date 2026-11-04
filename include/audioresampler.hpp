/*
Copyright (c) 2021 Jakob Ketterl <jakob.ketterl@gmx.de>

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

#include <samplerate.h>

namespace Csdr {

    class AudioResampler: public Module<float, float> {
        public:
            AudioResampler(unsigned int inputRate, unsigned int outputRate, unsigned int channels = 1);
            explicit AudioResampler(double rate, unsigned int channels = 1);
            ~AudioResampler() override;
            bool canProcess() override;
            void process() override;
        private:
            double rate;
            unsigned int channels;
            SRC_STATE* srcState;
    };

}