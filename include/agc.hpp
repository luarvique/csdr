/*
This file is part of libcsdr.

	Copyright (c) Andras Retzler, HA7ILM <randras@sdr.hu>
	Copyright (c) Warren Pratt, NR0V <warren@wpratt.com>
    Copyright (c) Jakob Ketterl, DD5JFK <jakob.ketterl@gmx.de>
	Copyright 2006,2010,2012 Free Software Foundation, Inc.

    libcsdr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libcsdr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libcsdr.  If not, see <http://www.gnu.org/licenses/>.

*/

#pragma once

#include "module.hpp"
#include "ringbuffer.hpp"
#include "writer.hpp"

namespace Csdr {

    class UntypedAgc {
        public:
            virtual ~UntypedAgc() = default;
            virtual void setReference(float reference) = 0;
            virtual void setAttack(float attack_rate) = 0;
            virtual void setDecay(float decay_rate) = 0;
            virtual void setMaxGain(float max_gain) = 0;
            virtual void setInitialGain(float initial_gain) = 0;
            virtual void setHangTime(unsigned long int hang_time) = 0;
    };

    template <typename T>
    class Agc: public UntypedAgc, public AnyLengthModule<T, T> {
        public:
            void process(T* input, T* output, size_t work_size) override;

            void setReference(float reference) override;
            void setAttack(float attack_rate) override;
            void setDecay(float decay_rate) override;
            void setMaxGain(float max_gain) override;
            void setInitialGain(float initial_gain) override;
            void setHangTime(unsigned long int hang_time) override;
        private:
            float abs(T in);
            bool isZero(T in);
            T scale(T in);

            // params
            // fast profile defaults
            float reference = 0.8;
            float attack_rate = 0.1;
            float decay_rate = 0.001;
            float max_gain = 65535.0;
            unsigned long int hang_time = 200;
            float gain_filter_alpha = 1.5;
            // state
            float gain = 1.0;
            unsigned long int hang_counter = 0;
    };

}
