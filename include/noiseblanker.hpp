/*
Copyright (c) 2022 Marat Fayzullin <luarvique@gmail.com>

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

#include "filter.hpp"
#include "complex.hpp"

namespace Csdr {

    template <typename T>
    class NoiseBlanker: public Filter<T> {
        public:
            NoiseBlanker(double thr1 = -1.0, double thr2 = -1.0);
            size_t apply(T* input, T* output, size_t size) override;

        private:
            double nb1_threshold;
            double nb2_threshold;
            double nb1_avgMag;
            double nb2_avgMag;
            int hangTime;
            int delIdx;
            int sigIdx;
            T delay[8];
            T avgSig;

            void apply_nb1(T* buf, size_t size);
            void apply_nb2(T* buf, size_t size);
    };

    class RFNoiseBlanker: public NoiseBlanker<complex<float>> {
        public:
            RFNoiseBlanker(double thr1 = -1.0, double thr2 = -1.0):
                NoiseBlanker<complex<float>>(thr1, thr2) {}
    };
}
