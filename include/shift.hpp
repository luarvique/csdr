/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2023 Jakob Ketterl <jakob.ketterl@gmx.de>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANDRAS RETZLER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "module.hpp"
#include "complex.hpp"

namespace Csdr {

    class Shift {
        public:
            explicit Shift(float rate);
            virtual void setRate(float rate);
        protected:
            float rate;
    };

    class ShiftAddfast: public Shift, public FixedLengthModule<complex<float>, complex<float>> {
        public:
            explicit ShiftAddfast(float rate);
            void setRate(float rate) override;
        protected:
            void process(complex<float>* input, complex<float>* output) override;
            void process_fmv(complex<float>* input, complex<float>* output, size_t size);
            size_t getLength() override { return 1024; }
        private:
            float starting_phase = 0.0;
            float dsin[4];
            float dcos[4];
            float phase_increment;
    };

    class ShiftMath: public Shift, public AnyLengthModule<complex<float>, complex<float>> {
        public:
            explicit ShiftMath(float rate);
            void setRate(float rate) override;
        protected:
            void process(complex<float>* input, complex<float>* output, size_t size) override;
            void process_fmv(complex<float>* input, complex<float>* output, size_t size);
        private:
            float phase = 0.0;
            float phase_increment;
    };

}