/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2021 Jakob Ketterl <jakob.ketterl@gmx.de>
Copyright (c) 2022 Marat Fayzullin <luarvique@gmail.com>
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

    class CwDecoder: public Module<float, unsigned char> {
        public:
            CwDecoder(unsigned int sampleRate=12000, unsigned int dahTime=240, unsigned int ditTime=10);
            bool canProcess() override;
            void process() override;

        private:
            static const int MAX_HISTORY  = 128;
            static const int QUANTUM_MSEC = 10;

            double hisLevel[MAX_HISTORY];
            double avgLevel;
            unsigned int hisPtr;

            unsigned int sampleRate;
            unsigned int curSignal;
            unsigned int curTime;
            unsigned int iniTime;
            unsigned int dahTime;
            unsigned int ditTime;
            unsigned int data;

            char parseSignal(unsigned int signal, unsigned int msec);

            unsigned int ms2smp(unsigned int msec)
            {
                return(sampleRate * msec / 1000);
            }
    };

}
