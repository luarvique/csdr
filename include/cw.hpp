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
            CwDecoder(unsigned int sampleRate=12000, unsigned int targetFreq=800, unsigned int buckets=64);
            bool canProcess() override;
            void process() override;

        private:
            unsigned int sampleRate; // Input sampling rate
            unsigned int targetFreq; // CW carrier offset
            unsigned int buckets;    // Number of FFT buckets
            unsigned int NBTime;     // Noise blanker time (ms)

            double MagLimit;
            double MagLimitL;
            double MagTotal;
            unsigned int RealState0;
            unsigned int FiltState0;

            double Coeff;

            unsigned int Code;
            unsigned int Stop;
            unsigned int WPM;
            unsigned long LastStartT;
            unsigned long StartTimeH;
            unsigned long DurationH;
            unsigned long LastDurationH;
            unsigned long AvgTimeH;
            unsigned long StartTimeL;
            unsigned long DurationL;
            unsigned long AvgTimeL;

            // Time counting
            unsigned long curTime;     // Current time in seconds
            unsigned int  curSamples;  // Sample count since last second mark

            // Code to character conversion
            static const char cwTable[];

            // Get current time in milliseconds
            unsigned long msecs()
            { return(1000*curTime + 1000*curSamples/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Convert CW code to a character
            char cw2char(unsigned int data)
            { return(data<256? cwTable[data] : '#'); }
    };

}
