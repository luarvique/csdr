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
            CwDecoder(unsigned int sampleRate=12000, unsigned int targetFreq=800, unsigned int targetWidth=100);
            ~CwDecoder();

            bool canProcess() override;
            void process() override;

        private:
            // Configurable input parameters
            unsigned int sampleRate;   // Input sampling rate (Hz)
            unsigned int targetFreq;   // CW carrier offset (Hz)
            unsigned int nbTime;       // Noise blanker time (ms)
            unsigned int quTime;       // Quantization step (ms)
            unsigned int dbgTime;      // Debug printout time (ms)
            bool showCw;               // TRUE: show dits/dahs

            // Computed FFT parameters
            unsigned int buckets;      // Number of FFT buckets (samples)
            unsigned int step;         // Quantization step (samples)
            double coeff;              // Used by Goertzel algorithm

            // Input signal characteristics
            double magL;               // Minimal observed magnitude
            double magH;               // Maximal observed magnitude
            unsigned int realState0;   // Last unfiltered signal state (0/1)
            unsigned int filtState0;   // Last filtered signal state (0/1)

            // Current CW code
            unsigned int code;         // Currently accumulated CW code or 1
            unsigned int stop;         // 1 if there is a code pending
            unsigned int wpm;          // Current CW speed (in wpm)

            // Dit / Dah timing
            unsigned long lastStartT;  // Time of the last signal change (ms)
            unsigned long startTimeH;  // Time HIGH signal started (ms)
            unsigned long durationH;   // Duration of the HIGH signal (ms)
            unsigned long avgTimeH;    // Average HIGH signal duration (ms)
            unsigned long startTimeL;  // Time LOW signal started (ms)
            unsigned long durationL;   // Duration of the LOW signal (ms)
            unsigned long avgTimeL;    // Average LOW signal duration (ms)

            // Sample buffer
            float *buf;
            unsigned int bufPos;

            // Time counting
            unsigned long curSeconds;  // Current time in seconds
            unsigned int  curSamples;  // Sample count since last second mark

            // Code to character conversion table
            static const char cwTable[];

            // Debugging data
            unsigned int histH[25];    // HIGH level duration histogram
            unsigned int histL[25];    // LOW level duration histogram
            unsigned int histCntH;     // Number of values in histH[]
            unsigned int histCntL;     // Number of values in histL[]
            unsigned long lastDebugT;  // Time of the last debug printout (ms)

            // Get current time in milliseconds
            unsigned long msecs()
            { return(1000*curSeconds + 1000*curSamples/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Convert CW code to a character
            char cw2char(unsigned int data)
            { return(data<256? cwTable[data] : '#'); }

            // Process a quantum of input data
            void processInternal(float *data, unsigned int size);

            // Print debug information
            void printDebug();
    };
}
