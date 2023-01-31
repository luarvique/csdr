/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2023 Marat Fayzullin <luarvique@gmail.com>
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

namespace Csdr {

    class RttyDecoder: public Module<float, unsigned char> {
        public:
            RttyDecoder(unsigned int sampleRate=12000, unsigned int targetFreq=450, unsigned int targetWidth=170, double baudRate=45.45, bool reverse=false);
            ~RttyDecoder();

            bool canProcess() override;
            void process() override;

        private:
            // Configurable input parameters
            unsigned int sampleRate;   // Input sampling rate (Hz)
            unsigned int targetFreq;   // Frequency (Hz)
            unsigned int targetWidth;  // Frequency shift (Hz)
            double baudRate;           // Baud rate
            unsigned int quTime;       // Quantization step (ms)
            unsigned int dbgTime;      // Debug printout time (ms)
            bool reverse;              // TRUE: reverse SPACE and MARK
            bool showRaw;              // TRUE: show raw undecoded data

            // Time counting
            unsigned long curSeconds = 0; // Current time in seconds
            unsigned int  curSamples = 0; // Sample count since last second mark

            // Sample buffer
            float *buf = 0;
            unsigned int bufPos = 0;

            // Computed FFT parameters
            unsigned int buckets;        // Number of FFT buckets (samples)
            unsigned int step;           // Quantization step (samples)
            double coeff1;               // Used by Goertzel algorithm
            double coeff2;               // Used by Goertzel algorithm

            // Input signal characteristics
            double magL = 1000.0;         // Minimal observed magnitude
            double magH = 0.0;            // Maximal observed magnitude
            unsigned long lastStartT  = 0; // Time of the current bit start (ms)
            unsigned long lastChangeT = 0; // Time of the last signal change (ms)
            unsigned int state0    = 0;   // ZEROs detected since bit start
            unsigned int state1    = 0;   // ONEs detected since bit start
            unsigned int lastState = -1;  // Last signal state

            // Current RTTY code
            unsigned int code = 1;        // Currently accumulated RTTY code or 1
            bool figsMode = false;        // TRUE for FIGS mode, FALSE for LTRS

            // Code to character conversion table
            static const char ita2Table[];

            // Debugging data
            unsigned long lastDebugT = 0; // Time of the last debug printout (ms)

            // Convert code to a character
            char ita2char(unsigned int data)
            { return(data<32? ita2Table[data + (figsMode? 32:0)] : '#'); }

            // Get current time in milliseconds
            unsigned long msecs()
            { return(1000*curSeconds + 1000*curSamples/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Process a quantum of input data
            void processInternal(float *data, unsigned int size);

            // Print debug information
            void printDebug();
    };
}
