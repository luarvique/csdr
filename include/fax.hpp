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
    template <typename T>
    class FaxDecoder: public Module<T, unsigned char> {
        public:
            enum
            {
                OPT_FM     = 0x0000,
                OPT_AM     = 0x0001,
                OPT_MONO   = 0x0000,
                OPT_COLOR  = 0x0002,
                OPT_SYNC   = 0x0004,
            };

            FaxDecoder(
                unsigned int sampleRate = 44100,
                unsigned int lpm        = 120,
                unsigned int options    = 0,
                unsigned int dbgTime    = 0
            );

            ~FaxDecoder();

            bool canProcess() override;
            void process() override;

        private:
            enum
            {
                FIR_NARROW = 0,
                FIR_MIDDLE = 1,
                FIR_WIDE   = 2,
                FIR_LENGTH = 17,

                FREQ_IOC576    = 300,
                FREQ_IOC288    = 675,
                FREQ_STOP      = 450,
                FREQ_CARRIER   = 1900,
                FREQ_DEVIATION = 400,

                HEIGHT_IOC576  = 1400,
                HEIGHT_IOC288  = 700,

//                HEIGHT_IOC576  = 1200,
//                HEIGHT_IOC288  = 600,

                WIDTH_IOC576   = 1812,
                WIDTH_IOC288   = 908,
            };

            class FirFilter
            {
                public:
                    FirFilter(int w = FIR_MIDDLE): bandwidth(w) {}

                    // Apply FIR filter to the next sample
                    double process(double sample);

                private:
                    int bandwidth;
                    int current = 0;
                    double buffer[FIR_LENGTH] = {0};
            };

            // Configurable input parameters
            unsigned int sampleRate;   // Input sampling rate (Hz)
            unsigned int dbgTime;      // Debug printout time (ms)

            // Time counting
            unsigned long curSeconds = 0; // Current time in seconds
            unsigned int  curSamples = 0; // Sample count since last second mark

            // Sample buffer
            unsigned char *buf = 0;       // Buffer holding current samples
            unsigned int curSize = 0;     // Current data size in buf[]
            unsigned int maxSize = 0;     // Total buf[] size

            // Buffers holding last three lines
            unsigned char *line[3] = {0, 0, 0};

            // Require 4 less lines than there really are to handle noise
            // and also misalignment on first and last lines.
            int leewayLines = 6;//4;

            // Throw away first 2 lines of phasing because we are not sure
            // if they are misaligned start lines.
            int phasingSkipLines = 2;

            // Configuration
            int phasingLines = 40;
            int lpm          = 120;
            int ioc          = 576;
            int lineWidth    = WIDTH_IOC576;
            int maxLines     = HEIGHT_IOC576;
            int colors       = 1;
            int carrier      = FREQ_CARRIER;
            int deviation    = FREQ_DEVIATION;
            int filter       = FIR_NARROW;
            int fm           = 1;
            int syncLines    = 0;
            int startFreq    = FREQ_IOC576;
            int stopFreq     = FREQ_STOP;
            int startLength  = 5;
            int stopLength   = 5;

            // Decoder state
            unsigned int lastLineT;      // Time of last scanline decoded (ms)
            int blockSize;               // Decoding block size in samples
            int curState;                // Current decoder state
            int curLine;                 // Current scanline
            int lastType;                // Last scanline type
            int typeCount;               // Number of times lastType repeated
            int tailLines;               // Empty tail lines left to output

            // Demodulator state
            FirFilter filters[2];
            double fstep;
            double coeff;

            // Phasing state
            int phasingSkipData;
            int *phasingPos;

            // Debugging data
            unsigned long lastDebugT = 0; // Time of the last debug printout (ms)

            // Get current time in milliseconds
            unsigned long msecs(int dSamples=0)
            { return(1000*curSeconds + 1000*(curSamples+dSamples)/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Print BMP file header
            void printBmpHeader();

            // Process and print BMP scanline
            void printBmpLine(int lineN);

            // Print BMP file footer, returns number of lines actually printed
            unsigned int printBmpEmptyLines(unsigned int lines);

            // Print debug information
            void printDebug();

            // Print given string
            void printString(const char *buf);

            // Print formatted string
            void print(const char *format, ...);

            // Find phasing line position
            int phasingLinePosition(const unsigned char *buf, unsigned int size);

            // Find phasing line position (newer version)
            int findSync(const unsigned char *buf, unsigned int size);

            // Decode line type
            int decodeLineType(const unsigned char *buf, unsigned int size);

            // Decode an image line
            int decodeImageLine(const unsigned char *buf, unsigned int size, unsigned char *image, unsigned int width, unsigned int colors);

            // Finish page (if any) and go back to header detection
            void finishPage();

            // Check for specific frequency.
            double fftSub(const unsigned char *buf, unsigned int size, int freq);

            // Skip input samples
            void skipInput(unsigned int size);

            // Write output data
            bool writeData(const void *buf, unsigned int size);
    };
}
