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

#include <fftw3.h>

namespace Csdr {
    typedef unsigned char pixel;

    class SSTVMode {
        public:
        const char *NAME;

        int ID;
        int COLOR;
        int LINE_WIDTH;
        int LINE_COUNT;

        double SCAN_TIME;
        double SYNC_PULSE;
        double SYNC_PORCH;
        double SEP_PULSE;
        double SEP_PORCH;

        int CHAN_COUNT;
        int CHAN_SYNC;
        double CHAN_TIME;

        double CHAN_OFFSETS[8];
        double LINE_TIME;
        double PIXEL_TIME;
        double WINDOW_FACTOR;

        bool HAS_START_SYNC;
        bool HAS_HALF_SCAN;
        bool HAS_ALT_SCAN;

        double HALF_SCAN_TIME;
        double HALF_PIXEL_TIME;
        double HALF_CHAN_TIME;
    };

    template <typename T>
    class SstvDecoder: public Module<T, unsigned char> {
        public:
            SstvDecoder(unsigned int sampleRate=44100);
            ~SstvDecoder();

            bool canProcess() override;
            void process() override;

        private:
            // Configurable input parameters
            unsigned int sampleRate;   // Input sampling rate (Hz)
            unsigned int dbgTime;      // Debug printout time (ms)

            // Time counting
            unsigned long curSeconds = 0; // Current time in seconds
            unsigned int  curSamples = 0; // Sample count since last second mark

            // Sample buffer
            float *buf = 0;               // Buffer holding current samples
            unsigned int curSize = 0;     // Current data size in buf[]
            unsigned int maxSize = 0;     // Total buf[] size

            // Main FFT
            float *fftIn = 0;            // Input buffer
            fftwf_complex *fftOut = 0;   // Output FFT
            fftwf_plan fftHeader;        // FFT header detection plan
            fftwf_plan fftSync;          // FFT sync detection plan
            fftwf_plan fftPixel;         // FFT pixel decoding plan
            fftwf_plan fftHalfp;         // FFT half-pixel decoding plan
            unsigned int fftSize = 0;    // Current FFT size

            // Total sizes and 2msec step, in samples
            unsigned int hdrSize;        // SSTV header size
            unsigned int wndSize;        // Tone search window size
            unsigned int bitSize;        // VIS bit size
            unsigned int visSize;        // Total VIS size
            unsigned int syncSize;       // Sync window size
            unsigned int pixelSize;      // Pixel window size
            unsigned int halfpSize;      // Half-pixel window size
            unsigned int step;           // 2ms step size

            // Header tone offsets
            unsigned int lead1_Start;
            unsigned int break_Start;
            unsigned int lead2_Start;
            unsigned int vis_Start;

            // Decoder state
            const SSTVMode *curMode;     // Current SSTV mode + parameters
            unsigned int lastLineT;      // Time of last scanline decoded (ms)
            int curState;                // Current decoder state

            // U/V component from a previous scanline (must be large enough!)
            unsigned char linebuf[1024];

            // Debugging data
            unsigned long lastDebugT = 0; // Time of the last debug printout (ms)

            // Get current time in milliseconds
            unsigned long msecs(int dSamples=0)
            { return(1000*curSeconds + 1000*(curSamples+dSamples)/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Print BMP file header
            void printBmpHeader(const SSTVMode *mode);

            // Print BMP file footer
            void printBmpEmptyLines(const SSTVMode *mode, unsigned int lines);

            // Print debug information
            void printDebug();

            // Print given string
            void printString(const char *buf);

            // Print formatted string
            void print(const char *format, ...);

            // Find peak frequency
            int fftPeakFreq(fftwf_plan fft, const float *buf, unsigned int size);

            // Find SSTV header
            unsigned int findHeader(const float *buf, unsigned int size);

            // Decode SSTV VIS section and return mode
            const SSTVMode *decodeVIS(const float *buf, unsigned int size);

            // Find SYNC signal
            unsigned int findSync(const SSTVMode *mode, const float *buf, unsigned int size);

            // Decode single scanline
            unsigned int decodeLine(const SSTVMode *mode, unsigned int line, const float *buf, unsigned int size);

            // Finish frame (if any) and go back to header detection
            void finishFrame(void);

            // Skip input samples
            void skipInput(unsigned int size);

            // Compute luminance from frequency
            unsigned char luminance(int freq);

            // Convert YUV to RGB values
            unsigned int yuv2rgb(unsigned char y, unsigned char u, unsigned char v);
    };
}
