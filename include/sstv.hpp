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
    // SSTV color modes
    enum SstvColor {
        COLOR_RGB = 1,
        COLOR_GBR = 2,
        COLOR_YUV = 3,
        COLOR_BW  = 4,
    };

    // SSTV VIS types
    enum SstvVIS {
        VIS_ROBOT12   = 0,
        VIS_ROBOT24   = 4,
        VIS_ROBOT36   = 8,
        VIS_ROBOT72   = 12,
        VIS_MARTIN4   = 32,
        VIS_MARTIN3   = 36,
        VIS_MARTIN2   = 40,
        VIS_MARTIN1   = 44,
        VIS_SCOTTIE4  = 48,
        VIS_SC2_30    = 51,
        VIS_SCOTTIE3  = 52,
        VIS_SC2_180   = 55,
        VIS_SCOTTIE2  = 56,
        VIS_SC2_60    = 59,
        VIS_SCOTTIE1  = 60,
        VIS_SC2_120   = 63,
        VIS_AVT90     = 68,
        VIS_SCOTTIEDX = 76,
        VIS_PD50      = 93,
        VIS_PD290     = 94,
        VIS_PD120     = 95,
        VIS_PD180     = 96,
        VIS_PD240     = 97,
        VIS_PD160     = 98,
        VIS_PD90      = 99,
        VIS_UNKNOWN   = 127
    };

    class SstvMode {
        public:
            const char *NAME = "Invalid";

            int ID         = VIS_UNKNOWN;
            int COLOR      = 0;
            int LINE_WIDTH = 320;
            int LINE_COUNT = 256;
            int LINE_STEP  = 1;
            int CHAN_COUNT = 3;
            int CHAN_SYNC  = 0;

            double SCAN_TIME  = 0.0;
            double SYNC_PULSE = 0.0;
            double SYNC_PORCH = 0.0;
            double SEP_PULSE  = 0.0;
            double SEP_PORCH  = 0.0;
            double WINDOW_FACTOR  = 1.0;

            bool HAS_START_SYNC = false;

            // These values are usually computed automatically
            double LINE_TIME;
            double CHAN_TIME;
            double PIXEL_TIME;
            double HALF_PIXEL_TIME;
            double CHAN_OFFSETS[8];

            // FFT plans are created with createPlans()
            unsigned int syncSize;   // Sync window size
            unsigned int pixelSize;  // Pixel window size
            unsigned int halfpSize;  // Half-pixel window size
            fftwf_plan fftSync  = 0; // Sync detection plan
            fftwf_plan fftPixel = 0; // Pixel decoding plan
            fftwf_plan fftHalfp = 0; // Half-pixel decoding plan
            unsigned int sampleRate = 0;
            fftwf_complex *fftOut   = 0;
            float *fftIn            = 0;

            // Destructor
            ~SstvMode() { destroyPlans(); }

            void createPlans(unsigned int rate, fftwf_complex *out, float *in);
            void destroyPlans();

            void computeTimings() {
                CHAN_TIME  = SEP_PULSE + SCAN_TIME;
                LINE_TIME  = SYNC_PULSE + SYNC_PORCH + CHAN_COUNT*CHAN_TIME;
                PIXEL_TIME = SCAN_TIME / LINE_WIDTH;
                HALF_PIXEL_TIME = SCAN_TIME / 2.0 / LINE_WIDTH;

                CHAN_OFFSETS[0] = SYNC_PULSE + SYNC_PORCH;
                CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME;
                CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME;
            }
    };

    class Martin1: public SstvMode {
        public:
            Martin1() {
                NAME       = "Martin 1";
                ID         = VIS_MARTIN1;
                COLOR      = COLOR_GBR;
                LINE_WIDTH = 320;
                LINE_COUNT = 256;
                SCAN_TIME  = 0.146432;
                SYNC_PULSE = 0.004862;
                SYNC_PORCH = 0.000572;
                SEP_PULSE  = 0.000572;
                WINDOW_FACTOR = 2.34;

                computeTimings();
            }
    };

    class Martin2: public Martin1 {
        public:
            Martin2() {
                NAME       = "Martin 2";
                ID         = VIS_MARTIN2;
                LINE_WIDTH = 320;
                SCAN_TIME  = 0.073216;
                SYNC_PULSE = 0.004862;
                SYNC_PORCH = 0.000572;
                SEP_PULSE  = 0.000572;
                WINDOW_FACTOR = 4.68;

                computeTimings();
            }
    };

    class Martin3: public Martin1 {
        public:
            Martin3() {
                NAME       = "Martin 3";
                ID         = VIS_MARTIN3;
                LINE_COUNT = 128;

                computeTimings();
            }
    };

    class Martin4: public Martin2 {
        public:
            Martin4() {
                NAME       = "Martin 4";
                ID         = VIS_MARTIN4;
                LINE_COUNT = 128;

                computeTimings();
            }
    };

    class Scottie1: public SstvMode {
        public:
            Scottie1() {
                NAME       = "Scottie 1";
                ID         = VIS_SCOTTIE1;
                COLOR      = COLOR_GBR;
                LINE_WIDTH = 320;
                LINE_COUNT = 256;
                SCAN_TIME  = 0.13824;
                SYNC_PULSE = 0.00900;
                SYNC_PORCH = 0.00150;
                SEP_PULSE  = 0.00150;
                CHAN_SYNC  = 2;
                WINDOW_FACTOR  = 2.48;
                HAS_START_SYNC = true;

                computeTimings();
            }

            void computeTimings() {
                SstvMode::computeTimings();
                // Sync is in the middle
                CHAN_OFFSETS[0] = SEP_PULSE;
                CHAN_OFFSETS[1] = SEP_PULSE + CHAN_TIME;
                CHAN_OFFSETS[2] = 2*CHAN_TIME + SYNC_PULSE + SYNC_PORCH;
                LINE_TIME       = SYNC_PULSE + CHAN_COUNT*CHAN_TIME;
            }
    };

    class Scottie2: public Scottie1 {
        public:
            Scottie2() {
                NAME       = "Scottie 2";
                ID         = VIS_SCOTTIE2;
                LINE_WIDTH = 320;
                SCAN_TIME  = 0.088064;
                SYNC_PULSE = 0.009000;
                SYNC_PORCH = 0.001500;
                SEP_PULSE  = 0.001500;
                WINDOW_FACTOR = 3.82;

                computeTimings();
            }
    };

    class Scottie3: public Scottie1 {
        public:
            Scottie3() {
                NAME       = "Scottie 3";
                ID         = VIS_SCOTTIE3;
                LINE_COUNT = 128;

                computeTimings();
            }
    };

    class Scottie4: public Scottie2 {
        public:
            Scottie4() {
                NAME       = "Scottie 4";
                ID         = VIS_SCOTTIE4;
                LINE_COUNT = 128;

                computeTimings();
            }
    };

    class ScottieDX: public Scottie2 {
        public:
            ScottieDX() {
                NAME       = "Scottie DX";
                ID         = VIS_SCOTTIEDX;
                LINE_WIDTH = 320;
                SCAN_TIME  = 0.3456;
                SYNC_PULSE = 0.0090;
                SYNC_PORCH = 0.0015;
                SEP_PULSE  = 0.0015;
                WINDOW_FACTOR = 0.98;

                computeTimings();
            }
    };

    class Robot36: public SstvMode {
        public:
            Robot36() {
                NAME       = "Robot 36";
                ID         = VIS_ROBOT36;
                COLOR      = COLOR_YUV;
                LINE_WIDTH = 320;
                LINE_COUNT = 240;
                SCAN_TIME  = 0.0880;
                SYNC_PULSE = 0.0090;
                SYNC_PORCH = 0.0030;
                SEP_PULSE  = 0.0045;
                SEP_PORCH  = 0.0015;
                CHAN_COUNT = 2;
                WINDOW_FACTOR  = 7.70;

                computeTimings();
            }

            void computeTimings() {
                SstvMode::computeTimings();
                // Only two channels, channel #1 is half width
                CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;
                CHAN_OFFSETS[2] = CHAN_OFFSETS[1];
                LINE_TIME       = CHAN_OFFSETS[1] + SCAN_TIME / 2.0;
            }
    };

    class Robot72: public Robot36 {
        public:
            Robot72() {
                NAME       = "Robot 72";
                ID         = VIS_ROBOT72;
                SCAN_TIME  = 0.1380;
                CHAN_COUNT = 3;
                WINDOW_FACTOR = 4.88;

                computeTimings();
            }

            void computeTimings() {
                SstvMode::computeTimings();
                // Channels #1 and #2 are half width
                CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME + SEP_PORCH;
                CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME / 2.0 + SEP_PORCH;
                LINE_TIME       = CHAN_OFFSETS[2] + SCAN_TIME / 2.0;
            }
    };

    class Robot12: public Robot36 {
        public:
            Robot12() {
                NAME       = "Robot 12";
                ID         = VIS_ROBOT12;
                LINE_WIDTH = 160;
                LINE_COUNT = 120;
                SCAN_TIME  = 0.0600;
                WINDOW_FACTOR = 2.81;

                computeTimings();
            }
    };

    class Robot24: public Robot72 {
        public:
            Robot24() {
                NAME       = "Robot 24";
                ID         = VIS_ROBOT24;
                LINE_WIDTH = 160;
                LINE_COUNT = 120;
                SCAN_TIME  = 0.0880;
                WINDOW_FACTOR = 3.83;

                computeTimings();
            }
    };

    class PD50: public SstvMode {
        public:
            PD50() {
                NAME       = "PD-50";
                ID         = VIS_PD50;
                COLOR      = COLOR_YUV;
                LINE_WIDTH = 320;
                LINE_COUNT = 256;
                LINE_STEP  = 2;
                SCAN_TIME  = 0.09152;
                SYNC_PULSE = 0.02000;
                SYNC_PORCH = 0.00208;
                SEP_PULSE  = 0.00000;
                WINDOW_FACTOR = 3.74;

                computeTimings();
            }
    };

    class PD90: public PD50 {
        public:
            PD90() {
                NAME      = "PD-90";
                ID        = VIS_PD90;
                SCAN_TIME = 0.17024;
                WINDOW_FACTOR = 2.01;

                computeTimings();
            }
    };

    class PD120: public PD50 {
        public:
            PD120() {
                NAME       = "PD-120";
                ID         = VIS_PD120;
                LINE_WIDTH = 640;
                LINE_COUNT = 496;
                SCAN_TIME  = 0.1216;
                WINDOW_FACTOR = 5.63;

                computeTimings();
            }
    };

    class PD160: public PD50 {
        public:
            PD160() {
                NAME       = "PD-160";
                ID         = VIS_PD160;
                LINE_WIDTH = 512;
                LINE_COUNT = 400;
                SCAN_TIME  = 0.195854;
                WINDOW_FACTOR = 2.79;

                computeTimings();
            }
    };

    class PD180: public PD120 {
        public:
            PD180() {
                NAME      = "PD-180";
                ID        = VIS_PD180;
                SCAN_TIME = 0.18304;
                WINDOW_FACTOR = 3.74;

                computeTimings();
            }
    };

    class PD240: public PD120 {
        public:
            PD240() {
                NAME      = "PD-240";
                ID        = VIS_PD240;
                SCAN_TIME = 0.24448;
                WINDOW_FACTOR = 2.80;

                computeTimings();
            }
    };

    class PD290: public PD50 {
        public:
            PD290() {
                NAME       = "PD-290";
                ID         = VIS_PD290;
                LINE_WIDTH = 800;
                LINE_COUNT = 616;
                SCAN_TIME  = 0.2288;
                WINDOW_FACTOR = 3.74;

                computeTimings();
            }
    };

    class AVT90: public SstvMode {
        public:
            AVT90() {
                NAME       = "AVT-90";
                ID         = VIS_AVT90;
                COLOR      = COLOR_RGB;
                LINE_WIDTH = 256;
                LINE_COUNT = 240;
                SCAN_TIME  = 0.125;
                SYNC_PULSE = 0.000;
                SYNC_PORCH = 0.000;
                SEP_PULSE  = 0.000;
                WINDOW_FACTOR = 2.74;

                computeTimings();
            }
    };

    class SC2_60: public SstvMode {
        public:
            SC2_60() {
                NAME       = "Wraase SC2-60";
                ID         = VIS_SC2_60;
                COLOR      = COLOR_RGB;
                LINE_WIDTH = 320;
                LINE_COUNT = 256;
                SCAN_TIME  = 0.117;
                SYNC_PULSE = 0.005;
                SYNC_PORCH = 0.000;
                SEP_PULSE  = 0.000;
                WINDOW_FACTOR = 5.91;

                computeTimings();
            }

            void computeTimings() {
                SstvMode::computeTimings();
                // Channels #0 (RED) and #2 (BLUE) are half width
                CHAN_OFFSETS[1] = CHAN_OFFSETS[0] + CHAN_TIME / 2.0;
                CHAN_OFFSETS[2] = CHAN_OFFSETS[1] + CHAN_TIME;
                LINE_TIME       = CHAN_OFFSETS[2] + CHAN_TIME / 2.0;
            }
    };

    class SC2_30: public SC2_60 {
        public:
            SC2_30() {
                NAME       = "Wraase SC2-30";
                ID         = VIS_SC2_30;
                LINE_COUNT = 128;

                computeTimings();
            }
    };

    class SC2_120: public SC2_60 {
        public:
            SC2_120() {
                NAME      = "Wraase SC2-120";
                ID        = VIS_SC2_120;
                SCAN_TIME = 0.235;
                WINDOW_FACTOR = 2.93;

                computeTimings();
            }
    };

    class SC2_180: public SC2_60 {
        public:
            SC2_180() {
                NAME      = "Wraase SC2-180";
                ID        = VIS_SC2_180;
                SCAN_TIME = 0.235;
                WINDOW_FACTOR = 1.46;

                // All channels are same length
                SstvMode::computeTimings();
            }
    };

    template <typename T>
    class SstvDecoder: public Module<T, unsigned char> {
        public:
            SstvDecoder(unsigned int sampleRate=44100, unsigned int dbgTime=0);
            ~SstvDecoder();

            bool canProcess() override;
            void process() override;

        private:
            // Maximum scanline width
            enum { MAX_LINE_WIDTH = 640 };

            // Configurable input parameters
            unsigned int sampleRate;   // Input sampling rate (Hz)
            unsigned int dbgTime;      // Debug printout time (ms)

            // Time counting
            unsigned long curSeconds = 0; // Current time in seconds
            unsigned int  curSamples = 0; // Sample count since last second mark

            // Main FFT
            float *fftIn = 0;            // Input buffer
            fftwf_complex *fftOut = 0;   // Output FFT
            fftwf_plan fftHeader;        // FFT header detection plan

            // Total sizes and 2msec step, in samples
            unsigned int hdrSize;        // SSTV header size
            unsigned int wndSize;        // Tone search window size
            unsigned int bitSize;        // VIS bit size
            unsigned int visSize;        // Total VIS size
            unsigned int step;           // 2ms step size

            // Header tone offsets
            unsigned int lead1_Start;
            unsigned int break_Start;
            unsigned int lead2_Start;
            unsigned int vis_Start;

            // Decoder state
            SstvMode *modes[128];        // Mapping from VIS to SSTV modes
            SstvMode *curMode;           // Current SSTV mode + parameters
            unsigned int lastLineT;      // Time of last scanline decoded (ms)
            int curState;                // Current decoder state

            // U/V components from a previous scanline (must be large enough!)
            unsigned char linebuf[2][MAX_LINE_WIDTH];

            // Debugging data
            unsigned long lastDebugT = 0; // Time of the last debug printout (ms)

            // Get current time in milliseconds
            unsigned long msecs(int dSamples=0)
            { return(1000*curSeconds + 1000*(curSamples+dSamples)/sampleRate); }

            // Get number of samples in given number of milliseconds
            unsigned int ms2smp(unsigned int msec)
            { return(sampleRate * msec / 1000); }

            // Print BMP file header
            void printBmpHeader(const SstvMode *mode);

            // Print BMP file footer
            void printBmpEmptyLines(const SstvMode *mode, unsigned int lines);

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
            SstvMode *decodeVIS(const float *buf, unsigned int size);

            // Find SYNC signal
            unsigned int findSync(const SstvMode *mode, const float *buf, unsigned int size);

            // Decode single scanline
            unsigned int decodeLine(const SstvMode *mode, unsigned int line, const float *buf, unsigned int size);

            // Finish frame (if any) and go back to header detection
            void finishFrame();

            // Skip input samples
            void skipInput(unsigned int size);

            // Compute luminance from frequency
            unsigned char luminance(int freq);

            // Convert YUV to RGB values
            unsigned int yuv2rgb(unsigned char y, unsigned char u, unsigned char v);

            // Write output data
            bool writeData(const void *buf, unsigned int size);

            // Individual scanline conversion functions for different modes
            void convertYUV(const SstvMode *mode, unsigned int line, unsigned char *buf[3]);
            void convertRGB(const SstvMode *mode, unsigned int line, unsigned char *buf[3]);
            void convertGBR(const SstvMode *mode, unsigned int line, unsigned char *buf[3]);
            void convertR36(const SstvMode *mode, unsigned int line, unsigned char *buf[3]);
            void convertPD(const SstvMode *mode, unsigned int line, unsigned char *buf[3]);
    };
}
