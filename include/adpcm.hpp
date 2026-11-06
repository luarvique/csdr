/*

This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2015, Andras Retzler <randras@sdr.hu>
Copyright (c) 2021 Jakob Ketterl <jakob.ketterl@gmx.de>
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

   Copyright 1997 Tim Kientzle.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
      This product includes software developed by Tim Kientzle
      and published in ``The Programmer's Guide to Sound.''
4. Neither the names of Tim Kientzle nor Addison-Wesley
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL TIM KIENTZLE OR ADDISON-WESLEY BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/***********************************************************
Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
Netherlands.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted,
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in
supporting documentation, and that the names of Stichting Mathematisch
Centrum or CWI not be used in advertising or publicity pertaining to
distribution of the software without specific, written prior permission.

STICHTING MATHEMATISCH CENTRUM DISCLAIMS ALL WARRANTIES WITH REGARD TO
THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS, IN NO EVENT SHALL STICHTING MATHEMATISCH CENTRUM BE LIABLE
FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

******************************************************************/
#pragma once

#include "module.hpp"

namespace Csdr {

    class AdpcmCodec {
        public:
            unsigned char encodeSample(short sample);
            short decodeSample(unsigned char deltaCode);
            // for FFT use only
            unsigned char encodeSample(float input);
            void reset();
            int16_t getIndex();
            int16_t getPredictor();
        private:
            int16_t index = 0;         // Index into step size table
            int32_t previousValue = 0; // Most recent sample value
            const int _stepSizeTable[89] = {
                    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34,
                    37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
                    157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494,
                    544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552,
                    1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
                    4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
                    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
                    27086, 29794, 32767
            };
            const int indexAdjustTable[16] = {
                    -1, -1, -1, -1,  // +0 - +3, decrease the step size
                    2, 4, 6, 8,      // +4 - +7, increase the step size
                    -1, -1, -1, -1,  // -0 - -3, decrease the step size
                    2, 4, 6, 8,      // -4 - -7, increase the step size
            };
    };

    class AdpcmCoder {
        protected:
            AdpcmCoder();
            ~AdpcmCoder();
            AdpcmCodec* codec;
    };

    class AdpcmEncoder: private AdpcmCoder, public Module<short, unsigned char> {
        public:
            explicit AdpcmEncoder(bool sync = false);
            bool canProcess() override;
            void process() override;
        private:
            bool sync;
            int syncCounter = 0;
    };

    class AdpcmDecoder: private AdpcmCoder, public Module<unsigned char, short> {
        public:
            bool canProcess() override;
            void process() override;
    };

//We will pad the FFT at the beginning, with the first value of the input data, COMPRESS_FFT_PAD_N times.
//No, this is not advanced DSP, just the ADPCM codec produces some gabarge samples at the beginning,
//so we just add data to become garbage and get skipped.
//COMPRESS_FFT_PAD_N should be even.
#define COMPRESS_FFT_PAD_N 10

    // ADPCM encoding for the FFT works a bit different
    class FftAdpcmEncoder: private AdpcmCoder, public Module<float, unsigned char> {
        public:
            explicit FftAdpcmEncoder(unsigned int fftSize);
            bool canProcess() override;
            void process() override;
        private:
            unsigned int fftSize;
    };

    // Stereo ADPCM encoding interleaves two output streams
    class StereoAdpcmEncoder: public Module<complex<short>, unsigned char> {
        public:
            explicit StereoAdpcmEncoder(bool sync = false);
            bool canProcess() override;
            void process() override;
        private:
            bool sync;
            int syncCounter = 0;
            AdpcmCodec codec[2];
    };
}
