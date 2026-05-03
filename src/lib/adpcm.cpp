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

#include "adpcm.hpp"

#include <cstring>
#include <algorithm>

using namespace Csdr;

unsigned char AdpcmCodec::encodeSample(short sample) {
    int diff = sample - previousValue;
    int step = _stepSizeTable[index];
    int deltaCode = 0;

    // Set sign bit
    if (diff < 0) {
        deltaCode = 8;
        diff = -diff;
    }

    // This is essentially deltaCode = (diff<<2)/step,
    // except the roundoff is handled differently.
    if (diff >= step) {
        deltaCode |= 4;
        diff -= step;
    }
    step >>= 1;
    if (diff >= step) {
        deltaCode |= 2;
        diff -= step;
    }
    step >>= 1;
    if (diff >= step) {
        deltaCode |= 1;
        diff -= step;
    }

    decodeSample(deltaCode);  // update state
    return deltaCode;
}

short AdpcmCodec::decodeSample(unsigned char deltaCode) {
    // Get the current step size
    int step = _stepSizeTable[index];

    // Construct the difference by scaling the current step size
    // This is approximately: difference = (deltaCode+.5)*step/4
    int difference = step >> 3;
    if (deltaCode & 1) {
        difference += step >> 2;
    }

    if (deltaCode & 2) {
        difference += step >> 1;
    }

    if (deltaCode & 4) {
        difference += step;
    }

    if (deltaCode & 8) {
        difference = -difference;
    }


    // Build the new sample
    previousValue += difference;
    if (previousValue > 32767) previousValue = 32767;
    else if (previousValue < -32768) previousValue = -32768;

    // Update the step for the next sample
    index += indexAdjustTable[deltaCode];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;

    return previousValue;
}

unsigned char AdpcmCodec::encodeSample(float input) {
    /*
     * TODO: 100 is a magic number here.
     * It only makes sense since the same constant is used in the OpenWebRX client-side javascript code.
     * SHRT_MAX does not work in it's place, so this is more than just a simple conversion.
     */
    return encodeSample((short) (input * 100));
}

void AdpcmCodec::reset() {
    previousValue = 0;
    index = 0;
}

int16_t AdpcmCodec::getIndex() {
    return index;
}

int16_t AdpcmCodec::getPredictor() {
    return (int16_t) previousValue;
}

AdpcmCoder::AdpcmCoder(): codec(new AdpcmCodec()) {}

AdpcmCoder::~AdpcmCoder() {
    delete codec;
}

AdpcmEncoder::AdpcmEncoder(bool sync): sync(sync) {}

bool AdpcmEncoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= 2 && writer->writeable() > 8;
}

void AdpcmEncoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    short* input = reader->getReadPointer();
    unsigned char* output = writer->getWritePointer();
    // clamp to 1000 samples since canProcess() only covers one sync frame for us
    size_t rsize = reader->available() & ~1;
    size_t wsize = writer->writeable();
    size_t ridx, widx;
    for (ridx = widx = 0; (ridx < rsize) && (widx < wsize) ; ridx+=2, ++widx) {
        if (sync && syncCounter-- <= 0) {
            if (widx + 8 >= wsize) {
                break;
            } else {
                std::memcpy(output + widx, "SYNC", 4);
                int16_t* data = (int16_t*) (output + widx + 4);
                data[0] = codec->getIndex();
                data[1] = codec->getPredictor();
                widx += 8;
                syncCounter = 1000;
            }
        }
        output[widx] =
                codec->encodeSample(input[ridx]) |
                codec->encodeSample(input[ridx + 1]) << 4;
    }
    reader->advance(ridx);
    writer->advance(widx);
}

bool AdpcmDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0 && writer->writeable() >= 10;
}

void AdpcmDecoder::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    unsigned char* input = reader->getReadPointer();
    short* output = writer->getWritePointer();
    size_t size = std::min(reader->available(), writer->writeable() / 2);
    for (int i = 0; i < size; i++) {
        output[i * 2] = codec->decodeSample(input[i] & 0x0f);
        output[i * 2 + 1] = codec->decodeSample(input[i] >> 4);
    }
    reader->advance(size);
    writer->advance(size * 2);
}

FftAdpcmEncoder::FftAdpcmEncoder(unsigned int fftSize): fftSize(fftSize) {}

bool FftAdpcmEncoder::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() >= fftSize && (COMPRESS_FFT_PAD_N + writer->writeable()) / 2 > fftSize;
}

void FftAdpcmEncoder::process() {
    std::lock_guard<std::mutex> lock(processMutex);
    float* input = reader->getReadPointer();
    unsigned char* output = writer->getWritePointer();
    // FFT always starts with the codec default values
    codec->reset();
    for (int i = 0; i < COMPRESS_FFT_PAD_N / 2; i++) {
        output[i] =
                // the cast only serves to help the compiler decide which function to call
                codec->encodeSample(input[0]) |
                codec->encodeSample(input[0]) << 4;
    }
    output += (COMPRESS_FFT_PAD_N / 2);
    for (size_t i = 0; i < fftSize / 2; i++) {
        output[i] =
                codec->encodeSample(input[i * 2]) |
                codec->encodeSample(input[i * 2 + 1]) << 4;
    }
    reader->advance(fftSize);
    writer->advance((COMPRESS_FFT_PAD_N + fftSize) / 2);
}
