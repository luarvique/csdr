/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2023 Jakob Ketterl <jakob.ketterl@gmx.de>
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

#include "noise.hpp"
#include <climits>

using namespace Csdr;

template <typename T>
NoiseSource<T>::NoiseSource() {
    random = fopen("/dev/urandom", "r");
}

template <typename T>
NoiseSource<T>::~NoiseSource() {
    fclose(random);
}

template <typename T>
void NoiseSource<T>::setWriter(Writer<T> *writer) {
    Source<T>::setWriter(writer);
    if (thread == nullptr) {
        thread = new std::thread( [this] () { loop(); });
    }
}

template <typename T>
void NoiseSource<T>::loop() {
    while (true) {
        generateSamples(this->writer->getWritePointer(), 1024);
        this->writer->advance(1024);
    }
}

template <>
void NoiseSource<complex<float>>::generateSamples(complex<float> *output, size_t length) {
    int* pioutput = (int*)output;
    fread(output, sizeof(complex<float>), length, random);
    for(int i = 0; i < length * 2; i++) {
        complex<float> tempi = {
                pioutput[i * 2],
                pioutput[i * 2 + 1]
        };
        output[i] = {
                tempi.i() / ((float) (INT_MAX)),
                tempi.q() / ((float) (INT_MAX))
        };
    }

}

namespace Csdr {
    template class NoiseSource<complex<float>>;
}