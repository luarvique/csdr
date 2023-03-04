/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2023 Marat Fayzullin <luarvique@gmail.com>
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

#include "noiseblanker.hpp"

#include <cstring>

using namespace Csdr;

template <typename T>
NoiseBlanker<T>::NoiseBlanker(double thr1, double thr2):
    nb1_avgMag(1.0),
    nb2_avgMag(1.0),
    avgSig(0),
    delay{0},
    delIdx(2),
    sigIdx(0),
    hangTime(0)
{
    // < 0.0 means "disabled"
    nb1_threshold = thr1<0.0? -1.0 : thr1<1.0? 1.0 : thr1>20.0? 20.0 : thr1;
    nb2_threshold = thr2<0.0? -1.0 : thr2>15.0? 15.0 : thr2;
}

template<typename T>
size_t NoiseBlanker<T>::apply(T *input, T *output, size_t size)
{
    // Copy input to output
    for(size_t i=0; i<size; ++i) output[i] = input[i];

    // Apply first noise blanking algorithm
    if(nb1_threshold>=0.0) apply_nb1(output, size);

    // Apply second noise blanking algorithm
    if(nb2_threshold>=0.0) apply_nb2(output, size);

    // Done
    return size;
}

//
// Noise blanker 1 is the first noise blanker in the processing chain.
// It is intended to reduce the effect of impulse type noise.
// FIXME: Needs different constants for higher sample rates?
//
template<typename T>
void NoiseBlanker<T>::apply_nb1(T *buf, size_t size)
{
    float mag;

    for(size_t i=0; i<size; ++i)
    {
        mag           = abs(buf[i]);
        delay[sigIdx] = buf[i];
        nb1_avgMag    = 0.999 * nb1_avgMag + 0.001 * mag;

        if((hangTime==0) && (mag>nb1_threshold*nb1_avgMag)) hangTime = 7;

        if(hangTime>0)
        {
            buf[i] = 0;
            --hangTime;
        }
        else
        {
            buf[i] = delay[delIdx];
        }

        sigIdx = (sigIdx + 7) & 7;
        delIdx = (delIdx + 7) & 7;
    }
}

//
// Noise blanker 2 is the second noise blanker in the processing chain.
// It is intended to reduce non-pulse type noise (i.e. longer time constants).
// FIXME: Needs different constants for higher sample rates?
//
template<typename T>
void NoiseBlanker<T>::apply_nb2(T *buf, size_t size)
{
    double mag;
    T c1(0.75);
    T c2(0.25);

    for(size_t i=0; i<size; ++i)
    {
        mag        = abs(buf[i]);
        avgSig     = c1 * avgSig + c2 * buf[i];
        nb2_avgMag = 0.999 * nb2_avgMag + 0.001 * mag;

        if(mag > nb2_threshold * nb2_avgMag) buf[i] = avgSig;
    }
}

namespace Csdr {
    template class NoiseBlanker<complex<float>>;
    template class NoiseBlanker<float>;
}
