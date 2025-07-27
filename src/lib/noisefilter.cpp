/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

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

#include "noisefilter.hpp"

#include <cstring>

using namespace Csdr;

#define MAX_SCALES (16)

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

template <typename T>
NoiseFilter<T>::NoiseFilter(size_t fftSize, size_t wndSize, unsigned int decay, unsigned int attack)
{
    // Keep FFT and overlap sizes reasonable
    this->fftSize = fftSize = fftSize>=32? fftSize : 32;
    this->ovrSize = fftSize>=64? (fftSize>>6) : 1;

    // Make sure window does not exceed half of the FFT size
    wndSize = wndSize>fftSize/2? fftSize/2 : wndSize;

    // Make sure window does not exceed unsigned char resolution
    wndSize = wndSize<2? 2 : wndSize>254? 254 : wndSize;

    // We are really interested in half-a-window
    this->wndSize = wndSize>>1;

    // Keep attack and decay positive
    this->attack    = attack>0? attack : 1;
    this->decay     = decay>0? decay : 1;
    this->threshold = 1.0;
    this->avgPower  = 0.0;

    // Allocate FFT buffers and plans
    overlapBuf    = fftwf_alloc_complex(ovrSize);
    forwardInput  = fftwf_alloc_complex(fftSize);
    forwardOutput = fftwf_alloc_complex(fftSize);
    forwardPlan   = fftwf_plan_dft_1d(fftSize, forwardInput, forwardOutput, FFTW_FORWARD, CSDR_FFTW_FLAGS);
    inverseInput  = fftwf_alloc_complex(fftSize);
    inverseOutput = fftwf_alloc_complex(fftSize);
    inversePlan   = fftwf_plan_dft_1d(fftSize, inverseInput, inverseOutput, FFTW_BACKWARD, CSDR_FFTW_FLAGS);

    // Fill with zeros so that the padding works
    for(size_t i = 0; i < fftSize; i++)
    {
        forwardInput[i][0] = 0.0f;
        forwardInput[i][1] = 0.0f;
    }
}

template<typename T>
NoiseFilter<T>::~NoiseFilter()
{
    fftwf_destroy_plan(forwardPlan);
    fftwf_free(forwardInput);
    fftwf_free(forwardOutput);
    fftwf_destroy_plan(inversePlan);
    fftwf_free(inverseInput);
    fftwf_free(inverseOutput);
    fftwf_free(overlapBuf);
}

template <typename T>
void NoiseFilter<T>::setThreshold(int dBthreshold)
{
    // Using power decibels here (square of amplitude)
    this->threshold = pow(10.0, (double)dBthreshold/20.0);
}

template<typename T>
size_t NoiseFilter<T>::apply(T *input, T *output, size_t size)
{
    size_t inputSize = fftSize - ovrSize;

    // Copy input but only partially fill FFT input
    auto* data = (complex<float>*) forwardInput;
    size_t dataSize = inputSize<size? inputSize : size;
    for(size_t i=0; i<dataSize; ++i)
        data[i] = input[i];

    // Calculate FFT on input buffer
    fftwf_execute(forwardPlan);

    struct { double power; int count; } scales[MAX_SCALES];

    auto* in = (complex<float>*) forwardOutput;
    auto* out = (complex<float>*) inverseInput;

    unsigned char gate[fftSize];
    unsigned char gain[fftSize];
    double level[fftSize];
    size_t i, j, k, n;
    double power;

    // Sort buckets into scales
    memset(scales, 0, sizeof(scales));
    for(i=0 ; i<fftSize ; ++i)
    {
        double v = std::norm(in[i]);
        int scale = floor(log(v) + MAX_SCALES/2);
        scale = scale<0? 0 : scale>MAX_SCALES-1? MAX_SCALES-1 : scale;
        scales[scale].power += v;
        scales[scale].count++;
        level[i] = v;
    }

    // Find most populated scales and use them for median power
    for(i=0, n=0, power=0.0 ; (i<MAX_SCALES-1) && (n<fftSize/2) ; ++i)
    {
        // Look for the next most populated scale
        for(k=i, j=i+1 ; j<MAX_SCALES ; ++j)
            if(scales[j].count>scales[k].count) k = j;

        // If found, swap with current one
        if(k!=i)
        {
            double v = scales[k].power;
            j = scales[k].count;
            scales[k] = scales[i];
            scales[i].power = v;
            scales[i].count = j;
        }

        // Keep track of the total number of buckets
        power += scales[i].power;
        n += scales[i].count;
    }

    // Compute the median power
    power /= n;

    // Track the peak median power over multiple FFTs
    avgPower += (power - avgPower) / (power>avgPower? attack : decay);

    // Calculate the effective threshold to compare against
    power = avgPower * threshold;

    // Compare signal's squared level against threshold
    for(i=0 ; i<fftSize ; ++i)
        gate[i] = level[i]>power? 1:0;

    // Compute initial gain for the first entry
    gain[0] = 0;
    for(i=0 ; i<wndSize ; ++i)
        gain[0] += gate[i] + gate[fftSize - i - 1];

    // Incrementally compute gains by moving window over gates
    int prev = fftSize - wndSize;
    int next = wndSize;
    for(i=1 ; i<fftSize ; ++i)
    {
        gain[i] = gain[i-1] + gate[next] - gate[prev];
        if(++prev>=fftSize) prev = 0;
        if(++next>=fftSize) next = 0;
    }

    // Filter out frequencies falling below threshold
    for(i=0 ; i<fftSize ; ++i)
        out[i] = gain[i]? in[i] * std::sqrt((float)gain[i]/(wndSize*2)) : 0.0f;

    // Calculate inverse FFT on the filtered buffer
    fftwf_execute(inversePlan);

    // Add the overlap of the previous segment
    auto result = (complex<float>*) inverseOutput;
    auto overlap = (complex<float>*) overlapBuf;

    // Blend with the overlap
    for(i=0 ; i<ovrSize ; ++i)
    {
        float f = (float)i/ovrSize;
        result[i] = (result[i]/(float)fftSize)*f + overlap[i]*(1.0f-f);
    }

    // Normalize the rest
    for(i=ovrSize ; i<fftSize ; ++i)
        result[i] /= fftSize;

    // Save overlap for the next time
    std::memcpy(overlap, result + inputSize, sizeof(complex<float>) * ovrSize);

    // Copy output but only partially fill FFT output
    for(i=0 ; i<inputSize ; ++i)
        output[i] = complex2sample(result[i]);

    // Done
    return inputSize;
}

template <>
inline complex<float> NoiseFilter<complex<float>>::complex2sample(complex<float> input)
{
    return input;
}

template<>
inline float NoiseFilter<float>::complex2sample(complex<float> input)
{
    return input.i();
}

namespace Csdr {
    template class NoiseFilter<complex<float>>;
    template class NoiseFilter<float>;
}
