/*
Copyright (c) 2023 Marat Fayzullin <luarvique@gmail.com>

This file is part of libcsdr.

libcsdr is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libcsdr is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with libcsdr.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "afc.hpp"
#include "complex.hpp"
#include <string.h>
#include <stdio.h>

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

Afc::Afc(unsigned int updatePeriod):
    ShiftAddfast(0.0),
    updatePeriod(updatePeriod),
    updateCount(0),
    curShift(0.0)
{
    unsigned int size = getLength();

    // Set up FFT
    fftIn   = fftwf_alloc_complex(size);
    fftOut  = fftwf_alloc_complex(size);
    fftPlan = fftwf_plan_dft_1d(size, fftIn, fftOut, FFTW_FORWARD, CSDR_FFTW_FLAGS);
}

Afc::~Afc()
{
    // Destroy FFT
    fftwf_destroy_plan(fftPlan);
    fftwf_free(fftIn);
    fftwf_free(fftOut);
}

void Afc::process(complex<float>* input, complex<float>* output)
{
    unsigned int size = getLength();

    if(++updateCount>=updatePeriod)
    {
        float maxMag;
        int j, i;

        // Reset update counter
        updateCount = 0;

        // Calculate FFT on the input buffer
        memcpy(fftIn, input, size * sizeof(fftIn[0]));
        fftwf_execute(fftPlan);

        // Find the carrier frequency
        maxMag = fftOut[0][0]*fftOut[0][0] + fftOut[0][1]*fftOut[0][1];
        for(j=1, i=0 ; j<size ; ++j)
        {
            float mag = fftOut[j][0]*fftOut[j][0] + fftOut[j][1]*fftOut[j][1];
            if(mag>maxMag) { i=j;maxMag=mag; }
        }

        // Take negative shifts into account
        i = i>=size/2? size-i : -i;

        // Slowly update frequency shift
        curShift += ((double)i/size - curShift) / 10.0;
        setRate(curShift);
    }

    // Shift frequency
    process_fmv(input, output, size);
}
