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
#include <stdlib.h>

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

Afc::Afc(unsigned int updatePeriod, unsigned int samplePeriod): ShiftAddfast(0.0)
{
    // Verify and initialize configuration
    this->samplePeriod = samplePeriod = samplePeriod>1? samplePeriod : 1;
    this->updatePeriod = updatePeriod = updatePeriod>samplePeriod? updatePeriod : samplePeriod;
    updateCount = updatePeriod;
    curShift    = 0.0;

    // Set up FFT
    unsigned int fftSize = samplePeriod * getLength();
    fftIn   = fftwf_alloc_complex(fftSize);
    fftOut  = fftwf_alloc_complex(fftSize);
    fftPlan = fftwf_plan_dft_1d(fftSize, fftIn, fftOut, FFTW_FORWARD, CSDR_FFTW_FLAGS);
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
    int j, i;

    // Count updates
    updateCount--;

    // If sampling input signal...
    if(updateCount<samplePeriod)
    {
        // Copy input signal into the buffer
        j = samplePeriod - updateCount - 1;
        memcpy(&fftIn[size * j], input, size * sizeof(fftIn[0]));

        // If detecting the carrier...
        if(!updateCount)
        {
            // Reset update counter
            updateCount = updatePeriod;

            // Calculate FFT on the input buffer
            fftwf_execute(fftPlan);

            unsigned int fftSize = size * samplePeriod;
            float maxMag = mag2(fftOut[0]);

            // Find the carrier frequency
            for(j=1, i=0 ; j<fftSize ; ++j)
            {
                float mag = mag2(fftOut[j]);
                if(mag>maxMag) { i=j;maxMag=mag; }
            }

            // Take negative shifts into account
            i = i>=fftSize/2? fftSize-i : -i;

            // Update frequency shift, if the change is large enough
            double newShift = (double)i / fftSize;
            if(fabs(newShift-curShift)>0.0001) setRate(curShift = newShift);
        }
    }

    // Shift frequency
    process_fmv(input, output, size);
}
