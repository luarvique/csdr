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
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace Csdr;

#if defined __arm__ || __aarch64__
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_ESTIMATE)
#else
#define CSDR_FFTW_FLAGS (FFTW_DESTROY_INPUT | FFTW_MEASURE)
#endif

// Hann window function
static inline float hann(unsigned int x, unsigned int size)
{
    return 0.5f - 0.5f * std::cos((2.0 * M_PI * x) / size);
}

// Squared magnitude of a complex value
static double mag2(const fftwf_complex &v)
{
    return v[0] * v[0] + v[1] * v[1];
}

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

    // Precompute input window
    inputWindow = new float[fftSize];
    for(size_t i=0; i < fftSize; i++)
        inputWindow[i] = hann(i, fftSize);
}

Afc::~Afc()
{
    // Destroy FFT
    fftwf_destroy_plan(fftPlan);
    fftwf_free(fftIn);
    fftwf_free(fftOut);
    delete [] inputWindow;
}

void Afc::process(complex<float>* input, complex<float>* output)
{
    unsigned int size = getLength();
    unsigned int j, i;

    // Count updates
    updateCount--;

    // If sampling input signal...
    if(updateCount < samplePeriod)
    {
        // Copy input signal into the buffer
        i = (samplePeriod - updateCount - 1) * size;
        for(j=0 ; j<size ; ++j, ++i)
        {
            std::complex<float> v = input[j] * inputWindow[i];
            fftIn[i][0] = v.real();
            fftIn[i][1] = v.imag();
        }

        // If detecting the carrier...
        if(!updateCount)
        {
            // Reset update counter
            updateCount = updatePeriod;

            // Calculate FFT on the input buffer
            fftwf_execute(fftPlan);

            unsigned int fftSize = size * samplePeriod;
            double curMag = mag2(fftOut[fftSize-1]) + mag2(fftOut[0]) + mag2(fftOut[1]);
            double maxMag = curMag;

            // Find the carrier frequency, searching by the combined
            // magnitude of three adjacent bins
            for(j=1, i=0 ; j<fftSize ; ++j)
            {
                curMag += mag2(fftOut[j<fftSize-1? j+1 : 0]);
                curMag -= mag2(fftOut[j>1? j-2 : fftSize-1]);
                if(curMag > maxMag) { i=j; maxMag=curMag; }
            }

            // Refine the peak location with parabolic interpolation
            // over the peak bin and its two immediate neighbors.
            double magL  = mag2(fftOut[i>0? i-1 : fftSize-1]);
            double magR  = mag2(fftOut[i<fftSize-1? i+1 : 0]);
            double delta = magL - 2.0 * mag2(fftOut[i]) + magR;
            delta = delta!=0.0? 0.5 * (magL - magR) / delta : 0.0;

            // Clamp to +/-0.5 bin, take negative shifts into account
            double shift = (double)i + std::max(std::min(delta, 0.5), -0.5);
            if(shift < 0.0)          shift += fftSize;
            if(shift >= fftSize)     shift -= fftSize;
            if(shift >= fftSize/2.0) shift -= fftSize;

            // Update frequency shift, if the change is large enough
            shift /= fftSize;
            if(std::abs(shift-curShift) >= 0.0001) setRate(curShift = shift);
        }
    }

    // Shift frequency
    process_fmv(input, output, size);
}
