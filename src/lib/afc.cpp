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

using namespace Csdr;

Afc::Afc(unsigned int sampleRate, unsigned int bandwidth, unsigned int syncWidth):
    ShiftAddfast(0.0),
    sampleRate(sampleRate),
    bandwidth(bandwidth<sampleRate/2? bandwidth : sampleRate/2),
    syncWidth(syncWidth<bandwidth/2?  syncWidth : bandwidth/2),
    buckets(3 * sampleRate / syncWidth),
    deltaF(0)
{
    // Goertzel algorithm coefficients
    omega = round((double)buckets * syncWidth / 2 / sampleRate);
    omega = omega * 2.0 * M_PI / buckets;
    coeff = 2.0 * cos(omega);
}

void Afc::process(complex<float>* input, complex<float>* output)
{
    unsigned int size = getLength();
    double q0, q1, q2;
    unsigned int j;

    // Shift frequency
    process_fmv(input, output, size);

    // Run Goertzel algorithm in three buckets
    for(j=0, q1=q2=0.0 ; j<size ; ++j)
    {
        q0 = q1 * coeff - q2 + output[j];
        q2 = q1;
        q1 = q0;
    }

    // We only need the real part
    double magnitude = sqrt(q1*q1 + q2*q2 - q1*q2*coeff);

    // Adjust frequency shift based on left/right magnitudes
    if(std::abs(q2-q1)>1.0)
    {
        // Need signed integer here
        int sw = 10 * syncWidth;

        if(q2-q1>0.0)
        {
            deltaF += 5;
            deltaF  = deltaF<=sw? deltaF : sw;
        }
        else
        {
            deltaF -= 5;
            deltaF  = deltaF>=-sw? deltaF : -sw;
        }

fprintf(stderr, "%f %f => %dHz\n", q1, q2, deltaF);

        // Update shifter rate
        setRate((double)deltaF / sampleRate);
    }
}
