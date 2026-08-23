/*
Copyright (c) 2026 Marat Fayzullin <luarvique@gmail.com>

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

#include "afcpll.hpp"

#include <cmath>

using namespace Csdr;

AfcPll::AfcPll(float sampleRate, float bandwidthHz, float dampingFactor)
: sampleRate(sampleRate)
{
    // Standard 2nd-order PLL loop filter design (bilinear approximation).
    // See e.g. Gardner, "Phaselock Techniques"; the same formulas are used
    // in liquid-dsp's nco_crcf_pll and similar libraries.
    float bnT = bandwidthHz / sampleRate;
    float theta = bnT / (dampingFactor + 1.0f / (4.0f * dampingFactor));
    float d = 1.0f + 2.0f * dampingFactor * theta + theta * theta;

    alpha = 4.0f * dampingFactor * theta / d;  // proportional gain
    beta  = 4.0f * theta * theta / d;          // integral gain
}

float AfcPll::getFrequency() const
{
    return freq * sampleRate / (2.0f * (float)M_PI);
}

void AfcPll::process(complex<float>* input, complex<float>* output)
{
    const size_t size = getLength();

    for(size_t n = 0; n < size; ++n)
    {
        // Mix the input down by the current NCO phase estimate, i.e.
        // multiply by e^{-j*phase} to cancel a carrier sitting at +phase.
        float c = cosf(phase), s = sinf(phase);
        float yi =  input[n].real() * c + input[n].imag() * s;
        float yq = -input[n].real() * s + input[n].imag() * c;

        output[n] = complex<float>(yi, yq);

        // Running magnitude estimate (simple one-pole smoother), used to
        // gate the phase detector below.
        float mag = sqrtf(yi * yi + yq * yq);
        magAvg += 0.001f * (mag - magAvg);

        // Phase detector: the angle of the corrected sample is the
        // instantaneous phase error. This is amplitude-independent, so
        // AM's own envelope modulation doesn't bias it directly -- except
        // very close to modulation nulls, where magnitude (and therefore
        // phase information) is dominated by noise; we skip the loop
        // update there rather than feed it garbage.
        if(mag > 0.3f * magAvg)
        {
            float error = atan2f(yq, yi);

            // PI loop filter: integral term tracks frequency (persistent
            // drift), proportional term reacts to instantaneous error.
            freq  += beta  * error;
            phase += alpha * error;

            // Smoothed lock-quality indicator, ~1.0 when phase error is
            // small and consistent.
            lockIndicator += 0.001f * (cosf(error) - lockIndicator);
        }

        // Advance the NCO by the current frequency estimate every sample,
        // regardless of whether the detector fired this sample.
        phase += freq;
        while(phase >  (float)M_PI) phase -= 2.0f * (float)M_PI;
        while(phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
    }
}
