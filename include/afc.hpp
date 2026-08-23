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

#pragma once

#include "module.hpp"
#include "complex.hpp"

namespace Csdr {

    // Costas/PLL-style continuous carrier tracker: an alternative to the
    // FFT-based Afc for phase-sensitive consumers such as synchronous AM
    // (SAm), where a small residual frequency error produces an audible,
    // periodic amplitude ("volume") oscillation in RealPart()-based
    // detection.
    //
    // Unlike Afc, this corrects both frequency AND phase, every single
    // sample, via a standard second-order PI loop, rather than measuring
    // frequency periodically from a block FFT and leaving phase
    // uncontrolled between updates.
    //
    // Trade-off: limited pull-in range. It only locks onto a carrier that
    // starts out within roughly its loop bandwidth of true center, so it's
    // best used for *fine* tracking after a coarse acquisition step (e.g.
    // one pass of the existing FFT-based Afc, or accurate-enough manual
    // tuning) has already gotten close. It is not a search tool.
    class Afc: public FixedLengthModule<complex<float>, complex<float>> {
        public:
            // sampleRate:     sample rate of the input signal, Hz
            // bandwidthHz:    loop (noise) bandwidth, Hz - controls the
            //                 tracking-speed vs. noise-rejection tradeoff
            //                 and, roughly, the pull-in range
            // dampingFactor:  loop damping factor; 0.707 (critically
            //                 damped) is the standard default
            explicit Afc(float sampleRate, float bandwidthHz = 100.0f, float dampingFactor = 0.3f);

            // Current frequency estimate, in Hz (positive = carrier above center)
            float getFrequency() const;

            // Smoothed lock-quality indicator in [0,1]; approaches 1.0 when
            // phase error is small and consistent (i.e. actually locked).
            // Useful for UI/telemetry or for gating a squelch/mute.
            float getLockQuality() const { return lockIndicator; }

        protected:
            void process(complex<float>* input, complex<float>* output) override;
            size_t getLength() override { return 1024; }

        private:
            float sampleRate;

            // Loop filter gains, derived from bandwidthHz / dampingFactor
            // at construction time (standard 2nd-order PLL design).
            float alpha;   // proportional gain
            float beta;    // integral gain

            // NCO / loop state (persists across process() calls, i.e.
            // across block boundaries -- this is what gives phase
            // continuity, analogous to Afc's curShift but updated every
            // sample instead of every updatePeriod blocks).
            float phase = 0.0f;   // current NCO phase, radians, wrapped to [-pi, pi]
            float freq  = 0.0f;   // current NCO frequency, radians/sample

            // Running magnitude estimate, used to gate the phase detector
            // during deep AM modulation nulls where phase information is
            // unreliable (dominated by noise rather than carrier phase).
            float magAvg = 1e-6f;

            // Smoothed lock-quality indicator
            float lockIndicator = 0.0f;
    };
}
