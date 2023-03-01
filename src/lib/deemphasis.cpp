/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2021 Jakob Ketterl <jakob.ketterl@gmx.de>
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

#include "deemphasis.hpp"
#include "predefined.h"

#include <cmath>

using namespace Csdr;

WfmDeemphasis::WfmDeemphasis(unsigned int sampleRate, float tau): dt(1.0f / sampleRate), alpha(dt / tau + dt) {}

void WfmDeemphasis::process(float *input, float *output, size_t size) {
    /*
        typical time constant (tau) values:
        WFM transmission in USA: 75 us -> tau = 75e-6
        WFM transmission in EU:  50 us -> tau = 50e-6
        More info at: http://www.cliftonlaboratories.com/fm_receivers_and_de-emphasis.htm
        Simulate in octave: tau=75e-6; dt=1/48000; alpha = dt/(tau+dt); freqz([alpha],[1 -(1-alpha)])
    */
    if (std::isnan(last_output)) last_output = 0.0;
    for (int i = 0; i < size; i++) {
        output[i] = last_output = alpha * input[i] + (1 - alpha) * last_output; //this is the simplest IIR LPF
    }
}

NfmDeephasis::NfmDeephasis(unsigned int sampleRate): FilterModule<float>(getFilter(sampleRate)) {}

FirFilter<float, float>* NfmDeephasis::getFilter(unsigned int sampleRate) {
    switch (sampleRate) {
        // we only cover selected sample rates. see predefined.h for details.
        case 8000:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_8000, 79);
        case 11025:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_11025, 79);
        case 12000:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_12000, 79);
        case 24000:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_24000, 199);
        case 44100:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_44100, 199);
        case 48000:
            return new FirFilter<float, float>(deemphasis_nfm_predefined_fir_48000, 199);
        default:
            throw std::runtime_error("invalid sample rate");
    }
}
