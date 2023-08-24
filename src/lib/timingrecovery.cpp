/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2014, Andras Retzler <randras@sdr.hu>
Copyright (c) 2019-2023 Jakob Ketterl <jakob.ketterl@gmx.de>
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

#include "timingrecovery.hpp"

using namespace Csdr;

template <typename T>
TimingRecovery<T>::TimingRecovery(unsigned int decimation, float loop_gain, float max_error):
    decimation(decimation),
    loop_gain(loop_gain),
    max_error(max_error)
{}

template <typename T>
bool TimingRecovery<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return this->reader->available() > (decimation / 2) * 3 && this->writer->writeable() > 0;
}

template <typename T>
void TimingRecovery<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    //We always assume that the input starts at center of the first symbol cross before the first symbol.
    //Last time we consumed that much from the input samples that it is there.
    unsigned int num_samples_bit = decimation;
    unsigned int num_samples_halfbit = decimation / 2;
    unsigned int num_samples_quarterbit = decimation / 4;

    if (correction_offset <= 0.9 * -num_samples_quarterbit || correction_offset >= 0.9 * num_samples_quarterbit) {
        correction_offset = 0;
    }

    //should check if the sign of the correction_offset (or disabling it) has an effect on the EVM.
    //it is also a possibility to disable multiplying with the magnitude
    float error = getError();

    if (error > max_error) error = max_error;
    if (error < -max_error) error = -max_error;

    correction_offset = (int) num_samples_halfbit * getErrorSign() * error * loop_gain;

    this->reader->advance(num_samples_bit + correction_offset);
}

template <>
float TimingRecovery<float>::calculateError(int el_point_right_index, int el_point_left_index, int el_point_mid_index) {
    float* input = reader->getReadPointer();
    return (input[el_point_right_index] - input[el_point_left_index]) * input[el_point_mid_index];
}

template <>
float TimingRecovery<complex<float>>::calculateError(int el_point_right_index, int el_point_left_index, int el_point_mid_index) {
    complex<float>* input = reader->getReadPointer();
    return (
        (input[el_point_right_index].i() - input[el_point_left_index].i()) * input[el_point_mid_index].i() + \
        (input[el_point_right_index].q() - input[el_point_left_index].q()) * input[el_point_mid_index].q()
    ) / 2;
}

template <typename T>
float GardnerTimingRecovery<T>::getError() {
    T* input = this->reader->getReadPointer();
    //maximum effect point is at current_bitstart_index
    int num_samples_halfbit = this->decimation / 2;

    int el_point_right_index  = num_samples_halfbit * 3;
    int el_point_left_index   = num_samples_halfbit * 1;
    int el_point_mid_index    = num_samples_halfbit * 2;
    *(this->writer->getWritePointer()) = input[el_point_left_index];
    this->writer->advance(1);

    return this->calculateError(el_point_right_index, el_point_left_index, el_point_mid_index);
}

template <typename T>
float EarlyLateTimingRecovery<T>::getError() {
    T* input = this->reader->getReadPointer();
    int num_samples_bit = this->decimation;
    int num_samples_halfbit = this->decimation / 2;
    int num_samples_earlylate_wing = num_samples_bit * earlylate_ratio;

    int el_point_right_index  = num_samples_earlylate_wing * 3;
    int el_point_left_index   = num_samples_earlylate_wing - this->correction_offset;
    int el_point_mid_index    = num_samples_halfbit;
    *(this->writer->getWritePointer()) = input[el_point_mid_index];
    this->writer->advance(1);

    return this->calculateError(el_point_right_index, el_point_left_index, el_point_mid_index);
}

namespace Csdr {
    template class TimingRecovery<float>;
    template class TimingRecovery<complex<float>>;
    template class GardnerTimingRecovery<float>;
    template class GardnerTimingRecovery<complex<float>>;
    template class EarlyLateTimingRecovery<float>;
    template class EarlyLateTimingRecovery<complex<float>>;
}