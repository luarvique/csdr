/*
This file is part of libcsdr.

	Copyright (c) Andras Retzler, HA7ILM <randras@sdr.hu>
	Copyright (c) Warren Pratt, NR0V <warren@wpratt.com>
    Copyright (c) Jakob Ketterl, DD5JFK <jakob.ketterl@gmx.de>
	Copyright 2006,2010,2012 Free Software Foundation, Inc.

    libcsdr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libcsdr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libcsdr.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "agc.hpp"
#include "complex.hpp"

#include <cmath>
#include <climits>
#include <algorithm>

using namespace Csdr;

template <typename T>
void Agc<T>::process(T* input, T* output, size_t work_size) {
    float input_abs, error, dgain;
    size_t j;

    // Find the initial max value for the envelope
    for (j = 0; (j < ahead_time) && (j < work_size) ; j++) {
        input_abs = this->abs(input[j]);
        max_abs = max_abs < input_abs? input_abs : max_abs * 0.99;
    }

    for (size_t i = 0; i < work_size; i++) {
        // The error is the difference between the required gain at
        // the actual sample, and the previous gain value.
        // We actually use an envelope detector.
        //input_abs = this->abs(input[i]);
        error = (max_abs * gain) / reference;

        // An AGC is something nonlinear that's easier to implement in
        // software:
        // * If the amplitude decreases, we increase the gain by
        //   minimizing the gain error by attack_rate.
        // * We also have a decay_rate that comes into consideration
        //   when the amplitude increases.
        // * The higher these rates are, the faster is the response of
        //   the AGC to amplitude changes.
        // * However, attack_rate should be higher than the decay_rate
        //   as we want to avoid clipping signals that had a sudden
        //   increase in their amplitude.
        // * It's also important to note that this algorithm has an
        //   exponential gain ramp.

        if (error > 1.0) {
            // INCREASE IN SIGNAL LEVEL
            // If the signal level increases, we decrease the gain
            // quite fast.
            dgain = 1.0 - attack_rate;
            // Before starting to increase the gain next time, we
            // will be waiting until hang_time for sure.
            hang_counter = hang_time;
        } else if (hang_counter > 0) {
            // Before starting to increase the gain, we will be waiting
            // until hang_time.
            hang_counter--;
            // Until then, AGC is inactive and gain doesn't change.
            dgain = 1.0;
        } else if (error < 1.0) {
            // DECREASE IN SIGNAL LEVEL
            // If the signal level decreases, we increase the gain
            // quite slowly.
            dgain = 1.0 + decay_rate;
        } else {
            // NO CHANGE IN SIGNAL LEVEL
            dgain = 1.0;
        }

        // Modify gain
        gain = gain * dgain;

        // Clamp gain to the [0 ; max_gain] range
        if (gain > max_gain) gain = max_gain;
        if (gain < 0.0) gain = 0.0;

        // Scale the sample
        output[i] = scale(input[i]);

        // Move the envelope
        input_abs = j < work_size? this->abs(input[j++]) : 0.0;
        max_abs = max_abs < input_abs? input_abs : max_abs * 0.99;
    }
}

template <>
float Agc<short>::abs(short in) {
    return std::fabs((float) in) / SHRT_MAX;
}

template <>
bool Agc<short>::isZero(short in) {
    return in == 0;
}

template <>
short Agc<short>::scale(short in) {
    float val = gain * in;
    if (val >= SHRT_MAX) return SHRT_MAX;
    if (val <= SHRT_MIN) return SHRT_MIN;
    return (short) val;
}

template <>
float Agc<float>::abs(float in) {
    return std::fabs(in);
}

template <>
bool Agc<float>::isZero(float in) {
    return in == 0.0f;
}

template <>
float Agc<float>::scale(float in) {
    float val = in * gain;
    if (val > 1.0f) return 1.0f;
    if (val < -1.0f) return -1.0f;
    return val;
}

template <>
float Agc<complex<float>>::abs(complex<float> in) {
    return std::abs(in);
}

template <>
bool Agc<complex<float>>::isZero(complex<float> in) {
    return in == complex<float>(0, 0);
}

template <>
complex<float> Agc<complex<float>>::scale(complex<float> in) {
    complex<float> val = in * gain;
    if (val.i() > 1.0f) val.i(1.0f);
    if (val.q() > 1.0f) val.q(1.0f);
    if (val.i() < -1.0f) val.i(-1.0f);
    if (val.q() < -1.0f) val.q(-1.0f);
    return val;
}

template <typename T>
void Agc<T>::setReference(float reference) {
    this->reference = reference;
}

template <typename T>
void Agc<T>::setAttack(float attack_rate) {
    this->attack_rate = attack_rate;
}

template <typename T>
void Agc<T>::setDecay(float decay_rate) {
    this->decay_rate = decay_rate;
}

template <typename T>
void Agc<T>::setMaxGain(float max_gain) {
    this->max_gain = max_gain;
}

template <typename T>
void Agc<T>::setInitialGain(float initial_gain) {
    gain = initial_gain;
}

template <typename T>
void Agc<T>::setHangTime(unsigned long int hang_time) {
    this->hang_time = hang_time;
}

namespace Csdr {
    template class Agc<short>;
    template class Agc<float>;
    template class Agc<complex<float>>;
}
