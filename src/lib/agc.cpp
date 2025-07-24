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
    float max_gain = this->max_gain;
    float input_abs;
    float error, dgain;

    float xk, vk, rk;
    float dt = 0.5;
    float beta = 0.005;

    //Determine magnitude
//    max_gain = 0.0;
//    for (int i = 0; i < work_size; i++)
//        max_gain = std::max(max_gain, (float)std::norm(input[i]));
    //Compute gain limit that avoids clipping
//    max_gain = max_gain > 0.0? maxMagnitude() / fsqrt(max_gain) : 1.0;
//    max_gain = std::min(max_gain, this->max_gain);

    for (int i = 0; i < work_size; i++) {
        //We skip samples containing 0, as the gain would be infinity for those to keep up with the reference.
        if (!isZero(input[i])) {
            //The error is the difference between the required gain at the actual sample, and the previous gain value.
            //We actually use an envelope detector.
            input_abs = this->abs(input[i]);
            error = (input_abs * gain) / reference;

            //An AGC is something nonlinear that's easier to implement in software:
            //if the amplitude decreases, we increase the gain by minimizing the gain error by attack_rate.
            //We also have a decay_rate that comes into consideration when the amplitude increases.
            //The higher these rates are, the faster is the response of the AGC to amplitude changes.
            //However, attack_rate should be higher than the decay_rate as we want to avoid clipping signals.
            //that had a sudden increase in their amplitude.
            //It's also important to note that this algorithm has an exponential gain ramp.

            if (error > 1) {
                //INCREASE IN SIGNAL LEVEL
                //If the signal level increases, we decrease the gain quite fast.
                dgain = 1 - attack_rate;
                //Before starting to increase the gain next time, we will be waiting until hang_time for sure.
                hang_counter = hang_time;
            } else {
                //DECREASE IN SIGNAL LEVEL
                if (hang_counter > 0) {
                    //Before starting to increase the gain, we will be waiting until hang_time.
                    hang_counter--;
                    dgain = 1; //..until then, AGC is inactive and gain doesn't change.
                } else {
                    dgain = 1 + decay_rate; //If the signal level decreases, we increase the gain quite slowly.
                }
            }
            gain = gain * dgain;
        }

        // alpha beta filter
        xk = this->xk + (this->vk * dt);
        vk = this->vk;

        rk = gain - xk;

        xk += gain_filter_alpha * rk;
        vk += (beta * rk) / dt;

        this->xk = xk;
        this->vk = vk;

        gain = this->xk;

        // clamp gain to max_gain and 0
        if (gain > max_gain) gain = max_gain;
        if (gain < 0) gain = 0;

        // actual sample scaling
        output[i] = scale(input[i]);
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
float Agc<short>::maxMagnitude() {
    return SHRT_MAX - 1;
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
float Agc<float>::maxMagnitude() {
    return 1.0f;
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

template <>
float Agc<complex<float>>::maxMagnitude() {
    return M_SQRT2;
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
