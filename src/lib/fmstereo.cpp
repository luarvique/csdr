/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2025-2026 Csongor Dobre <csdobre@outlook.com>
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

#include "fmstereo.hpp"

#include <algorithm>
#include <vector>
#include <complex>
#include <cmath>
#include <iostream>

#define TEST_DIRECTFMINPUT false

using namespace Csdr;

template <typename T>
MonoFractionalDecimator<T>::MonoFractionalDecimator() {

}

template <typename T>
typename MonoFractionalDecimator<T>::Denominator MonoFractionalDecimator<T>::calculateDenominator(double rate, unsigned int num_poly_points, FirFilter<T, float> *filter) {

    Denominator denom;

    denom.num_poly_points = (num_poly_points &~ 1);
    denom.poly_precalc_denomiator.resize(denom.num_poly_points);
    denom.xifirst = (-(denom.num_poly_points / 2) + 1);
    denom.xilast = (denom.num_poly_points / 2);
    denom.coeffs_buf.resize(denom.num_poly_points);
    denom.rate = (rate);

    int id = 0;
    for (int xi = denom.xifirst; xi <= denom.xilast; xi++) {
        denom.poly_precalc_denomiator[id] = 1;
        for(int xj = denom.xifirst; xj <= denom.xilast; xj++) {
            if (xi != xj) denom.poly_precalc_denomiator[id] *= (xi - xj);
        }
        id++;
    }

    denom.where = -denom.xifirst;

    return denom;
}

template <typename T>
MonoFractionalDecimator<T>::~MonoFractionalDecimator() {

}

template <typename T>
bool MonoFractionalDecimator<T>::canProcess(DenominatorImmutable* denomImmutable, DenominatorState* denomState, size_t readerAvailable, size_t writterAvailable, double rate) {
    size_t size = std::min(readerAvailable, (size_t) std::ceil((writterAvailable) / rate));
    size_t filterLen = denomImmutable->filter != nullptr ? denomImmutable->filter->getOverhead() : 0;
    return ceilf(denomState->where) + denomImmutable->num_poly_points + filterLen < size;
}

template <typename T>
typename MonoFractionalDecimator<T>::ProcessState MonoFractionalDecimator<T>::process(DenominatorImmutable* denom, DenominatorState* denomState, const std::vector<T>& input, size_t readerAvailable, size_t writerWriteable, double rate) {

    if (!denom || !denomState) {
        printf("MonoFractionalDecimator::ProcessState::process - Denominators not available!\n");
        return ProcessState();
    }

    ProcessState state;
    state.output = std::vector<T>(writerWriteable);

    int oi = 0;
    int index_high;
    int index = 0;
    size_t size = std::min(readerAvailable, (size_t) std::ceil(writerWriteable / rate));
    size_t filterLen = denom->filter != nullptr ? denom->filter->getOverhead() : 0;

    int ceil_val = static_cast<int>(std::ceil(denomState->where));
    index_high = ceil_val;

    int num_poly = denom->num_poly_points;
    int limit = ceil_val + num_poly + filterLen;

    while (limit < size) {
        index = index_high - 1;

        int id = 0;
        double xwhere = denomState->where - index;
        for (int xi = denom->xifirst; xi <= denom->xilast; xi++) {
            denomState->coeffs_buf[id] = 1;
            for (int xj = denom->xifirst; xj <= denom->xilast; xj++) {
                if (xi != xj) denomState->coeffs_buf[id] *= (xwhere - xj);
            }
            id++;
        }

        T acc = T(0);
        if (denom->filter != nullptr) {
            T* input_ptr = const_cast<T*>(input.data());
            for (int i = 0; i < denom->num_poly_points; i++) {
                SparseView<float> sparse = denom->filter->sparse(input_ptr);
                acc += (denomState->coeffs_buf[i] / denom->poly_precalc_denomiator[i]) * sparse[index + i];
            }
        } else {
            for (int i = 0; i < denom->num_poly_points; i++) {
                acc += (denomState->coeffs_buf[i] / denom->poly_precalc_denomiator[i]) * (input[index + i]);
            }
        }

        state.output[oi++] = acc;
        denomState->where += rate;

        ceil_val = static_cast<int>(std::ceil(denomState->where));
        index_high = ceil_val;

        num_poly = denom->num_poly_points;
        limit = ceil_val + num_poly + filterLen;
    }

    if (oi == 0) {
        state.input_processed = 0;
        state.output_processed = 0;
        return state;
    }

    int input_processed = index + denom->xifirst;
    denomState->where -= input_processed;
    if (input_processed < 0) input_processed = 0;
    if ((size_t) input_processed > readerAvailable) input_processed = (int) readerAvailable;

    state.input_processed = (size_t) input_processed;
    state.output_processed = (size_t) oi;

    return state;
}

template <typename T>
void StereoFractionalDecimator<T>::initializeFilters() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    filter_19k     = new BiquadFilter();
    filter_lp_lr   = new MultistageFilter();
    filter_lp_mono = new MultistageFilter();
    pilot_pll      = new PilotPLL(inputSampleRate, 19000.0, 0.707, 14.0);

    filter_19k->setBandpass2(19000.0, 500.0, inputSampleRate);
    filter_lp_lr  ->setLowpass(15000.0, inputSampleRate, 8);
    filter_lp_mono->setLowpass(15000.0, inputSampleRate, 8);
    deemph_alpha = exp(-(1.0 / inputSampleRate) / deemph_tau);
    deemph_state_L = 0.0;
    deemph_state_R = 0.0;
    deesser_lp_a = std::exp(-2.0 * M_PI * 3500.0 / inputSampleRate);
    deesser_lp_L = 0.0;
    deesser_lp_R = 0.0;

    left_dc_offset = right_dc_offset = 0.0;
    balance_alpha  = 0.0001;

    stereo_factor = 1.5;
    stereo_factor_auto = 1.0;
    stereo_factor_auto_alpha = 1.0 - std::exp(-1.0 / (inputSampleRate * 0.15));
    demod_mode = StereoDemodMode::SdrPll;

    env_sq_smoothed = 0.0;
    env_sq_alpha    = 1.0 - std::exp(-1.0 / (inputSampleRate * 0.003));
    pll_polarity_metric = 1.0;
    pll_polarity_alpha  = 1.0 - std::exp(-1.0 / (inputSampleRate * 0.02));
    pll_polarity_sign = 1;
    pll_polarity_locked = false;
    pll_polarity_lock_samples = 0;

    blend_low_threshold  = 0.0;
    blend_high_threshold = 0.0;
    pilot_blend_smoothed = 0.0;
    pilot_blend_alpha    = 1.0 - std::exp(-1.0 / (inputSampleRate * 0.08));

    quiet_audio_env = 0.0;
    quiet_audio_alpha = 1.0 - std::exp(-1.0 / (inputSampleRate * 0.05));
    quiet_blend_low = 0.0;
    quiet_blend_high = 0.0;
    quiet_blend_min_stereo = 1.0;

    double quad_delay_samples = inputSampleRate / (4.0 * 19000.0);
    quad_delay_int  = static_cast<size_t>(std::floor(quad_delay_samples));
    quad_delay_frac = quad_delay_samples - quad_delay_int;
    pilot_history_idx = 0;
    for (size_t i = 0; i < PILOT_HISTORY_LEN; ++i) pilot_history[i] = 0.0;
    mpx_align_delay_samples = static_cast<size_t>(std::lround(quad_delay_samples)) + 1;
    if (mpx_align_delay_samples >= MPX_ALIGN_MAX_DELAY) {
        mpx_align_delay_samples = MPX_ALIGN_MAX_DELAY - 1;
    }
    mpx_delay_index = 0;
    for (size_t i = 0; i < MPX_ALIGN_MAX_DELAY; ++i) mpx_delay_line[i] = 0.0;

    subcarrier_phase_rad = M_PI / 2.0;
    subcarrier_phase_cos = 0.0;
    subcarrier_phase_sin = 1.0;

    initializedFilters_ = true;
}

template <typename T>
StereoFractionalDecimator<T>::StereoFractionalDecimator(float rateMPX, float rate, float tau, unsigned int num_poly_points, FirFilter<T, float> *filter):
    num_poly_points(num_poly_points &~ 1),
    inputSampleRate(rateMPX), outputSampleRate(rateMPX),
    rate(rate),
    deemph_tau(tau),
    filter(nullptr)
{
    try {
        initializeFilters();

        pilot_strength = 0.0;

        denomImmutable = new typename MonoFractionalDecimator<T>::DenominatorImmutable(this->num_poly_points, rate, filter);
        denomState_left = new typename MonoFractionalDecimator<T>::DenominatorState(-denomImmutable->xifirst, denomImmutable->num_poly_points);
        denomState_right = new typename MonoFractionalDecimator<T>::DenominatorState(-denomImmutable->xifirst, denomImmutable->num_poly_points);

    }
    catch (const std::exception& e) {
        printf("StereoFractionalDecimator::Constructor exception: %s\n", e.what());
        throw;
    }
}

template <typename T>
StereoFractionalDecimator<T>::~StereoFractionalDecimator() {
    delete denomState_right;
    delete denomState_left;
    delete denomImmutable;

    delete filter_19k;
    delete filter_lp_lr;
    delete filter_lp_mono;
    delete pilot_pll;
    deemph_state_L = deemph_state_R = 0.0;
    pilot_strength = 0.0;
    left_dc_offset = right_dc_offset = 0.0;
}

template <typename T>
bool StereoFractionalDecimator<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    size_t writeable_output_frames = this->writer->writeable();
    size_t max_input_for_writer = static_cast<size_t>(std::ceil(writeable_output_frames / rate));
    size_t size = std::min(this->reader->available() + pending_left.size(), max_input_for_writer);
    size_t filterLen = denomImmutable->filter != nullptr ? denomImmutable->filter->getOverhead() : 0;
    return ceilf(denomState_left->where) + denomImmutable->num_poly_points + filterLen < size;
}

template <typename T>
void StereoFractionalDecimator<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    int oi = 0;
    int input_frames_processed = 0;

    size_t available_input_samples = this->reader->available();

    size_t writeable_output_frames = this->writer->writeable();
    size_t size_frames = std::min(available_input_samples, (size_t) std::ceil(writeable_output_frames / rate));
    size_t filterLen = filter != nullptr ? filter->getOverhead() : 0;

    T* input = this->reader->getReadPointer();
    complex<float>* output = this->writer->getWritePointer();

    typename MonoFractionalDecimator<T>::ProcessState state_left;
    typename MonoFractionalDecimator<T>::ProcessState state_right;

    bool leftCanProcess = left_decimator.canProcess(
        denomImmutable,
        denomState_left,
        available_input_samples + pending_left.size(),
        writeable_output_frames,
        rate
    );

    if(leftCanProcess)
    {
        if(!initializedFilters_){
            printf("StereoFractionalDecimator::process - Filters not initialized!\n");
            std::terminate();
        }

        if (pending_left.size() != pending_right.size()) {
            size_t n = std::min(pending_left.size(), pending_right.size());
            pending_left.resize(n);
            pending_right.resize(n);
        }

        size_t pending = pending_left.size();
        pending_left.resize(pending + available_input_samples);
        pending_right.resize(pending + available_input_samples);

        for(size_t i = 0; i < available_input_samples; i++) {
            double mpx_signal = input[i];

            mpx_delay_line[mpx_delay_index] = mpx_signal;
            size_t mpx_read_idx = (mpx_delay_index + MPX_ALIGN_MAX_DELAY - mpx_align_delay_samples) % MPX_ALIGN_MAX_DELAY;
            double mpx_aligned = mpx_delay_line[mpx_read_idx];
            mpx_delay_index = (mpx_delay_index + 1) % MPX_ALIGN_MAX_DELAY;

            double pilot_19k = filter_19k->process(mpx_signal);
            pilot_history[pilot_history_idx] = pilot_19k;
            size_t i_newer = (pilot_history_idx + PILOT_HISTORY_LEN - quad_delay_int    ) % PILOT_HISTORY_LEN;
            size_t i_older = (pilot_history_idx + PILOT_HISTORY_LEN - quad_delay_int - 1) % PILOT_HISTORY_LEN;
            double pilot_q = (1.0 - quad_delay_frac) * pilot_history[i_newer]
                           + quad_delay_frac        * pilot_history[i_older];
            pilot_history_idx = (pilot_history_idx + 1) % PILOT_HISTORY_LEN;

            double env_sq_inst = pilot_19k * pilot_19k + pilot_q * pilot_q;
            env_sq_smoothed += env_sq_alpha * (env_sq_inst - env_sq_smoothed);
            double cos_38k_analytic = 0.0;
            double sin_38k_analytic = 0.0;
            if (env_sq_smoothed > 1e-12) {
                cos_38k_analytic = (pilot_19k * pilot_19k - pilot_q * pilot_q) / env_sq_smoothed;
                sin_38k_analytic = -2.0 * (pilot_19k * pilot_q) / env_sq_smoothed;
            }
            double coherent_38k_analytic = subcarrier_phase_cos * cos_38k_analytic
                                         + subcarrier_phase_sin * sin_38k_analytic;

            double coherent_38k_shifted = 0.0;
            if (demod_mode == StereoDemodMode::SdrPll) {
                pilot_pll->processIQ(pilot_19k, pilot_q, pilot_strength);
                double coherent_38k_pll = std::cos(2.0 * pilot_pll->getPhase() - subcarrier_phase_rad);
                coherent_38k_shifted = coherent_38k_pll;
            } else {
                pilot_strength = std::sqrt(std::max(0.0, env_sq_smoothed));
                coherent_38k_shifted = coherent_38k_analytic;
            }

            double mono = filter_lp_mono->process(mpx_aligned);

            double p = pilot_strength;
            if (p < 0.0) p = 0.0;
            if (p > 1.0) p = 1.0;
            double target_auto = 1.15 + (1.55 - 1.15) * p;
            double mono_abs = std::fabs(mono);
            if (mono_abs < 0.02) {
                target_auto *= 0.85;
            }
            stereo_factor_auto += stereo_factor_auto_alpha * (target_auto - stereo_factor_auto);
            if (stereo_factor_auto < 1.10) stereo_factor_auto = 1.10;
            if (stereo_factor_auto > 1.70) stereo_factor_auto = 1.70;

            const double effective_stereo_gain = stereo_factor * stereo_factor_auto;
            double lr   = filter_lp_lr  ->process(mpx_aligned * coherent_38k_shifted * effective_stereo_gain);
            if (blend_high_threshold > blend_low_threshold) {
                const double hard_mono_pilot = 0.002;
                if (pilot_strength < hard_mono_pilot) {
                    pilot_blend_smoothed = 0.0;
                    lr = 0.0;
                } else {
                    pilot_blend_smoothed += pilot_blend_alpha * (pilot_strength - pilot_blend_smoothed);
                    double blend = (pilot_blend_smoothed - blend_low_threshold)
                                 / (blend_high_threshold - blend_low_threshold);
                    if (blend < 0.0) blend = 0.0;
                    if (blend > 1.0) blend = 1.0;
                    lr *= blend;
                }
            }

            quiet_audio_env += quiet_audio_alpha * (std::fabs(mono) - quiet_audio_env);
            if (quiet_blend_high > quiet_blend_low) {
                double q = (quiet_audio_env - quiet_blend_low)
                         / (quiet_blend_high - quiet_blend_low);
                if (q < 0.0) q = 0.0;
                if (q > 1.0) q = 1.0;
                double quiet_blend = quiet_blend_min_stereo + (1.0 - quiet_blend_min_stereo) * q;
                lr *= quiet_blend;
            }

            double left_raw  = mono - lr;
            double right_raw = mono + lr;

            left_dc_offset  += balance_alpha * (left_raw  - left_dc_offset);
            right_dc_offset += balance_alpha * (right_raw - right_dc_offset);
            left_raw  -= left_dc_offset;
            right_raw -= right_dc_offset;

            double left_deemph  = deemphasisFilter(left_raw,  true);
            double right_deemph = deemphasisFilter(right_raw, false);

            double left_out  = left_deemph;
            double right_out = right_deemph;
            const double output_gain = 0.72;
            auto peak_guard = [](double x) -> double {
                if (x > 0.98) return 0.98;
                if (x < -0.98) return -0.98;
                return x;
            };
            pending_left[pending + i]  = peak_guard(output_gain * left_out);
            pending_right[pending + i] = peak_guard(output_gain * right_out);
        }

#if !TEST_DIRECTFMINPUT

        denomState_right->where = denomState_left->where;

        state_left  = left_decimator.process(denomImmutable,  denomState_left,  pending_left,  pending_left.size(),  writeable_output_frames, rate);
        state_right = right_decimator.process(denomImmutable, denomState_right, pending_right, pending_right.size(), writeable_output_frames, rate);

        size_t synced_input_consumed = std::min(state_left.input_processed, state_right.input_processed);
        size_t synced_output_frames  = std::min(state_left.output_processed, state_right.output_processed);
        if (synced_input_consumed > pending_left.size()) synced_input_consumed = pending_left.size();

        if (state_left.input_processed > synced_input_consumed)
            denomState_left->where += (double) (state_left.input_processed - synced_input_consumed);
        if (state_right.input_processed > synced_input_consumed)
            denomState_right->where += (double) (state_right.input_processed - synced_input_consumed);
        if (state_left.output_processed > synced_output_frames)
            denomState_left->where -= rate * (double) (state_left.output_processed - synced_output_frames);
        if (state_right.output_processed > synced_output_frames)
            denomState_right->where -= rate * (double) (state_right.output_processed - synced_output_frames);
        denomState_right->where = denomState_left->where;

        for (size_t i = 0; i < synced_output_frames; i++) {
            output[i] = complex<float>(state_left.output[i], state_right.output[i]);
        }

        pending_left.erase(pending_left.begin(), pending_left.begin() + synced_input_consumed);
        pending_right.erase(pending_right.begin(), pending_right.begin() + synced_input_consumed);

        this->reader->advance(available_input_samples);
        this->writer->advance(synced_output_frames);
#else
        size_t output_frames = std::min(pending_left.size(), writeable_output_frames);

        for (size_t i = 0; i < output_frames; i++) {
            output[i] = complex<float>(pending_left[i], pending_right[i]);
        }

        pending_left.erase(pending_left.begin(), pending_left.begin() + output_frames);
        pending_right.erase(pending_right.begin(), pending_right.begin() + output_frames);

        this->reader->advance(available_input_samples);
        this->writer->advance(output_frames);
#endif
    }
}

namespace Csdr {
    template class MonoFractionalDecimator<float>;
    template class StereoFractionalDecimator<float>;
}