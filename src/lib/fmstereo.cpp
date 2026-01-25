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
typename MonoFractionalDecimator<T>::Denominator MonoFractionalDecimator<T>::calculateDenominator(float rate, unsigned int num_poly_points, FirFilter<T, float> *filter) {

    Denominator denom;

    denom.num_poly_points = (num_poly_points &~ 1);
    denom.poly_precalc_denomiator.resize(denom.num_poly_points);
    denom.xifirst = (-(denom.num_poly_points / 2) + 1);
    denom.xilast = (denom.num_poly_points / 2);
    denom.coeffs_buf.resize(denom.num_poly_points);
    denom.rate = (rate);

    int id = 0; //index in poly_precalc_denomiator
    for (int xi = denom.xifirst; xi <= denom.xilast; xi++) {
        denom.poly_precalc_denomiator[id] = 1;
        for(int xj = denom.xifirst; xj <= denom.xilast; xj++) {
            //poly_precalc_denomiator could be integer as well. But that would later add a necessary conversion.
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
bool MonoFractionalDecimator<T>::canProcess(DenominatorImmutable* denomImmutable, DenominatorState* denomState, size_t readerAvailable, size_t writterAvailable, float rate) {   
    size_t size = std::min(readerAvailable, (size_t) ceilf((writterAvailable) / rate));
    size_t filterLen = denomImmutable->filter != nullptr ? denomImmutable->filter->getOverhead() : 0;
    return ceilf(denomState->where) + denomImmutable->num_poly_points + filterLen < size;
}

template <typename T>
typename MonoFractionalDecimator<T>::ProcessState MonoFractionalDecimator<T>::process(DenominatorImmutable* denom, DenominatorState* denomState, std::vector<T> input, size_t readerAvailable, size_t writerWriteable, float rate) {
    
    if (!denom || !denomState) {
        printf("MonoFractionalDecimator::ProcessState::process - Denominators not available!\n");
        return ProcessState(); // Return empty state
    }
    
    ProcessState state;
    state.output = std::vector<T>(writerWriteable);

    int oi = 0; //output index
    int index_high, index;
    size_t size = std::min(readerAvailable, (size_t) ceilf(writerWriteable / rate));
    size_t filterLen = denom->filter != nullptr ? denom->filter->getOverhead() : 0;

    int ceil_val = static_cast<int>(ceilf(denomState->where));
    index_high = ceil_val;

    int num_poly = denom->num_poly_points;
    int limit = ceil_val + num_poly + filterLen;

    //we optimize to calculate ceilf(where) only once every iteration, so we do it here:
    while (limit < size) {
        // num_poly_points above is theoretically more than we could have here, but this makes the spectrum look good
        index = index_high - 1;

        int id = 0;
        float xwhere = denomState->where - index;
        for (int xi = denom->xifirst; xi <= denom->xilast; xi++) {
            denomState->coeffs_buf[id] = 1;
            for (int xj = denom->xifirst; xj <= denom->xilast; xj++) {
                if (xi != xj) denomState->coeffs_buf[id] *= (xwhere - xj);
            }
            id++;
        }
        
        T acc = T(0);
        if (denom->filter != nullptr) {
            for (int i = 0; i < denom->num_poly_points; i++) {
                SparseView<float> sparse = denom->filter->sparse(input.data());
                acc += (denomState->coeffs_buf[i] / denom->poly_precalc_denomiator[i]) * sparse[index + i];
            }
        } else {
            for (int i = 0; i < denom->num_poly_points; i++) {
                acc += (denomState->coeffs_buf[i] / denom->poly_precalc_denomiator[i]) * (input[index + i]);
            }
        }

        state.output[oi++] = acc;
        denomState->where += rate;

        ceil_val = static_cast<int>(ceilf(denomState->where));
        index_high = ceil_val;

        num_poly = denom->num_poly_points;
        limit = ceil_val + num_poly + filterLen;
    }

    int input_processed = 0;

    input_processed = index + denom->xifirst;
    denomState->where -= input_processed;

    state.input_processed = input_processed;
    state.output_processed = oi;

    return state;
}

template <typename T>
void StereoFractionalDecimator<T>::initializeFilters() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    filter_19k = new BiquadFilter();
    notch_19k = new NotchFilter();
    notch_38k = new NotchFilter();
    filter_hp = new BiquadFilter();
    filter_lp_lr = new MultistageFilter();
    filter_lp_mono = new MultistageFilter();
    
    pilot_pll = new PilotPLL(inputSampleRate);

    // 19kHz pilot tone bandpass filter (Q = 1000)
    filter_19k->setBandpass2(19000.0, 1000.0, inputSampleRate);
    
    // Notch filters to remove pilot and carrier from audio (0 = 50)
    notch_19k->setNotch(19000.0, 50.0, inputSampleRate);  // Remove 19kHz pilot from mono
    notch_38k->setNotch(38000.0, 50.0, inputSampleRate);  // Remove 38kHz carrier from L-R

    // High-pass filter for DC removal (~10Hz cutoff)
    filter_hp->setHighpass(10.0, inputSampleRate);

    // Multi-stage low-pass filters for audio (15kHz cutoff, 8th order)
    filter_lp_lr->setLowpass(15000.0, inputSampleRate, 8);
    filter_lp_mono->setLowpass(15000.0, inputSampleRate, 8);

    // Deemphasis time constant (50 microseconds)
    deemph_alpha = exp(-(1.0 / inputSampleRate) / deemph_tau);
    deemph_state_L = 0.0;
    deemph_state_R = 0.0;

    // ==================================================================================
    carrier_leak_i = 0.0;
    leak_alpha = 0.001; // 0.001

    // Channel balance correction
    left_dc_offset = right_dc_offset = 0.0;   // DC offset tracking
    left_gain_correction = right_gain_correction = 1.0;  // Gain imbalance correction 1.0
    balance_alpha = 0.0001;                     // Balance adaptation rate, 0.0001
    left_energy = right_energy = 0.0;        // Channel energy tracking

    signal_level = 0.0;
    noise_floor = 0.01;
    gate_threshold = 0.001;
    gate_alpha = 0.0001;
    signal_present = false;
    lr_gain_correction = 2.5;
    lr_gain_alpha = 0.00001;
    mono_rms = lr_rms = 0.0;

    // Direct crosstalk cancellation
    left_to_right_leak = right_to_left_leak = 0.0;  // Measured crosstalk coefficients
    crosstalk_alpha = 0.0001;                   // Crosstalk adaptation rate
    left_reference = right_reference = 0.0;   // Reference signals for crosstalk measurement
    
    // ==================================================================================

    delay_index = 0;

    // Setup delay compensation for phase alignment
    // The pilot processing chain introduces delay, compensate mono path
    delay_samples = 16;  // Empirically determined for phase alignment
    if (delay_samples > MAX_DELAY_SAMPLES) {
        delay_samples = MAX_DELAY_SAMPLES;
    }
    
    // Initialize delay lines to zero
    for (size_t i = 0; i < MAX_DELAY_SAMPLES; ++i) {
        lr_delay_line[i] = 0.0;
        mono_delay_line[i] = 0.0;
    }

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

        delay_enabled = true;
        pilot_strength = 0.0;
        stereo_threshold = 0.005;

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
    delete notch_19k;
    delete notch_38k;
    delete filter_hp;
    delete filter_lp_lr;
    delete filter_lp_mono;
    delete pilot_pll;

    deemph_state_L = deemph_state_R = 0.0;
    delay_index = 0;

    carrier_leak_i = 0.0;
    pilot_strength = 0.0;
    left_dc_offset = right_dc_offset = 0.0;
    left_gain_correction = right_gain_correction = 1.0;
    left_energy = right_energy = 0.0;

    // Clear delay lines
    for (size_t i = 0; i < MAX_DELAY_SAMPLES; ++i) {
        lr_delay_line[i] = 0.0;
        mono_delay_line[i] = 0.0;
    }
}

template <typename T>
bool StereoFractionalDecimator<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    
    size_t size = std::min(this->reader->available(), (size_t) ceilf((this->writer->writeable() / 2) / rate));
    size_t filterLen = denomImmutable->filter != nullptr ? denomImmutable->filter->getOverhead() : 0;
    return ceilf(denomState_left->where) + denomImmutable->num_poly_points + filterLen < size;
}

template <typename T>
void StereoFractionalDecimator<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    
    int oi = 0;
    int input_frames_processed = 0;
    
    size_t available_samples = this->reader->available();
    size_t available_stereo_frames = available_samples / 2;
    size_t writeable_samples = this->writer->writeable();
    size_t writeable_stereo_frames = writeable_samples / 2;
    size_t size_frames = std::min(available_stereo_frames, (size_t) std::ceil(writeable_stereo_frames / rate));
    size_t filterLen = filter != nullptr ? filter->getOverhead() : 0;
 
    T* input = this->reader->getReadPointer();
    T* output = this->writer->getWritePointer();

    typename MonoFractionalDecimator<T>::ProcessState state_left;
    typename MonoFractionalDecimator<T>::ProcessState state_right;

    bool leftCanProcess = left_decimator.canProcess(denomImmutable, denomState_left, available_stereo_frames, writeable_stereo_frames, rate);

    if(leftCanProcess)
    {
        std::vector<double> input_fm(available_samples);
        
        for(size_t i = 0; i < available_samples; i++) {
            input_fm[i] = input[i];
        }

        std::vector<T> input_left(available_samples);
        std::vector<T> input_right(available_samples);
        size_t i_left = 0;
        size_t i_right = 0;

        if(!initializedFilters_){
            printf("StereoFractionalDecimator::process - Filters not initialized!\n");
            std::terminate();
        }

        double stereo_factor = 0.5;
        std::vector<double> raw_left_output(available_samples);
        std::vector<double> raw_right_output(available_samples);
        std::vector<double> test_lp15k_output(available_samples);
        std::vector<double> test_lmr_filtered_output(available_samples);

        for(size_t i = 0; i < available_samples; i++) {

            // FM demodulation to get MPX signal
            double mpx_signal = input_fm[i];

            // Extract 19kHz pilot tone with wider bandwidth for PLL
            double pilot_19k = filter_19k->process(mpx_signal);

            // Generate phase-coherent 38kHz carrier using PLL
            double coherent_38k = pilot_pll->process(pilot_19k, pilot_strength);

            double lr_signal = mpx_signal * coherent_38k;

            // Extract L+R signal (mono) with phase-matched filtering
            double mono_raw = filter_lp_mono->process(mpx_signal);
            double mono_clean = notch_19k->process(mono_raw);

            // Apply stereo factor only if pilot is present
            double stereo_blend = (pilot_strength > stereo_threshold) ? stereo_factor : 0.0;
            lr_signal *= stereo_blend;

            // Remove 38kHz carrier residue from L-R signal
            lr_signal = notch_38k->process(lr_signal);

            double lr_filtered = filter_lp_lr->process(lr_signal);
            
            // Track L-R RMS
            lr_rms += lr_gain_alpha * (lr_filtered * lr_filtered - lr_rms);

            // Adaptive L-R gain correction to match mono level
            // The L-R signal should be roughly the same amplitude as mono for proper stereo
            if (mono_rms > 1e-6 && lr_rms > 1e-8 && signal_present) {
                double target_ratio = 1.0;  // L-R should be comparable to mono
                double current_ratio = sqrt(lr_rms / mono_rms);
                
                if (current_ratio < 0.3) {  // L-R is too quiet
                    lr_gain_correction += lr_gain_alpha * 100.0 * (target_ratio - current_ratio);
                } else if (current_ratio > 1.5) {  // L-R is too loud
                    lr_gain_correction -= lr_gain_alpha * 100.0 * (current_ratio - target_ratio);
                }
                
                // Limit gain correction range
                if (lr_gain_correction < 1.0) lr_gain_correction = 1.0;
                if (lr_gain_correction > 4.0) lr_gain_correction = 4.0;
            }
            
            // Apply adaptive gain to L-R signal
            lr_filtered *= lr_gain_correction;
            
            // Apply delay compensation for phase alignment
            lr_delay_line[delay_index] = lr_filtered;
            size_t delayed_index = (delay_index + delay_samples - 1) % delay_samples;
            double lr_compensated = lr_delay_line[delayed_index];
            
            mono_delay_line[delay_index] = mono_clean;
            double mono_compensated = mono_delay_line[delayed_index];
            
            delay_index = (delay_index + 1) % delay_samples;

            // Matrix to stereo channels with improved separation, swaped
            double left_raw = mono_compensated + lr_compensated;
            double right_raw = mono_compensated - lr_compensated;

            // Store reference signals before any processing
            left_reference = left_raw;
            right_reference = right_raw;
            
            // Adaptive DC offset removal
            left_dc_offset += balance_alpha * (left_raw - left_dc_offset);
            right_dc_offset += balance_alpha * (right_raw - right_dc_offset);
            left_raw -= left_dc_offset;
            right_raw -= right_dc_offset;
            
            // Track channel energies for balance detection
            left_energy += balance_alpha * (left_raw * left_raw - left_energy);
            right_energy += balance_alpha * (right_raw * right_raw - right_energy);

            // Direct crosstalk measurement and cancellation
            // When right is dominant, measure left channel content as crosstalk
            if (right_energy > 4.0 * left_energy && right_energy > 0.001) {
                // Right dominant - left signal is mostly crosstalk
                double crosstalk_error = left_raw;
                right_to_left_leak += crosstalk_alpha * (crosstalk_error - right_to_left_leak * right_reference);
                
                // Limit crosstalk coefficient
                if (right_to_left_leak > 0.5) right_to_left_leak = 0.5;
                if (right_to_left_leak < -0.5) right_to_left_leak = -0.5;
            }
            
            // When left is dominant, measure right channel content as crosstalk  
            if (left_energy > 4.0 * right_energy && left_energy > 0.001) {
                // Left dominant - right signal is mostly crosstalk
                double crosstalk_error = right_raw;
                left_to_right_leak += crosstalk_alpha * (crosstalk_error - left_to_right_leak * left_reference);
                
                // Limit crosstalk coefficient
                if (left_to_right_leak > 0.5) left_to_right_leak = 0.5;
                if (left_to_right_leak < -0.5) left_to_right_leak = -0.5;
            }

            // Apply crosstalk cancellation
            left_raw -= right_to_left_leak * right_reference;
            right_raw -= left_to_right_leak * left_reference;

            // Additional aggressive silencing for very imbalanced signals
            double total_energy = left_energy + right_energy;
            if (total_energy > 1e-12) {
                double left_ratio = left_energy / total_energy;
                double right_ratio = right_energy / total_energy;
                
                // If one channel is >90% dominant, aggressively suppress the other
                if (right_ratio > 0.9) {
                    left_raw *= 0.1;  // Reduce left by 20dB
                }
                if (left_ratio > 0.9) {
                    right_raw *= 0.1;  // Reduce right by 20dB  
                }
                
                // If one channel is >95% dominant, almost completely suppress the other
                if (right_ratio > 0.95) {
                    left_raw *= 0.03;  // Reduce left by 30dB
                }
                if (left_ratio > 0.95) {
                    right_raw *= 0.03;  // Reduce right by 30dB
                }
            }

            if(delay_enabled)
            {
                raw_left_output[i] = left_raw;
                raw_right_output[i] = right_raw;
            } else {
                test_lp15k_output[i] = mono_clean;
                test_lmr_filtered_output[i] = lr_filtered;
            }

        }

        for(size_t i = 0; i < available_samples; i++) {
            
            double left_raw;
            double right_raw;

            if(delay_enabled)
            {
                left_raw = tanh(raw_left_output[i] * 0.8);
                right_raw = tanh(raw_right_output[i] * 0.8);
            } else {
                // Matrix to stereo channels with proper scaling
                left_raw = tanh((test_lp15k_output[i] - test_lmr_filtered_output[i]) * 0.5);
                right_raw = tanh((test_lp15k_output[i] + test_lmr_filtered_output[i]) * 0.5);
            }

            input_left[i_left]    = deemphasisFilter(left_raw, true);
            input_right[i_right]  = deemphasisFilter(right_raw, false);

            i_left++;
            i_right++;
        }

#if !TEST_DIRECTFMINPUT
        state_left = left_decimator.process(denomImmutable, denomState_left, input_left, i_left, writeable_stereo_frames, rate);

        //denomState_right->where = denomState_left->where;  // Keep in sync
        
        state_right = right_decimator.process(denomImmutable, denomState_right, input_right, i_right, writeable_stereo_frames, rate);

        size_t max_input_consumed = (state_left.input_processed + state_right.input_processed) / 2;
        size_t min_output = state_left.output_processed + state_right.output_processed;
        
        /*if (state_left.output_processed != state_right.output_processed) {
            printf("WARNING: Channel output mismatch! Left=%zu, Right=%zu\n", state_left.output_processed, state_right.output_processed);
        }*/

        // Interleave output
        for (size_t i = 0; i < min_output / 2; i++) {
            output[i * 2] = state_left.output[i];
            output[i * 2 + 1] = state_right.output[i];
        }

        size_t input_samples_consumed = max_input_consumed;  // Convert frames to samples
        size_t output_samples_produced = min_output;         // Convert frames to samples
#else
        size_t max_input_consumed = (i_left + i_right) / 2;
        size_t min_output = i_left + i_right;

        for (size_t i = 0; i < i_left; i++) {
            output[i * 2] = input_left[i];
            output[i * 2 + 1] = input_right[i];
        }

        size_t input_samples_consumed = max_input_consumed;  // Convert frames to samples
        size_t output_samples_produced = min_output;         // Convert frames to samples
#endif
        //printf("Advancing: reader by %zu samples, writer by %zu samples\n", input_samples_consumed, output_samples_produced);
        
        this->reader->advance(input_samples_consumed);
        this->writer->advance(output_samples_produced);
    }
}

namespace Csdr {
    template class MonoFractionalDecimator<float>;
    template class StereoFractionalDecimator<float>;
}