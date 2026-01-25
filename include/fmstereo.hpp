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

#pragma once

#include "module.hpp"
#include "complex.hpp"
#include "fir.hpp"
#include "fmdemod.hpp"

#include <queue>
#include <functional>
#include <future>
#include <iostream>

namespace Csdr {

    class PilotPLL {
    private:
        double samplerate_;  // Sampling rate in Hz
        double pilot_freq_;  // Pilot frequency (typically 19 kHz)
        double alpha_;       // Integral gain (wn^2)
        double beta_;        // Proportional gain (2*damp*wn)
        double minfreq_;     // Minimum frequency (phase inc)
        double maxfreq_;     // Maximum frequency (phase inc)
        double phzref_;      // Reference phase increment
        double freq_;        // Current frequency (phase inc)
        double phase_;       // Current phase of the NCO
        
        // Lock detection (from original)
        double lock_;        // Lock metric
        double lockalpha_;   // Lock filter alpha
        double lockbeta_;    // Lock filter beta
        double locklimit_;   // Lock threshold
        double lockdelay_;   // Lock delay time in samples
        double lockcount_;   // Lock delay counter
        
    public:
        // Constructor
        // sample_rate: Audio sampling rate (e.g., 44100 Hz)
        // pilot_freq: Pilot tone frequency (default 19000 Hz)
        // damp: Damping factor (default 0.707 for critical damping)
        // bw: Loop bandwidth (e.g., 20 Hz for stable lock)
        PilotPLL(double sample_rate, double pilot_freq = 19000.0, double damp = 0.707, double bw = 30.0)
        : samplerate_(sample_rate), pilot_freq_(pilot_freq), phase_(0.0), lock_(0.0), lockcount_(0.0) {
            
            double fn = bw * 0.707;  // Natural frequency approximation
            double wn = 2.0 * M_PI * fn / samplerate_;
            
            alpha_ = wn * wn;            // Integral gain
            beta_ = 2.0 * damp * wn;     // Proportional gain
            
            double phz = 2.0 * M_PI * pilot_freq / samplerate_;
            minfreq_ = phz - 2.0 * M_PI * 50.0 / samplerate_;  // ±50 Hz limit
            maxfreq_ = phz + 2.0 * M_PI * 50.0 / samplerate_;
            phzref_ = phz;
            freq_ = phz;
            
            // Lock detection parameters
            lockalpha_ = 1.0 - std::exp(-1.0 / (samplerate_ * 0.2));  // 0.2 sec time constant
            lockbeta_ = 1.0 - lockalpha_;
            locklimit_ = 0.1;  // Lock threshold (low error means locked)
            lockdelay_ = samplerate_ * 0.5;  // 0.5 sec delay for stability
        }
        
        // Process a single sample from the composite signal.
        // input: The input sample (bandpassed pilot around 19 kHz).
        // Returns: The 38 kHz subcarrier sample (cos(2 * phase)) for demodulating (L-R).
        double process(double input, double& pilot_strength) {
            // Phase detector: input * sin(phase) ≈ sin(error) for small errors
            double error = input * std::sin(phase_);
            
            // Loop filter: PI control
            freq_ += alpha_ * error;  // Integrate error to frequency
            phase_ += freq_ + beta_ * error;  // Update phase with freq + proportional
            
            // Wrap phase to [-2π, 2π]
            if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
            else if (phase_ < -2.0 * M_PI) phase_ += 2.0 * M_PI;
            
            // Clamp frequency
            if (freq_ > maxfreq_) freq_ = maxfreq_;
            else if (freq_ < minfreq_) freq_ = minfreq_;
            
            // Lock detection
            double abserr = std::fabs(error);
            lock_ = lock_ * lockalpha_ + lockbeta_ * abserr;
            if (lock_ < locklimit_) {
                lockcount_ = lockdelay_;  // Reset delay counter if low error
            }
            if (lockcount_ > 0) {
                lockcount_--;
            }
            
            // Set pilot_strength based on lock (1.0 locked, 0.0 unlocked) - adapt as needed
            pilot_strength = (lockcount_ > 0) ? 1.0 - lock_ : 0.0;
            
            // Return doubled phase for 38 kHz subcarrier
            return std::cos(2.0 * phase_);
        }
        
        void reset() {
            phase_ = 0.0;
            freq_ = phzref_;
            lock_ = 0.0;
            lockcount_ = 0.0;
        }
        
        double getPhase() {
            return phase_;
        }
    };

    class BiquadFilter {
    private:
        double b0, b1, b2, a1, a2;  // Filter coefficients
        double x1, x2, y1, y2;     // Delay line states
        
    public:
        BiquadFilter() : b0(1.0), b1(0.0), b2(0.0), a1(0.0), a2(0.0), 
                        x1(0.0), x2(0.0), y1(0.0), y2(0.0) {}
        
        void setBandpass(double fc, double Q, double fs) {
            double omega = 2.0 * M_PI * fc / fs;
            double alpha = sin(omega) / (2.0 * Q);
            double cos_omega = cos(omega);
            
            // Bandpass coefficients
            double norm = 1.0 + alpha;
            b0 = alpha / norm;
            b1 = 0.0;
            b2 = -alpha / norm;
            a1 = -2.0 * cos_omega / norm;
            a2 = (1.0 - alpha) / norm;
        }

        void setBandpass2(double fc, double bw, double fs) {
            double Q = fc / bw;
            double omega = 2.0 * M_PI * fc / fs;
            // Optional: Adjust for bilinear warping if fc is high relative to fs/2
            double adj = (omega > 0.0) ? std::sin(omega) / omega : 1.0;
            Q /= adj;  // Compensates bandwidth cramping; omit if fs >> 2*fc (e.g., 192 kHz+)
            double alpha = std::sin(omega) / (2.0 * Q);
            double cos_omega = std::cos(omega);

            double norm = 1.0 + alpha;
            b0 = alpha / norm;
            b1 = 0.0;
            b2 = -alpha / norm;
            a1 = -2.0 * cos_omega / norm;
            a2 = (1.0 - alpha) / norm;
        }
        
        void setLowpassWithQ(double fc, double Q, double fs) {
            double omega = 2.0 * M_PI * fc / fs;
            double cos_omega = cos(omega);
            double alpha = sin(omega) / (2.0 * Q);
            
            double norm = 1.0 + alpha;
            b0 = (1.0 - cos_omega) / 2.0 / norm;
            b1 = (1.0 - cos_omega) / norm;
            b2 = (1.0 - cos_omega) / 2.0 / norm;
            a1 = -2.0 * cos_omega / norm;
            a2 = (1.0 - alpha) / norm;
        }
        
        void setLowpass(double fc, double fs) {
            setLowpassWithQ(fc, 0.707, fs);  // Standard Butterworth Q
        }
        
        void setHighpass(double fc, double fs) {
            double omega = 2.0 * M_PI * fc / fs;
            double cos_omega = cos(omega);
            double alpha = sin(omega) / sqrt(2.0);
            
            double norm = 1.0 + alpha;
            b0 = (1.0 + cos_omega) / 2.0 / norm;
            b1 = -(1.0 + cos_omega) / norm;
            b2 = (1.0 + cos_omega) / 2.0 / norm;
            a1 = -2.0 * cos_omega / norm;
            a2 = (1.0 - alpha) / norm;
        }
        
        double process(double input) {
            double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            
            // Update delay line
            x2 = x1;
            x1 = input;
            y2 = y1;
            y1 = output;
            
            return output;
        }
        
        void reset() {
            x1 = x2 = y1 = y2 = 0.0;
        }
    };

    class MultistageFilter {
    private:
        static const int MAX_STAGES = 4;
        BiquadFilter stages[MAX_STAGES];
        int num_stages;
        
    public:
        MultistageFilter() : num_stages(1) {}
        
        void setLowpass(double fc, double fs, int order = 6) {
            num_stages = (order + 1) / 2;  // Each biquad = 2 poles
            if (num_stages > MAX_STAGES) num_stages = MAX_STAGES;
            
            // Calculate Butterworth poles for higher order filter
            for (int i = 0; i < num_stages; ++i) {
                double angle = M_PI * (2 * i + 1) / (2 * order);
                double Q = 1.0 / (2.0 * cos(angle));
                
                // Limit Q to reasonable values
                if (Q < 0.5) Q = 0.5;
                if (Q > 10.0) Q = 10.0;
                
                stages[i].setLowpassWithQ(fc, Q, fs);
            }
        }
        
        void setBandpass(double fc, double Q, double fs) {
            num_stages = 1;
            stages[0].setBandpass(fc, Q, fs);
        }
        
        void setHighpass(double fc, double fs) {
            num_stages = 1;
            stages[0].setHighpass(fc, fs);
        }
        
        double process(double input) {
            double output = input;
            for (int i = 0; i < num_stages; ++i) {
                output = stages[i].process(output);
            }
            return output;
        }
        
        void reset() {
            for (int i = 0; i < num_stages; ++i) {
                stages[i].reset();
            }
        }
    };
    
    class NotchFilter {
    private:
        double b0, b1, b2, a1, a2;
        double x1, x2, y1, y2;
        
    public:
        NotchFilter() : b0(1.0), b1(0.0), b2(0.0), a1(0.0), a2(0.0),
                        x1(0.0), x2(0.0), y1(0.0), y2(0.0) {}
        
        void setNotch(double fc, double Q, double fs) {
            double omega = 2.0 * M_PI * fc / fs;
            double alpha = sin(omega) / (2.0 * Q);
            double cos_omega = cos(omega);
            
            double norm = 1.0 + alpha;
            b0 = 1.0 / norm;
            b1 = -2.0 * cos_omega / norm;
            b2 = 1.0 / norm;
            a1 = -2.0 * cos_omega / norm;
            a2 = (1.0 - alpha) / norm;
        }
        
        double process(double input) {
            double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            
            x2 = x1; x1 = input;
            y2 = y1; y1 = output;
            
            return output;
        }
        
        void reset() {
            x1 = x2 = y1 = y2 = 0.0;
        }
    };
        
    class SimpleDCBlock {
        float R;
        float prev_in;
        float prev_out;
    public:
        SimpleDCBlock(float R_ = 0.995f) : R(R_), prev_in(0.0f), prev_out(0.0f) {}
        float process(float x) {
            float y = x - prev_in + R * prev_out;
            prev_in = x;
            prev_out = y;
            return y;
        }
    };

    template <typename T>
    class MonoFractionalDecimator {

        public:
            MonoFractionalDecimator();
            ~MonoFractionalDecimator();
            
            struct Denominator {

                Denominator() { 
                    readerAvailable = 0;
                    writterAvailable = 0;

                    where = 0;
                    num_poly_points = 0;

                    poly_precalc_denomiator.resize(0);
                    coeffs_buf.resize(0);

                    xifirst = 0;
                    xilast = 0;
                    rate = 0;
                    output_processed = 0;
                }

                size_t readerAvailable;
                size_t writterAvailable;

                float where;
                size_t output_processed;
                unsigned int num_poly_points; //number of samples that the Lagrange interpolator will use
                std::vector<float> poly_precalc_denomiator; //while we don't precalculate coefficients here as in a Farrow structure, because it is a fractional interpolator, but we rather precaculate part of the interpolator expression
                std::vector<float> coeffs_buf;
                int xifirst;
                int xilast;
                float rate;
            };

            struct DenominatorImmutable {
                unsigned int num_poly_points;
                std::vector<float> poly_precalc_denomiator;
                int xifirst;
                int xilast;
                float rate;

                FirFilter<T, float>* filter;

                DenominatorImmutable(unsigned int n, float r, FirFilter<T, float>* f) 
                    : num_poly_points(n & ~1), rate(r), filter(f)
                {
                    xifirst = -(num_poly_points / 2) + 1;
                    xilast  = num_poly_points / 2;
                    poly_precalc_denomiator.resize(num_poly_points);

                    for (int xi = xifirst, id = 0; xi <= xilast; xi++, id++) {
                        poly_precalc_denomiator[id] = 1;
                        for (int xj = xifirst; xj <= xilast; xj++) {
                            if (xi != xj) poly_precalc_denomiator[id] *= (xi - xj);
                        }
                    }
                }
            };

            struct DenominatorState {
                float where; // szál-specifikus állapot
                size_t output_processed;
                std::vector<float> coeffs_buf;

                std::mutex processMutex;

                DenominatorState() {};

                DenominatorState(int startWhere, int num_poly_points) 
                    : where(startWhere), output_processed(0) {
                        coeffs_buf.resize(num_poly_points);
                    }
            };

            struct ProcessState {
                size_t input_processed;
                size_t output_processed;
                std::vector<T> output;

                ProcessState() : input_processed(0), output_processed(0) {}
            };

            Denominator calculateDenominator(float rate, unsigned int num_poly_points, FirFilter<T, float>* filter = nullptr);
            bool canProcess(DenominatorImmutable* denom, DenominatorState* denomState, size_t readerAvailable, size_t writterAvailable, float rate);

            ProcessState process(DenominatorImmutable* denom, DenominatorState* denomState, std::vector<T> input, size_t readerAvailable, size_t writerWriteable, float rate);

        private:

    };

    template <typename T>
    class StereoFractionalDecimator: public Module<T, T> {
        
        public:
            StereoFractionalDecimator(float rateMPX, float rate, unsigned int num_poly_points, FirFilter<T, float>* filter = nullptr);
            ~StereoFractionalDecimator();
            
            bool canProcess() override;
            void process() override;

            double deemphasisFilter(double sample, bool is_left_channel) {
                if (is_left_channel) {
                    deemph_state_L = (1.0 - deemph_alpha) * sample + deemph_alpha * deemph_state_L;
                    return deemph_state_L;
                } else {
                    deemph_state_R = (1.0 - deemph_alpha) * sample + deemph_alpha * deemph_state_R;
                    return deemph_state_R;
                }
            }
            
            private:
            unsigned int num_poly_points; //number of samples that the Lagrange interpolator will use
            float rate;
            FirFilter<T, float>* filter;
            
            // FMDemodMPX START
            unsigned int inputSampleRate;
            unsigned int outputSampleRate;

            void initializeFilters();
            bool initializedFilters_ = false;

            BiquadFilter* filter_19k;      // 19kHz pilot tone bandpass
            NotchFilter* notch_19k;           // 19kHz pilot notch filter
            NotchFilter* notch_38k;           // 38kHz L-R carrier notch filter
            BiquadFilter* filter_hp;       // High-pass for DC removal
            MultistageFilter* filter_lp_lr;    // Low-pass for L-R signal
            MultistageFilter* filter_lp_mono;  // Low-pass for mono signal

            // Phase-coherent pilot processing
            PilotPLL* pilot_pll;              // Phase-locked loop for coherent 38kHz generation

            // Stereo presence detection
            double pilot_strength;
            double stereo_threshold;

            // Deemphasis filter states (50µs time constant)
            double deemph_alpha;
            double deemph_state_L;
            double deemph_state_R;

            // Adaptive carrier nulling
            double carrier_leak_i, carrier_leak_q;  // I/Q carrier leakage compensation
            double leak_alpha;                       // Adaptation rate

            // Channel balance correction
            double left_dc_offset, right_dc_offset;   // DC offset tracking
            double left_gain_correction, right_gain_correction;  // Gain imbalance correction
            double balance_alpha;                     // Balance adaptation rate
            double left_energy, right_energy;        // Channel energy tracking

            // Direct crosstalk cancellation
            double left_to_right_leak, right_to_left_leak;  // Measured crosstalk coefficients
            double crosstalk_alpha;                   // Crosstalk adaptation rate
            double left_reference, right_reference;   // Reference signals for crosstalk measurement

            double signal_level;
            double noise_floor;
            double gate_threshold;
            double gate_alpha;
            bool signal_present;

            // Adaptive L-R gain correction
            double lr_gain_correction;
            double lr_gain_alpha;
            double mono_rms, lr_rms;  // Track signal levels

            // Phase correction and delay compensation
            bool delay_enabled;
            static const size_t MAX_DELAY_SAMPLES = 16; // 16
            double lr_delay_line[MAX_DELAY_SAMPLES];
            double mono_delay_line[MAX_DELAY_SAMPLES];
            size_t delay_samples;
            size_t delay_index;

            // FMDemodMPX STOP

            typename MonoFractionalDecimator<T>::DenominatorImmutable* denomImmutable;

            typename MonoFractionalDecimator<T>::DenominatorState* denomState_left;
            typename MonoFractionalDecimator<T>::DenominatorState* denomState_right;

            MonoFractionalDecimator<T> left_decimator;
            MonoFractionalDecimator<T> right_decimator;
    };

}