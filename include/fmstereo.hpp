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
        double samplerate_;
        double pilot_freq_;
        double alpha_;
        double beta_;
        double minfreq_;
        double maxfreq_;
        double phzref_;
        double freq_;
        double phase_;

        double lock_;
        double lockalpha_;
        double lockbeta_;
        double locklimit_;
        double lockdelay_;
        double lockcount_;

    public:

        PilotPLL(double sample_rate, double pilot_freq = 19000.0, double damp = 0.707, double bw = 30.0)
        : samplerate_(sample_rate), pilot_freq_(pilot_freq), phase_(0.0), lock_(0.0), lockcount_(0.0) {

            double fn = bw * 0.707;
            double wn = 2.0 * M_PI * fn / samplerate_;

            alpha_ = wn * wn;
            beta_ = 2.0 * damp * wn;

            double phz = 2.0 * M_PI * pilot_freq / samplerate_;

            minfreq_ = phz - 2.0 * M_PI * 250.0 / samplerate_;
            maxfreq_ = phz + 2.0 * M_PI * 250.0 / samplerate_;
            phzref_ = phz;
            freq_ = phz;

            lockalpha_ = 1.0 - std::exp(-1.0 / (samplerate_ * 0.2));
            lockbeta_ = 1.0 - lockalpha_;
            locklimit_ = 0.1;
            lockdelay_ = samplerate_ * 0.5;
        }

        double processIQ(double input_i, double input_q, double& pilot_strength) {
            double nco_i = std::cos(phase_);
            double nco_q = std::sin(phase_);

            double re = input_i * nco_i + input_q * nco_q;
            double im = input_q * nco_i - input_i * nco_q;
            double error = std::atan2(im, re);

            if (error > 0.7) error = 0.7;
            else if (error < -0.7) error = -0.7;

            freq_ += alpha_ * error;
            phase_ += freq_ + beta_ * error;

            if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
            else if (phase_ < -2.0 * M_PI) phase_ += 2.0 * M_PI;

            if (freq_ > maxfreq_) freq_ = maxfreq_;
            else if (freq_ < minfreq_) freq_ = minfreq_;

            double mag = std::sqrt(input_i * input_i + input_q * input_q);
            lock_ = lock_ * lockalpha_ + lockbeta_ * std::fabs(error);
            if (lock_ < locklimit_ && mag > 1e-4) {
                lockcount_ = lockdelay_;
            }
            if (lockcount_ > 0.0) lockcount_--;

            pilot_strength = (lockcount_ > 0.0) ? (1.0 - lock_) : 0.0;
            if (pilot_strength < 0.0) pilot_strength = 0.0;
            if (pilot_strength > 1.0) pilot_strength = 1.0;

            return std::cos(2.0 * phase_);
        }

        double process(double input, double& pilot_strength) {
            return processIQ(input, 0.0, pilot_strength);
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
        double b0, b1, b2, a1, a2;
        double x1, x2, y1, y2;

    public:
        BiquadFilter() : b0(1.0), b1(0.0), b2(0.0), a1(0.0), a2(0.0),
                        x1(0.0), x2(0.0), y1(0.0), y2(0.0) {}

        void setBandpass(double fc, double Q, double fs) {
            double omega = 2.0 * M_PI * fc / fs;
            double alpha = sin(omega) / (2.0 * Q);
            double cos_omega = cos(omega);

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

            double adj = (omega > 0.0) ? std::sin(omega) / omega : 1.0;
            Q /= adj;
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
            setLowpassWithQ(fc, 0.707, fs);
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

        static const int MAX_STAGES = 12;
        BiquadFilter stages[MAX_STAGES];
        int num_stages;

    public:
        MultistageFilter() : num_stages(1) {}

        void setLowpass(double fc, double fs, int order = 6) {
            num_stages = (order + 1) / 2;
            if (num_stages > MAX_STAGES) num_stages = MAX_STAGES;

            for (int i = 0; i < num_stages; ++i) {
                double angle = M_PI * (2 * i + 1) / (2 * order);
                double Q = 1.0 / (2.0 * cos(angle));

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

                double where;
                size_t output_processed;
                unsigned int num_poly_points;
                std::vector<float> poly_precalc_denomiator;
                std::vector<float> coeffs_buf;
                int xifirst;
                int xilast;
                double rate;
            };

            struct DenominatorImmutable {
                unsigned int num_poly_points;
                std::vector<float> poly_precalc_denomiator;
                int xifirst;
                int xilast;
                double rate;

                FirFilter<T, float>* filter;

                DenominatorImmutable(unsigned int n, double r, FirFilter<T, float>* f)
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
                double where;
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

            Denominator calculateDenominator(double rate, unsigned int num_poly_points, FirFilter<T, float>* filter = nullptr);
            bool canProcess(DenominatorImmutable* denom, DenominatorState* denomState, size_t readerAvailable, size_t writterAvailable, double rate);

            ProcessState process(DenominatorImmutable* denom, DenominatorState* denomState, const std::vector<T>& input, size_t readerAvailable, size_t writerWriteable, double rate);

        private:

    };

    template <typename T>
    class StereoFractionalDecimator: public Module<T, complex<float>> {

        public:
            enum class StereoDemodMode {
                SdrPll,
                AnalyticQuadrature
            };

            StereoFractionalDecimator(float rateMPX, float rate, float tau, unsigned int num_poly_points, FirFilter<T, float>* filter = nullptr);
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

            void setStereoFactor(double factor) { stereo_factor = factor; }
            double getStereoFactor() const { return stereo_factor; }

            void setSubcarrierPhase(double degrees) {
                subcarrier_phase_rad = degrees * (M_PI / 180.0);
                subcarrier_phase_cos = std::cos(subcarrier_phase_rad);
                subcarrier_phase_sin = std::sin(subcarrier_phase_rad);
            }
            double getSubcarrierPhase() const { return subcarrier_phase_rad * (180.0 / M_PI); }

            void setPilotEnvelopeTimeConstant(double seconds) {
                if (seconds <= 0.0) return;
                env_sq_alpha = 1.0 - std::exp(-1.0 / (inputSampleRate * seconds));
            }

            void setStereoBlendThresholds(double pilot_low_rms, double pilot_high_rms) {
                blend_low_threshold  = pilot_low_rms;
                blend_high_threshold = pilot_high_rms;
            }
            double getStereoBlendLowThreshold()  const { return blend_low_threshold; }
            double getStereoBlendHighThreshold() const { return blend_high_threshold; }

            void setQuietBlendThresholds(double audio_low, double audio_high, double min_stereo) {
                quiet_blend_low = audio_low;
                quiet_blend_high = audio_high;
                quiet_blend_min_stereo = min_stereo;
            }

            void setDemodMode(StereoDemodMode mode) { demod_mode = mode; }
            StereoDemodMode getDemodMode() const { return demod_mode; }

            void setMpxAlignDelaySamples(size_t samples) {
                if (samples >= MPX_ALIGN_MAX_DELAY) samples = MPX_ALIGN_MAX_DELAY - 1;
                mpx_align_delay_samples = samples;
            }
            size_t getMpxAlignDelaySamples() const { return mpx_align_delay_samples; }

            private:
            unsigned int num_poly_points;
            float rate;
            FirFilter<T, float>* filter;

            unsigned int inputSampleRate;
            unsigned int outputSampleRate;

            void initializeFilters();
            bool initializedFilters_ = false;

            BiquadFilter* filter_19k;
            MultistageFilter* filter_lp_lr;
            MultistageFilter* filter_lp_mono;

            std::vector<double> input_fm_buf;
            std::vector<T> input_left_buf;
            std::vector<T> input_right_buf;

            std::vector<T> pending_left;
            std::vector<T> pending_right;

            double stereo_factor;

            double stereo_factor_auto;
            double stereo_factor_auto_alpha;

            double env_sq_smoothed;
            double env_sq_alpha;

            double blend_low_threshold;
            double blend_high_threshold;
            double pilot_blend_smoothed;
            double pilot_blend_alpha;

            double quiet_audio_env;
            double quiet_audio_alpha;
            double quiet_blend_low;
            double quiet_blend_high;
            double quiet_blend_min_stereo;

            static const size_t PILOT_HISTORY_LEN = 8;
            double pilot_history[PILOT_HISTORY_LEN];
            size_t pilot_history_idx;
            size_t quad_delay_int;
            double quad_delay_frac;

            static const size_t MPX_ALIGN_MAX_DELAY = 32;
            double mpx_delay_line[MPX_ALIGN_MAX_DELAY];
            size_t mpx_delay_index;
            size_t mpx_align_delay_samples;

            double subcarrier_phase_rad;
            double subcarrier_phase_cos;
            double subcarrier_phase_sin;

            PilotPLL* pilot_pll;

            double pll_polarity_metric;
            double pll_polarity_alpha;
            int pll_polarity_sign;
            bool pll_polarity_locked;
            size_t pll_polarity_lock_samples;

            double pilot_strength;

            double deemph_tau;
            double deemph_alpha;
            double deemph_state_L;
            double deemph_state_R;

            double deesser_lp_a;
            double deesser_lp_L;
            double deesser_lp_R;

            double left_dc_offset, right_dc_offset;
            double balance_alpha;

            StereoDemodMode demod_mode;

            typename MonoFractionalDecimator<T>::DenominatorImmutable* denomImmutable;

            typename MonoFractionalDecimator<T>::DenominatorState* denomState_left;
            typename MonoFractionalDecimator<T>::DenominatorState* denomState_right;

            MonoFractionalDecimator<T> left_decimator;
            MonoFractionalDecimator<T> right_decimator;
    };

}