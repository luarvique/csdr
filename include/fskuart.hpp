/*
This file is part of libcsdr, licensed under the GNU General Public License,
version 3 or (at your option) any later version. See LICENSE-GPL for details.
*/
#pragma once

#include "module.hpp"
#include <array>
#include <complex>
#include <string>
#include <vector>

namespace Csdr {

    // Audio 2-FSK to asynchronous octets. Unlike RTTY's Baudot alphabet,
    // telemetry modems transmit all eight bits, optionally followed by parity.
    // Both character lengths are tried; the protocol checksum selects a frame.
    // Output: "bits comma-separated-start-samples hex-octets\n" per UART run.
    // The absolute sample positions identify duplicate UART interpretations
    // without suppressing a later transmission of the same octets.
    class FskUartDecoder: public Module<float, unsigned char> {
        public:
            explicit FskUartDecoder(unsigned int sampleRate, double baudRate = 1200,
                double markFreq = 1300, double spaceFreq = 2100,
                double gapChars = 3.5, unsigned int maxFrame = 264);
            bool canProcess() override;
            void process() override;

        private:
            struct Uart {
                unsigned int bits;
                uint64_t gap;
                bool previous = true;
                int state = -1;
                double next = 0;
                unsigned int shift = 0;
                uint64_t start = 0;
                uint64_t lastEnd = 0;
                std::vector<unsigned char> bytes;
                std::vector<uint64_t> starts;
            };

            void step(Uart& uart, bool bit);
            void finish(Uart& uart);
            void writePending();

            double samplesPerBit;
            double dcAlpha;
            double dc = 0;
            std::complex<double> markRotation, spaceRotation;
            std::complex<double> markPhase{1, 0}, spacePhase{1, 0};
            std::complex<double> markSum{0, 0}, spaceSum{0, 0};
            std::vector<std::complex<double>> markWindow, spaceWindow;
            size_t windowIndex = 0;
            uint64_t sample = 0;
            unsigned int maxFrame;
            std::array<Uart, 2> uarts;
            // At most two bounded UART runs can finish at the same sample.
            // Drain them before consuming further input, even at end of input.
            std::string pending;
            size_t pendingOffset = 0;
    };
}
