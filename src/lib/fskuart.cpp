/*
This file is part of libcsdr, licensed under the GNU General Public License,
version 3 or (at your option) any later version. See LICENSE-GPL for details.
*/
#include "fskuart.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <limits>
#include <stdexcept>

using namespace Csdr;

// CSDR is built with -ffast-math, which may optimize std::isfinite away.
// Inspect the exponent for constructor validation before any allocation.
static bool finiteParameter(double value) {
    static_assert(sizeof(double) == sizeof(uint64_t) && std::numeric_limits<double>::is_iec559,
        "IEEE 754 binary64 double required");
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}

FskUartDecoder::FskUartDecoder(unsigned int sampleRate, double baudRate,
    double markFreq, double spaceFreq, double gapChars, unsigned int maxFrame):
    maxFrame(maxFrame)
{
    if (!sampleRate || !finiteParameter(baudRate) || !finiteParameter(markFreq)
        || !finiteParameter(spaceFreq) || !finiteParameter(gapChars)
        || baudRate <= 0 || baudRate > sampleRate / 2.0
        || markFreq <= 0 || markFreq >= sampleRate / 2.0
        || spaceFreq <= 0 || spaceFreq >= sampleRate / 2.0
        || markFreq == spaceFreq || gapChars <= 0 || gapChars > 100
        || !maxFrame || maxFrame > 4096) {
        throw std::invalid_argument("invalid FSK/UART parameters");
    }
    samplesPerBit = sampleRate / baudRate;
    // Bound allocation even for an accidentally tiny baud rate.
    if (samplesPerBit > 1000000) {
        throw std::invalid_argument("FSK bit window is too large");
    }
    const size_t window = std::max<size_t>(2, std::lround(samplesPerBit));
    markWindow.resize(window);
    spaceWindow.resize(window);
    const double tau = 2 * std::acos(-1.0);
    markRotation = std::polar(1.0, -tau * markFreq / sampleRate);
    spaceRotation = std::polar(1.0, -tau * spaceFreq / sampleRate);
    dcAlpha = 1 - std::exp(-tau * 50.0 / sampleRate);
    for (size_t i = 0; i < uarts.size(); ++i) {
        Uart& uart = uarts[i];
        uart.bits = 8 + i;
        uart.gap = std::llround(gapChars * (uart.bits + 2) * samplesPerBit);
        uart.bytes.reserve(maxFrame);
        uart.starts.reserve(maxFrame);
    }
}

bool FskUartDecoder::canProcess() {
    std::lock_guard<std::mutex> lock(processMutex);
    return writer->writeable() > 0 && (!pending.empty() || reader->available() > 0);
}

void FskUartDecoder::writePending() {
    const size_t count = std::min(writer->writeable(), pending.size() - pendingOffset);
    if (!count) return;
    std::memcpy(writer->getWritePointer(), pending.data() + pendingOffset, count);
    writer->advance(count);
    pendingOffset += count;
    if (pendingOffset == pending.size()) {
        pending.clear();
        pendingOffset = 0;
    }
}

void FskUartDecoder::process() {
    std::lock_guard<std::mutex> lock(processMutex);
    if (!pending.empty()) {
        writePending();
        return;
    }
    if (!writer->writeable()) return;
    const float* input = reader->getReadPointer();
    const size_t available = reader->available();
    size_t consumed = 0;
    while (consumed < available && pending.empty()) {
        double value = input[consumed++];
        // Remove FM discriminator DC, then compare sliding one-bit DFT energies.
        dc += (value - dc) * dcAlpha;
        value -= dc;
        markPhase *= markRotation;
        spacePhase *= spaceRotation;
        const auto mark = value * markPhase;
        const auto space = value * spacePhase;
        markSum += mark - markWindow[windowIndex];
        spaceSum += space - spaceWindow[windowIndex];
        markWindow[windowIndex] = mark;
        spaceWindow[windowIndex] = space;
        windowIndex = (windowIndex + 1) % markWindow.size();
        const bool bit = std::norm(markSum) >= std::norm(spaceSum);
        for (auto& uart: uarts) step(uart, bit);
        ++sample;
        // Bound numerical drift during continuous reception.
        if ((sample & 0xffff) == 0) {
            markPhase /= std::abs(markPhase);
            spacePhase /= std::abs(spacePhase);
            markSum = std::accumulate(markWindow.begin(), markWindow.end(), std::complex<double>{0, 0});
            spaceSum = std::accumulate(spaceWindow.begin(), spaceWindow.end(), std::complex<double>{0, 0});
        }
    }
    reader->advance(consumed);
    writePending();
}

void FskUartDecoder::step(Uart& uart, bool bit) {
    if (uart.state < 0) {
        if (!uart.bytes.empty() && sample - uart.lastEnd > uart.gap) finish(uart);
        if (uart.previous && !bit) {
            uart.state = 0;
            uart.next = sample + samplesPerBit / 2;
            uart.shift = 0;
            uart.start = sample;
        }
        uart.previous = bit;
        return;
    }
    uart.previous = bit;
    if (sample < uart.next) return;
    uart.next += samplesPerBit;
    if (uart.state == 0) {
        // Reject a short falling-edge glitch at the centre of the start bit.
        uart.state = bit ? -1 : 1;
    } else if (uart.state <= static_cast<int>(uart.bits)) {
        uart.shift |= static_cast<unsigned int>(bit) << (uart.state - 1);
        ++uart.state;
    } else {
        uart.state = -1;
        // Consume the whole character before resynchronizing after a bad stop.
        if (!bit) {
            finish(uart);
            return;
        }
        uart.bytes.push_back(uart.shift & 0xff);
        uart.starts.push_back(uart.start);
        uart.lastEnd = sample;
        if (uart.bytes.size() >= maxFrame) finish(uart);
    }
}

void FskUartDecoder::finish(Uart& uart) {
    if (uart.bytes.empty()) return;
    pending += std::to_string(uart.bits) + " ";
    for (size_t i = 0; i < uart.starts.size(); ++i) {
        if (i) pending += ',';
        pending += std::to_string(uart.starts[i]);
    }
    pending += ' ';
    const char* hex = "0123456789abcdef";
    for (unsigned char byte: uart.bytes) {
        pending += hex[byte >> 4];
        pending += hex[byte & 15];
    }
    pending += '\n';
    uart.bytes.clear();
    uart.starts.clear();
}
