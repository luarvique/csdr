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

#include "logaveragepower.hpp"

#include <cstring>

using namespace Csdr;

LogAveragePower::LogAveragePower(unsigned int fftSize, unsigned int avgNumber, float add_db): fftSize(fftSize), avgNumber(avgNumber), add_db(add_db) {
    collector = (float*) malloc(sizeof(float) * fftSize);
    std::memset(collector, 0, sizeof(float) * fftSize);
}

LogAveragePower::LogAveragePower(unsigned int fftSize, unsigned int avgNumber): LogAveragePower(fftSize, avgNumber, 0.0) {}

LogAveragePower::~LogAveragePower() {
    free(collector);
}

void LogAveragePower::setAvgNumber(unsigned int avgNumber) {
    this->avgNumber = avgNumber;
}

bool LogAveragePower::canProcess() {
    std::lock_guard<std::mutex> lock(processMutex);
    return reader->available() > fftSize && writer->writeable() > fftSize;
}

void LogAveragePower::process() {
    std::lock_guard<std::mutex> lock(processMutex);
    complex<float>* input = reader->getReadPointer();
    for (int i = 0; i < fftSize; i++) {
        collector[i] += std::norm(input[i]);
    }
    reader->advance(fftSize);
    if (++collected == avgNumber) {
        float* output = writer->getWritePointer();
        float correction = add_db - 10.0f * std::log10(avgNumber);

        for (int i = 0; i < fftSize; i++) {
            output[i] = std::log10(collector[i]);
        }

        for (int i = 0; i < fftSize; i++) {
            output[i] = 10.0f * output[i] + correction;
        }

        writer->advance(fftSize);

        std::memset(collector, 0, sizeof(float) * fftSize);
        collected = 0;
    }
}
