/*
Copyright (c) 2023 Marat Fayzullin <luarvique@gmail.com>

This file is part of libcsdr.

libcsdr is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

libcsdr is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with libcsdr.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "resampler.hpp"

using namespace Csdr;

Resampler::Resampler(double rate):
    rate(rate)
{
    int error = 0;
    srcState = src_new(SRC_SINC_FASTEST, 2, &error);
}

Resampler::Resampler(unsigned int inputRate, unsigned int outputRate):
    Resampler((double) outputRate / inputRate)
{}

Resampler::~Resampler() {
    src_delete(srcState);
}

bool Resampler::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0 && writer->writeable() > 0;
}

void Resampler::process() {
    std::lock_guard<std::mutex> lock(processMutex);
    SRC_DATA data = {
        .data_in = (float *)reader->getReadPointer(),
        .data_out = (float *)writer->getWritePointer(),
        .input_frames = (long) reader->available(),
        .output_frames = (long) writer->writeable(),
        .end_of_input = 0,
        .src_ratio = rate
    };

    src_process(srcState, &data);

    reader->advance(data.input_frames_used);
    writer->advance(data.output_frames_gen);
}

