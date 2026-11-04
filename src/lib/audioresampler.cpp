/*
Copyright (c) 2021 Jakob Ketterl <jakob.ketterl@gmx.de>

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

#include "audioresampler.hpp"

using namespace Csdr;

AudioResampler::AudioResampler(double rate, unsigned int channels):
    rate(rate),
    channels(channels)
{
    int error = 0;
    srcState = src_new(SRC_SINC_MEDIUM_QUALITY, (int) channels, &error);
}

AudioResampler::AudioResampler(unsigned int inputRate, unsigned int outputRate, unsigned int channels):
    AudioResampler((double) outputRate / inputRate, channels)
{}

AudioResampler::~AudioResampler() {
    src_delete(srcState);
}

bool AudioResampler::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return reader->available() > 0 && writer->writeable() > 0;
}

void AudioResampler::process() {
    std::lock_guard<std::mutex> lock(processMutex);
    long input_frames = (long) (reader->available() / channels);
    long output_frames = (long) (writer->writeable() / channels);
    SRC_DATA data = {
        .data_in = reader->getReadPointer(),
        .data_out = writer->getWritePointer(),
        .input_frames = input_frames,
        .output_frames = output_frames,
        .end_of_input = 0,
        .src_ratio = rate
    };

    src_process(srcState, &data);

    reader->advance(data.input_frames_used * channels);
    writer->advance(data.output_frames_gen * channels);
}

