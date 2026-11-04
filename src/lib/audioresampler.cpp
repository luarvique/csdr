/*
Copyright (c) 2021 Jakob Ketterl <jakob.ketterl@gmx.de>
Copyright (c) 2026 Marat Fayzullin <luarvique@gmail.com>

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

template <typename T>
AudioResampler<T>::AudioResampler(double rate):
    rate(rate)
{
    int error = 0;

    // T can be either float (mono) or complex<float> (stereo)
    srcState = src_new(SRC_SINC_MEDIUM_QUALITY, sizeof(T) / sizeof(float), &error);
}

template <typename T>
AudioResampler<T>::AudioResampler(unsigned int inputRate, unsigned int outputRate):
    AudioResampler((double) outputRate / inputRate)
{}

template <typename T>
AudioResampler<T>::~AudioResampler() {
    src_delete(srcState);
}

template <typename T>
bool AudioResampler<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return this->reader->available() > 0 && this->writer->writeable() > 0;
}

template <typename T>
void AudioResampler<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    SRC_DATA data = {
        .data_in = (float *) this->reader->getReadPointer(),
        .data_out = (float *) this->writer->getWritePointer(),
        .input_frames = (long) this->reader->available(),
        .output_frames = (long) this->writer->writeable(),
        .end_of_input = 0,
        .src_ratio = rate
    };

    src_process(srcState, &data);

    this->reader->advance(data.input_frames_used);
    this->writer->advance(data.output_frames_gen);
}

namespace Csdr {
    template class AudioResampler<float>;
    template class AudioResampler<complex<float>>;
}
