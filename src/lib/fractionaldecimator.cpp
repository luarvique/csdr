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

#include "fractionaldecimator.hpp"
#include <cmath>

using namespace Csdr;

template <typename T>
FractionalDecimator<T>::FractionalDecimator(float rate, unsigned int num_poly_points, FirFilter<T, float> *filter):
    num_poly_points(num_poly_points &~ 1),
    poly_precalc_denomiator((float*) malloc(this->num_poly_points * sizeof(float))),
    xifirst(-(this->num_poly_points / 2) + 1),
    xilast(this->num_poly_points / 2),
    coeffs_buf((float*) malloc(this->num_poly_points * sizeof(float))),
    rate(rate),
    filter(filter)
{
    int id = 0; //index in poly_precalc_denomiator
    for (int xi = xifirst; xi <= xilast; xi++) {
        poly_precalc_denomiator[id] = 1;
        for(int xj = xifirst; xj <= xilast; xj++) {
            //poly_precalc_denomiator could be integer as well. But that would later add a necessary conversion.
            if (xi != xj) poly_precalc_denomiator[id] *= (xi - xj);
        }
        id++;
    }

    where = -xifirst;
}

template <typename T>
FractionalDecimator<T>::~FractionalDecimator() {
    free(poly_precalc_denomiator);
    free(coeffs_buf);
}

template <typename T>
bool FractionalDecimator<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    size_t size = std::min(this->reader->available(), (size_t) ceil(this->writer->writeable() / rate));
    size_t filterLen = filter != nullptr ? filter->getOverhead() : 0;
    return ceil(where) + num_poly_points + filterLen < size;
}

template <typename T>
void FractionalDecimator<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    //This routine can handle floating point decimation rates.
    //It applies polynomial interpolation to samples that are taken into consideration from a pre-filtered input.
    //The pre-filter can be switched off by applying filter = nullptr.
    int oi = 0; //output index
    int index_high, index;
    size_t size = std::min(this->reader->available(), (size_t) ceil(this->writer->writeable() / rate));
    size_t filterLen = filter != nullptr ? filter->getOverhead() : 0;
    T* input = this->reader->getReadPointer();
    T* output = this->writer->getWritePointer();
    //we optimize to calculate ceil(where) only once every iteration, so we do it here:
    while ((index_high = ceil(where)) + num_poly_points + filterLen < size) {
        // num_poly_points above is theoretically more than we could have here, but this makes the spectrum look good
        index = index_high - 1;
        int id = 0;
        float xwhere = (float) (where - index);
        for (int xi = xifirst; xi <= xilast; xi++) {
            coeffs_buf[id] = 1;
            for (int xj = xifirst; xj <= xilast; xj++) {
                if (xi != xj) coeffs_buf[id] *= (xwhere - xj);
            }
            id++;
        }
        T acc = 0;
        if (filter != nullptr) {
            for (int i = 0; i < num_poly_points; i++) {
                SparseView<T> sparse = filter->sparse(input);
                acc += (coeffs_buf[i] / poly_precalc_denomiator[i]) * sparse[index + i];
            }
        } else {
            for (int i = 0; i < num_poly_points; i++) {
                acc += (coeffs_buf[i] / poly_precalc_denomiator[i]) * input[index + i];
            }
        }
        output[oi++] = acc;
        where += rate;
    }

    int input_processed = index + xifirst;
    where -= input_processed;

    this->reader->advance(input_processed);
    this->writer->advance(oi);
}

namespace Csdr {
    template class FractionalDecimator<float>;
    template class FractionalDecimator<complex<float>>;
}