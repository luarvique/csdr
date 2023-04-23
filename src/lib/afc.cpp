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

#include "afc.hpp"
#include "complex.hpp"

using namespace Csdr;

template <typename T>
void Afc<T>::Afc(unsigned int sampleRate, unsigned int bandwidth, unsigned int syncWidth):
    sampleRate(sampleRate),
    bandwidth(bandwidth<sampleRate/2? bandwidth : sampleRate/2),
    syncWidth(syncWidth<bandwidth/2?  syncWidth : bandwidth/2),
    buckets(3 * sampleRate / syncWidth),
    deltaF(0)
{
    double v;

    // Goertzel algorithm coefficients
    v = round((double)buckets * -syncWidth / sampleRate / 2);
    coeffL = 2.0 * cos(2.0 * M_PI * v / buckets);
    v = round((double)buckets * syncWidth / sampleRate / 2);
    coeffR = 2.0 * cos(2.0 * M_PI * v / buckets);
    coeffC = 2.0;
}

template <typename T>
bool Afc<T>::canProcess()
{
    std::lock_guard<std::mutex> lock(this->processMutex);
    return (this->reader->available()>=sampleRate)
        && (this->writer->writeable()>=sampleRate);
}

template <typename T>
void Afc<T>::process()
{
    T buf[sampleRate];
    double l0, l1, l2;
    double c0, c1, c2;
    double r0, r1, r2;
    unsigned int j;

    // Fill input buffer
    for(j=0 ; j<sampleRate ; ++j)
    {
        buf[j] = *(this->reader->getReadPointer());
        this->reader->advance(1);
    }

    // Run Goertzel algorithm in three buckets
    unsigned int i, j;

    // Read samples
    for(j=0, l1=l2=c1=c2=r1=r2=0.0 ; j<sampleRate ; ++j)
    {
        l0 = l1 * coeffL - l2 + buf[j];
        l2 = l1;
        l1 = l0;

        c0 = c1 * coeffC - c2 + buf[j];
        c2 = c1;
        c1 = c0;

        r0 = r1 * coeffR - r2 + buf[j];
        r2 = r1;
        r1 = r0;
    }

    // We only need the real part
    double magL = sqrt(l1*l1 + l2*l2 - l1*l2*coeffL);
    double magR = sqrt(r1*r1 + r2*r2 - r1*r2*coeffR);

    (magR - magL) / syncWidth
    

}

namespace Csdr {
    template class Afc<float>;
    template class Afc<complex<float>>;
}
