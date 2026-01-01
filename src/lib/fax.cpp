/*
This software is part of libcsdr, a set of simple DSP routines for
Software Defined Radio.

Copyright (c) 2022-2023 Marat Fayzullin <luarvique@gmail.com>
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

#include "fax.hpp"
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cstdio>

using namespace Csdr;

#define STATE_HEADER (0)
#define STATE_SYNC   (1)
#define STATE_IMAGE  (2)

#define TYPE_IMAGE   (0)    // or >0 representing sync position
#define TYPE_IOC576  (-576)
#define TYPE_IOC288  (-288)
#define TYPE_STOP    (-1)
#define TYPE_INVALID (-2)

//
// BMP file header
//
typedef struct
{
  unsigned char magic[2];
  unsigned char fileSize[4];
  unsigned char reserved[4];
  unsigned char dataOffset[4];

  unsigned char dibSize[4];
  unsigned char width[4];
  unsigned char height[4];
  unsigned char planes[2];
  unsigned char bitCount[2];
  unsigned char compression[4];
  unsigned char imageSize[4];
  unsigned char xPixelsPerM[4];
  unsigned char yPixelsPerM[4];
  unsigned char colorsUsed[4];
  unsigned char colorsImportant[4];
} BMPHeader;

static int minus(const void *a, const void *b)
{
    return(*(const int *)a - *(const int *)b);
}

static int median(int *x, int n)
{
    qsort(x, n, sizeof(*x), minus);
    return(x[n/2]);
}

template <typename T>
FaxDecoder<T>::FaxDecoder(unsigned int sampleRate, unsigned int lpm, unsigned int maxLines, unsigned int options, unsigned int dbgTime)
: sampleRate(sampleRate),
  lpm(lpm),
  maxLines(maxLines),
  am(!!(options & OPT_AM)),
  post(!!(options & OPT_POST)),
  colors(options & OPT_COLOR? 3:1),
  syncLines(options & OPT_SYNC),
  curState(STATE_HEADER),
  lastType(TYPE_INVALID),
  typeCount(0),
  curLine(0),
  tailLines(0),
  iFirOld(0.0),
  qFirOld(0.0),
  dbgTime(dbgTime)  // Debug printout period (ms)
{
    phasingPos = new int[phasingLines];
    blockSize  = sampleRate * colors * 60 / lpm;

    // Incoming filters setup
    filters[0] = FirFilter(filter);
    filters[1] = FirFilter(filter);
    fstep      = (double)carrier * 2.0 * M_PI / sampleRate;
}

template <typename T>
FaxDecoder<T>::~FaxDecoder() {
    if(phasingPos) { delete[] phasingPos;phasingPos=0; }
    if(buf)        { delete[] buf;buf=0; }

    for(int j=0 ; j<3 ; ++j)
        if(line[j]) { delete[] line[j];line[j]=0; }
}

template <typename T>
bool FaxDecoder<T>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return
        (this->reader->available() >= 2*blockSize-curSize) &&
        (this->writer->writeable() >= 4*lineWidth*colors);
}

template <typename T>
void FaxDecoder<T>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    const T *ptr = this->reader->getReadPointer();
    unsigned int size = this->reader->available();
    unsigned int j, i;

    // If not enough space in the current buffer...
    if(!buf || (curSize+size > maxSize))
    {
        // Create the new buffer, drop out if failed
        unsigned char *newBuf = new unsigned char[curSize+size];
        if(!newBuf) return;
        // Move current data over and delete the old buffer
        if(buf)
        {
            memcpy(newBuf, buf, curSize*sizeof(buf[0]));
            delete[] buf;
        }
        // Now using the new buffer
        buf     = newBuf;
        maxSize = curSize + size;
    }

    // Used to normalize result from radians and add contrast
    double coeff = contrast * 2.0 / M_PI;

    // Demodulate new data into the buffer
    for(j=0 ; (j<size) && (curSize<maxSize) ; ++j)
    {
        // Read incoming data and apply FIR filters
        double in = sample2double(ptr[j]);
        double f  = fstep * j;
        double iFirOut = filters[0].process(in * cos(f));
        double qFirOut = filters[1].process(in * sin(f));

        // Demodulate
        if(am)
        {
            iFirOut /= 3.0;
            qFirOut /= 3.0;

            double mag = sqrt(qFirOut*qFirOut + iFirOut*iFirOut);
            buf[curSize++] = mag<0.0? 0 : mag>255.0? 255 : (int)mag;
        }
        else
        {
            double mag = sqrt(qFirOut*qFirOut + iFirOut*iFirOut);
            iFirOut /= mag;
            qFirOut /= mag;

            if(mag<1.0) buf[curSize++] = 0;
            else
            {
                double x = asin(qFirOld*iFirOut - iFirOld*qFirOut) * coeff;
                buf[curSize++] = x<-1.0? 0 : x>1.0? 255 : (int)((x/2.0+0.5)*255.0);
            }
        }

        // Save previous values
        iFirOld = iFirOut;
        qFirOld = qFirOut;
    }

    // Advance input pointer
    this->reader->advance(j);

    // Must have at least one scanline worth of data
    if(curSize<2*blockSize) return;

    // Detect line type AND sync, if it is an image line
    int curType = decodeLineType(buf, blockSize);
    typeCount = curType==lastType? typeCount + 1 : 0;
    lastType  = curType;

    // Periodically print debug information, if enabled
    unsigned long millis = msecs();
    if(dbgTime && (millis-lastDebugT >= dbgTime))
    {
        lastDebugT = millis;
        printDebug();
    }

    // Depending on the line type...
    switch(curType)
    {
        case TYPE_STOP:
            // Detect stop lines and reset to header search if transmission stopped
            if(typeCount >= stopLength * lpm / 60 - leewayLines)
            {
                finishPage();
                print(" [STOP]");
            }
            // Done with the input data
            skipInput(blockSize);
            return;

        case TYPE_IOC576:
        case TYPE_IOC288:
            // Searching for a leader tone...
            if(typeCount >= startLength * lpm / 60 - leewayLines)
            {
                finishPage();

                // NEXT: Phasing
                ioc       = -curType;
                lineWidth = (int)(ioc * M_PI + 3.5) & ~3;
                curState  = STATE_SYNC;
                curLine   = 0;
                tailLines = 0;

                // Create line buffers
                for(int j=0 ; j<3 ; ++j)
                {
                    if(line[j]) { delete[] line[j];line[j]=0; }
                    line[j] = new unsigned char[lineWidth * colors];
                    memset(line[j], 0xFF, lineWidth * colors);
                }

                print(" [START IOC%d %dx%d]", ioc, lineWidth, maxLines);
            }
            // Done with the input data
            skipInput(blockSize);
            return;
    }

    // Depending on the decoder state...
    switch(curState)
    {
        default:
            // Keep printing tail lines as long as necessary
            if(tailLines) tailLines -= printBmpEmptyLines(tailLines);
            // Skip input data, do not do anything with it
            skipInput(blockSize);
            break;

        case STATE_SYNC:
            // Phasing...
            if(curLine >= phasingLines)
            {
                // This should not happen, but if it does, restart page
                finishPage();
            }
            else
            {
                // Save sync positions after skipping a few lines
                if(curLine >= phasingSkipLines)
                    phasingPos[curLine - phasingSkipLines] =
                        phasingLinePosition(buf, blockSize);

                // Count phasing lines
                if(++curLine >= phasingLines)
                {
                    // Compute skip value based on previous phasing positions
                    phasingSkipData = median(phasingPos, phasingLines - phasingSkipLines);

                    print(" [PHASING-POS %d at %d]", phasingSkipData, curLine);

                    // Skip to the computed phasing position
                    skipInput(phasingSkipData);

                    // NEXT: Receiving image
                    curState  = STATE_IMAGE;
                    curLine   = 0;
                    lastLineT = msecs();
                }
            }

            // Done with the input data
            skipInput(blockSize);
            break;

        case STATE_IMAGE:
            // If synchronizing lines to the black strip...
            if(syncLines)
            {
                // Look for sync
                int syncPos = findSync(buf, blockSize);

                // If detected sync...
                if(syncPos>0)
                {
                    // Skip input to the sync signal, if it moved far enough
                    if(syncPos>blockSize/20) skipInput(syncPos);
                    // Mark the last time sync has been received
                    lastLineT = msecs();
                }
                // ...else, if no sync for 100 lines...
                else if(msecs() > lastLineT + 100 * 60000 / lpm)
                {
                    finishPage();
                    skipInput(curSize);
                    break;
                }
            }

            // Decode actual image line
            int done = decodeImageLine(buf, blockSize, line[2], lineWidth, colors);

            // If decoder had enough input data...
            if(done)
            {
                // Write out BMP header before the first scanline
                if(!curLine++) printBmpHeader();

                // Optionally post-process and print BMP line
                printBmpLine();

                // Rotate scanline buffers
                unsigned char *p = line[0];
                line[0] = line[1];
                line[1] = line[2];
                line[2] = p;

                // Finish page if we hit maxLines
                if(curLine >= maxLines) finishPage();

                // Done with the input data
                skipInput(done);
            }

            // Done
            break;
    }
}

template <typename T>
double FaxDecoder<T>::FirFilter::process(double sample)
{
    // Narrow, middle and wide FIR low pass filters from ACfax
    const double lpfCoeff[3][FIR_LENGTH] =
    {
        { -7,-18,-15, 11, 56,116,177,223,240,223,177,116, 56, 11,-15,-18, -7},
        {  0,-18,-38,-39,  0, 83,191,284,320,284,191, 83,  0,-39,-38,-18,  0},
        {  6, 20,  7,-42,-74,-12,159,353,440,353,159,-12,-74,-42,  7, 20,  6}
    };

    const double *c = lpfCoeff[bandwidth];
    double sum = 0.0;
    int j = current;
    int i;

    // Replace oldest value with the current value
    buffer[j] = sample;

    // Convolution
    for(i=0 ; j<FIR_LENGTH ; ) sum += buffer[j++] * c[i++];
    for(j=0 ; i<FIR_LENGTH ; ) sum += buffer[j++] * c[i++];

    // Point again to oldest value
    current = j>0? j-1 : FIR_LENGTH-1;

    return(sum);
}

template <typename T>
double FaxDecoder<T>::fftSub(const unsigned char *buf, unsigned int size, int freq)
{
     double coeff = -2.0 * M_PI * freq * 60.0 / lpm / size;
     std::complex<double> ret(0.0, 0.0);

     for(unsigned int i=0 ; i<size ; ++i)
          ret += std::complex<double>(buf[i], 0.0)
               * std::exp(std::complex<double>(0.0, coeff * i));

     return(std::abs(ret));
}

template <typename T>
int FaxDecoder<T>::phasingLinePosition(const unsigned char *buf, unsigned int size)
{
    // Use 5% of the line size to detect
    int n = 5 * size / 100;
    int i, j, total, min, minTotal;

    for(i=0, minTotal=-1 ; i<size ; ++i)
    {
        for(j=0, total=0 ; j<n ; ++j)
            total += (n/2 - abs(j - n/2)) * (255 - buf[(i+j) % size]);

        if((total<minTotal) || (minTotal<0))
        {
            minTotal = total;
            min = i;
        }
    }

    return((min + n*3/4) % size);
}

template <typename T>
int FaxDecoder<T>::findSync(const unsigned char *buf, unsigned int size)
{
    int n = sampleRate * 60 / lpm;
    int nSync = 2 * n / 100;
    int nTotal = nSync * 3;
    int minSum, minPos, sum, j;

    // Must have enough samples
    if(nTotal>size) return(-1);

    // Compute initial sum
    for(j=0, sum=0 ; j<nSync ; ++j)
        sum += 2*255 - buf[j] + buf[nSync+j] - buf[2*nSync+j];

    minSum = sum;
    minPos = 0;

    // Slide the window
    for(j=0 ; j<size-nTotal ; ++j)
    {
        // Compute the next sum
        sum += buf[j] - 2*buf[nSync+j] + 2*buf[2*nSync+j] - buf[3*nSync+j];

        // Keep track of the minimum
        if(sum<minSum) { minSum=sum; minPos=j+1; }
    }

//if(curState!=STATE_HEADER)
//print(" [LINE = %d, STATE = %d, POS = %d, SUM = %d => %s]",
//curLine, curState, minPos, 100*minSum/nTotal/255, minSum<0x50*nTotal? "OK":"NO-SYNC");

    // Return found position, or 0 if not clear enough
    return(minSum<0x50*nTotal? minPos : 0);
}

template <typename T>
int FaxDecoder<T>::decodeLineType(const unsigned char *buf, unsigned int size)
{
    // This is pretty arbitrary but works in practice even with lots of noise
    const double threshold = 5.0;
    double v1, v2;
    int type;

    // Detect special line types
    v1 = fftSub(buf, size, FREQ_IOC576);
    v2 = fftSub(buf, size, FREQ_IOC288);
    if(v2 > v1) type = TYPE_IOC288;
    else
    {
        type = TYPE_IOC576;
        v2   = v1;
    }
    v1 = fftSub(buf, size, FREQ_STOP);
    if (v1 > v2)
    {
        type = TYPE_STOP;
        v2   = v1;
    }

    // If we are below threshold, assume a regular image scanline
    return(v2 > threshold * size? type : TYPE_IMAGE);
}


template <typename T>
int FaxDecoder<T>::decodeImageLine(const unsigned char *buf, unsigned int size, unsigned char *image, unsigned int width, unsigned int colors)
{
    int n = sampleRate * 60 / lpm;
    int i, j, c;

    if(size < n * colors) return(0);

    for(i=0 ; i<width ; ++i)
    {
        for(c=0 ; c<colors ; ++c)
        {
            int first = n*c + n*i/width;
            int last  = n*c + n*(i+1)/width;
            int value = 0;

            for(j=first ; j<last ; ++j) value+=buf[j];

            image[i*colors + c] = value / (last-first);
        }
    }

    return(n * colors);
}

template <typename T>
void FaxDecoder<T>::skipInput(unsigned int size)
{
    // Make sure we do not skip more than we have
    size = !buf? 0 : size<curSize? size : curSize;

    // If skipping...
    if(size)
    {
        // Move data
        for(int j=0 ; j<curSize-size ; ++j) buf[j] = buf[j+size];
        curSize -= size;

        // Update time
        curSamples += size;
        if(curSamples>=sampleRate)
        {
            unsigned int secs = curSamples/sampleRate;
            curSeconds += secs;
            curSamples -= secs*sampleRate;
        }
    }
}

template <typename T>
void FaxDecoder<T>::finishPage()
{
    bool finishing = curState==STATE_IMAGE;

    // Complete current image
    if(finishing && (curLine<maxLines))
    {
        tailLines  = maxLines - curLine;
        tailLines -= printBmpEmptyLines(tailLines);
    }

    // Delete current line buffers, if any
    for(int j=0 ; j<3 ; ++j)
        if(line[j]) { delete[] line[j];line[j]=0; }

    // NEXT: Searching for the next header
    curState = STATE_HEADER;
    curLine  = 0;

if(finishing) print(" [FINISH-PAGE]");
}

template <typename T>
void FaxDecoder<T>::printBmpHeader()
{
    unsigned int pal[256];
    BMPHeader bmp;

    // If there is enough output buffer available...
    unsigned int dataOffset = sizeof(bmp) + (colors>1? 0 : sizeof(pal));
    if(this->writer->writeable() >= dataOffset)
    {
        unsigned int imageSize  = lineWidth * colors * maxLines;
        unsigned int fileSize   = imageSize + dataOffset;

        memset(&bmp, 0, sizeof(bmp));

        // Use reserved bytes at offset 6-7 to store IOC/4 and LPM
        bmp.magic[0]      = 'B';
        bmp.magic[1]      = 'M';
        bmp.fileSize[0]   = fileSize & 0xFF;
        bmp.fileSize[1]   = (fileSize >> 8) & 0xFF;
        bmp.fileSize[2]   = (fileSize >> 16) & 0xFF;
        bmp.fileSize[3]   = (fileSize >> 24) & 0xFF;
        bmp.reserved[0]   = ioc / 4; // IOC576 -> 144, IOC288 -> 72
        bmp.reserved[1]   = lpm;
        bmp.dataOffset[0] = dataOffset & 0xFF;
        bmp.dataOffset[1] = (dataOffset >> 8) & 0xFF;

        bmp.dibSize[0]    = 40;
        bmp.width[0]      = lineWidth & 0xFF;
        bmp.width[1]      = (lineWidth >> 8) & 0xFF;
        bmp.width[2]      = (lineWidth >> 16) & 0xFF;
        bmp.width[3]      = (lineWidth >> 24) & 0xFF;
        bmp.height[0]     = (-maxLines) & 0xFF;
        bmp.height[1]     = (-maxLines >> 8) & 0xFF;
        bmp.height[2]     = (-maxLines >> 16) & 0xFF;
        bmp.height[3]     = (-maxLines >> 24) & 0xFF;
        bmp.planes[0]     = 1;
        bmp.bitCount[0]   = colors>1? 24 : 8;
        bmp.imageSize[0]  = imageSize & 0xFF;
        bmp.imageSize[1]  = (imageSize >> 8) & 0xFF;
        bmp.imageSize[2]  = (imageSize >> 16) & 0xFF;
        bmp.imageSize[3]  = (imageSize >> 24) & 0xFF;

        // Place BMP header into the output buffer
        writeData(&bmp, sizeof(bmp));

        // If receiving grayscale fax...
        if(colors==1)
        {
            // Generate grayscale palette
            for(int i=0 ; i<256 ; ++i)
                pal[i] = i | (i<<8) | (i<<16) | 0xFF000000;

            // Place palette into the output buffer
            writeData(pal, sizeof(pal));
        }
    }
}

template <typename T>
void FaxDecoder<T>::printBmpLine()
{
    // If not post-processing, output line as-is and drop out
    if(!post)
    {
        writeData(line[2], lineWidth * colors);
        return;
    }

    unsigned char image[lineWidth * colors];
    int n = (lineWidth-1) * colors;
    int i, j, k;

    // Compute first and last pixels
    for(j=0 ; j<colors ; ++j)
    {
        i = j + colors;
        i = 2*line[0][j] +   line[0][i] +
            8*line[1][j] + 2*line[1][i] +
            2*line[2][j] +   line[2][i];
        image[j] = i >> 4;

        i = n + j - colors;
        k = n + j;
        i = 2*line[0][k] +   line[0][i] +
            8*line[1][k] + 2*line[1][i] +
            2*line[2][k] +   line[2][i];
        image[k] = i >> 4;
    }

    // Apply the filter
    for(j=colors ; j<n ; ++j)
    {
        i = j - colors;
        k = j + colors;
        i = line[0][i] +  2*line[0][j] +   line[0][k] +
          2*line[1][i] + 12*line[1][j] + 2*line[1][k] +
            line[2][i] +  2*line[2][j] +   line[2][k];
        image[j] = i / 24;
    }

    // Output the scanline
    writeData(image, sizeof(image));
}

template <typename T>
unsigned int FaxDecoder<T>::printBmpEmptyLines(unsigned int lines)
{
    // 3x3 = 9 bytes
    static const char endMark[] = { 'E', 'N', 'D', '-', 'P', 'A', 'G', 'E', '!' };

    unsigned int size = lineWidth * colors;
    unsigned int todo = this->writer->writeable() / size;
    char buf[size];

    // Clear line to white
    memset(buf, 0xFF, size);

    // Insert end-mark at the start of each empty line
    memcpy(buf, endMark, sizeof(endMark));

    // Compute the number of lines we can write
    todo = lines<todo? lines : todo;

    // Write empty lines
    for(int i=0 ; i<todo ; ++i)
        if(!writeData(buf, size)) break;

    // Number of lines written
    return(todo);
}

template <typename T>
void FaxDecoder<T>::printDebug()
{
    // TODO: Insert periodic debug printouts here, as needed
    print(" [BUF %d/%d at %dms]", curSize, maxSize, msecs());
}

template <typename T>
void FaxDecoder<T>::print(const char *format, ...)
{
    char buf[256];

    va_list args;
    va_start(args, format);
    vsprintf(buf, format, args);
    printString(buf);
    va_end(args);
}

template <typename T>
void FaxDecoder<T>::printString(const char *buf)
{
#if 0
    // @@@ Enable to dump log into a file
    {
        FILE *F = fopen("/tmp/fax-decoder.log", "a");
        if(F)
        {
            fprintf(F, "%08X: %s\n", (unsigned int)((unsigned long)this),buf);
            fclose(F);
        }
    }
#endif

    // If we are in debug mode, and not outputting an image, print
    if(dbgTime && !tailLines && (curState!=STATE_IMAGE))
        writeData(buf, strlen(buf));
}

template <typename T>
bool FaxDecoder<T>::writeData(const void *buf, unsigned int size)
{
    // Must have enough writeable space
    if(this->writer->writeable()<size) return(false);

    // Write data then advance pointer
    memcpy(this->writer->getWritePointer(), buf, size);
    this->writer->advance(size);

    // Done
    return(true);
}

template <>
inline double FaxDecoder<complex<float>>::sample2double(complex<float> input)
{
    return input.i();
}

template<>
inline double FaxDecoder<float>::sample2double(float input)
{
    return input;
}

namespace Csdr {
    template class FaxDecoder<float>;
}
