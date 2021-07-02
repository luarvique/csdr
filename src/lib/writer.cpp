#include "writer.hpp"

#include <iostream>

using namespace Csdr;

template <typename T>
StdoutWriter<T>::StdoutWriter(size_t buffer_size): buffer_size(buffer_size) {
    buffer = (T*) malloc(sizeof(T) * buffer_size);
}

template <typename T>
StdoutWriter<T>::StdoutWriter(): StdoutWriter(10240) {}

template <typename T>
StdoutWriter<T>::~StdoutWriter() {
    free(buffer);
}

template <typename T>
size_t StdoutWriter<T>::writeable() {
    return buffer_size;
}

template <typename T>
T* StdoutWriter<T>::getWritePointer() {
    return buffer;
}

template <typename T>
void StdoutWriter<T>::advance(size_t how_much) {
    std::cout.write((const char*) buffer, sizeof(T) * how_much);
    std::cout.flush();
}