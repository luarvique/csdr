#include "exec.hpp"

#include <unistd.h>
#include <sys/wait.h>
#include <thread>
#include <utility>
#include <iostream>
#include <cstring>

using namespace Csdr;

template <typename T, typename U>
ExecModule<T, U>::ExecModule(std::vector<std::string> args, size_t flushSize):
    Module<T, U>(),
    args(std::move(args)),
    flushSize(flushSize)
{
    startChild();
}

template <typename T, typename U>
ExecModule<T, U>::ExecModule(std::vector<std::string> args):
    ExecModule(std::move(args), 0)
{}

template <typename T, typename U>
ExecModule<T, U>::~ExecModule<T, U>() {
    stopChild();
}

template <typename T, typename U>
void ExecModule<T, U>::startChild() {
    std::lock_guard<std::mutex> lock(this->childMutex);
    size_t s = args.size();
    char* c_args[s];
    for (size_t i = 0; i < s; i++) {
        c_args[i] = (char*) args[i].c_str();
    }
    c_args[s] = NULL;

    int readPipes[2];
    pipe(readPipes);
    int writePipes[2];
    pipe(writePipes);

    child_pid = fork();
    int r;
    switch (child_pid) {
        case -1:
            throw std::runtime_error("could not fork");
        case 0:
            // we are the child.
            // set up pipes and exec()
            close(readPipes[0]);
            dup2(readPipes[1], STDOUT_FILENO);
            close(readPipes[1]);

            close(writePipes[1]);
            dup2(writePipes[0], STDIN_FILENO);
            close(writePipes[0]);

            r = execvp(c_args[0], c_args);
            // if we're still here, that means we encountered an error.
            if (r == -1) {
                std::cerr << "could not exec(): " << strerror(errno) << "\n";
            } else {
                std::cerr << "exec() failed for unknown reason (r = " << r << ")\n";
            }
            // gotta get out somehow since we fork()ed
            exit(-1);
        default:
            // we are the parent; pid is the child's PID
            // set up pipes and reader thread
            close(readPipes[1]);
            this->readPipe = readPipes[0];
            close(writePipes[0]);
            this->writePipe = writePipes[1];
            if (this->writer != nullptr) {
                run = true;
                readThread = new std::thread([this] { readLoop(); });
            }
            break;
    }
}

template <typename T, typename U>
void ExecModule<T, U>::stopChild() {
    {
        std::lock_guard<std::mutex> lock(this->childMutex);
        run = false;
        if (child_pid != 0) {
            kill(child_pid, SIGTERM);
            if (flushSize > 0) {
                T toflush[flushSize] = {0};
                write(this->writePipe, &toflush, sizeof(T) * flushSize);
            }
            // this probably has no effect
            closePipes();

            pid_t rc = 0, r = 0;

            // 50 retries with a 100ms delay ~= 5 second timeout
            int retries = 50;
            while (retries--) {
                r = waitpid(child_pid, &rc, WNOHANG);
                if (r == 0) {
                    // 100ms delay
                    struct timespec delay = {0, 100000000}, remaining = {0, 0};
                    nanosleep(&delay, &remaining);
                } else {
                    break;
                }
            }
            if (r == -1) {
                std::cerr << "waitpid failed: " << strerror(errno) << "\n";
            } else if (r == 0) {
                std::cerr << "child failed to terminate within 5 seconds, sending SIGKILL...\n";
                kill(child_pid, SIGKILL);
                r = waitpid(child_pid, &rc, 0);
            }
            if (rc != 0) {
                std::cerr << "child exited with rc = " << rc << "\n";
            }
            child_pid = 0;
        }
    }
    if (readThread != nullptr) {
        readThread->join();
        delete readThread;
        readThread = nullptr;
    }
}

template <typename T, typename U>
void ExecModule<T, U>::readLoop() {
    size_t available;
    size_t read_bytes;
    while (run) {
        available = std::min(this->writer->writeable(), (size_t) 1024) * sizeof(U) - offset;
        read_bytes = read(this->readPipe, ((char*) this->writer->getWritePointer()) + offset, available);
        if (read_bytes <= 0) {
            run = false;
        } else {
            this->writer->advance((offset + read_bytes) / sizeof(U));
            offset = (offset + read_bytes) % sizeof(U);
        }
    }
    closePipes();
}

template <typename T, typename U>
void ExecModule<T, U>::closePipes() {
    if (this->readPipe != -1) {
        close(this->readPipe);
        this->readPipe = -1;
    }
    if (this->writePipe != -1) {
        close(this->writePipe);
        this->writePipe = -1;
    }
}

template <typename T, typename U>
void ExecModule<T, U>::setWriter(Writer<U> *writer) {
    Module<T, U>::setWriter(writer);
    if (writer != nullptr && readThread == nullptr) {
        std::lock_guard<std::mutex> lock(this->childMutex);
        run = true;
        readThread = new std::thread([this] { readLoop(); });
    }
}

template <typename T, typename U>
bool ExecModule<T, U>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return this->writePipe != -1 && this->reader->available() > 0;
}

template <typename T, typename U>
void ExecModule<T, U>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    size_t size = std::min(this->reader->available(), (size_t) 1024);
    write(this->writePipe, this->reader->getReadPointer(), size * sizeof(T));
    this->reader->advance(size);
}

template <typename T, typename U>
void ExecModule<T, U>::reload() {
    if (this->child_pid != 0) {
        kill(this->child_pid, SIGHUP);
    }
}

template <typename T, typename U>
void ExecModule<T, U>::restart() {
    stopChild();
    startChild();
}

namespace Csdr {
    template class ExecModule<complex<short>, short>;
    template class ExecModule<short, short>;
    template class ExecModule<short, unsigned char>;
    template class ExecModule<unsigned char, unsigned char>;
    template class ExecModule<complex<short>, unsigned char>;
    template class ExecModule<complex<unsigned char>, short>;
}
