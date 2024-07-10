/*
Copyright (c) 2023 Jakob Ketterl <jakob.ketterl@gmx.de>

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

#include "exec.hpp"

#include <unistd.h>
#include <sys/wait.h>
#include <thread>
#include <utility>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <poll.h>

using namespace Csdr;

template <typename T, typename U>
ExecModule<T, U>::ExecModule(std::vector<std::string> args, size_t flushSize, bool doNotKill):
    Module<T, U>(),
    args(std::move(args)),
    flushSize(flushSize),
    doNotKill(doNotKill)
{
    startChild();
}

template <typename T, typename U>
ExecModule<T, U>::~ExecModule<T, U>() {
    stopChild();
}

template <typename T, typename U>
void ExecModule<T, U>::setArgs(const std::vector<std::string>& new_args) {
    this->args = new_args;
}

template <typename T, typename U>
void ExecModule<T, U>::startChild() {
    std::lock_guard<std::mutex> lock(this->childMutex);

    if (child_pid != 0) {
        throw std::runtime_error("ExecModule child is already running");
    }

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
                std::cerr << "ExecModule: could not exec(): " << strerror(errno) << "\n";
            } else {
                std::cerr << "ExecModule: exec() failed for unknown reason (r = " << r << ")\n";
            }
            // gotta get out somehow since we fork()ed
            exit(-1);
        default:
            // we are the parent; pid is the child's PID
            // set up pipes and reader thread
            close(readPipes[1]);
            r = fcntl(readPipes[0], F_SETFL, fcntl(readPipes[0], F_GETFL) | O_NONBLOCK);
            if (r == -1) {
                std::cerr << "ExecModule: failed to set pipe to non-blocking: " << strerror(errno) << "\n";
            }
            this->readPipe = readPipes[0];

            close(writePipes[0]);
            r = fcntl(writePipes[1], F_SETFL, fcntl(writePipes[1], F_GETFL) | O_NONBLOCK);
            if (r == -1) {
                std::cerr << "ExecModule: failed to set pipe to non-blocking: " << strerror(errno) << "\n";
            }
            this->writePipe = writePipes[1];

            if (this->writer != nullptr) {
                if (readThread != nullptr) {
                    throw std::runtime_error("ExecModule reader thread  is already running");
                }
                run = true;
                readThread = new std::thread([this] { readLoop(); });
            }
            break;
    }
}

template <typename T, typename U>
void ExecModule<T, U>::stopChild() {
    std::lock_guard<std::mutex> lock(this->childMutex);
    run = false;
    if (child_pid != 0) {
        kill(child_pid, SIGTERM);
        if (flushSize > 0) {
            fcntl(this->writePipe, F_SETFL, fcntl(this->writePipe, F_GETFL) & ~O_NONBLOCK);
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
            std::cerr << "ExecModule: waitpid failed: " << strerror(errno) << "\n";
        } else if (r == 0 && !doNotKill) {
            std::cerr << "ExecModule: child failed to terminate within 5 seconds, sending SIGKILL...\n";
            kill(child_pid, SIGKILL);
            r = waitpid(child_pid, &rc, 0);
        }
        if (rc != 0) {
            std::cerr << "ExecModule: child exited with rc = " << rc << "\n";
        }
        child_pid = 0;
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
    ssize_t read_bytes;

    while (run) {
        pollfd pfd {
            .fd = this->readPipe,
            .events = POLLIN
        };
        int rc = poll(&pfd, 1, 10000);
        if (rc == -1) {
            std::cerr << "ExecModule: poll() failed: " << strerror(errno) << "\n";
            return;
        }
        if (pfd.revents & POLLERR) {
            std::cerr << "ExecModule: read pipe indicates error. Stopping readLoop\n";
            break;
        }
        if (run && pfd.revents & POLLIN) {
            std::lock_guard<std::mutex> lock(this->processMutex);
            available = this->writer->writeable();
            if (available == 0) {
                std::cerr << "ExecModule: writer cannot accept data. Stopping readLoop";
                run = false;
            } else {
                available = std::min(available, (size_t) 1024) * sizeof(U) - readOffset;
                read_bytes = read(this->readPipe, ((char*) this->writer->getWritePointer()) + readOffset, available);
                if (read_bytes <= 0) {
                    if (errno != EAGAIN) {
                        run = false;
                    }
                } else {
                    this->writer->advance((readOffset + read_bytes) / sizeof(U));
                    readOffset = (readOffset + read_bytes) % sizeof(U);
                }
            }
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
    std::lock_guard<std::mutex> p_lock(this->processMutex);
    std::lock_guard<std::mutex> c_lock(this->childMutex);
    if (this->writer != nullptr && readThread == nullptr) {
        run = true;
        readThread = new std::thread([this] { readLoop(); });
    }
}

template <typename T, typename U>
bool ExecModule<T, U>::canProcess() {
    std::lock_guard<std::mutex> lock(this->processMutex);
    return this->reader->available() > 0 && this->isPipeWriteable();
}

template <typename T, typename U>
void ExecModule<T, U>::process() {
    std::lock_guard<std::mutex> lock(this->processMutex);

    size_t available = this->reader->available();
    if (available == 0) return;

    size_t size = std::min(available, (size_t) 1024) * sizeof(T) - writeOffset;
    ssize_t written = write(this->writePipe, ((char*) this->reader->getReadPointer()) + writeOffset, size);
    if (written == -1) {
        // EAGAIN may happen since writePipe is non-blocking.
        if (errno == EAGAIN) return;
        std::cerr << "ExecModule: error writing data to child pipe: " << strerror(errno) << "\n";
        return;
    }
    this->reader->advance((writeOffset + written) / sizeof(T));
    writeOffset = (writeOffset + written) % sizeof(T);
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

template <typename T, typename U>
bool ExecModule<T, U>::isPipeWriteable() {
    if (this->child_pid == 0 || this->writePipe == -1) {
        return false;
    }
    pollfd pfd = {
        .fd = this->writePipe,
        .events = POLLOUT
    };
    // we want an immediate result, so set time to 0
    // if the pipe is not writeable now, we'll come back once the next chunk of data becomes available
    // in the meantime, there is probably plenty of data in the pipe for the child to consume
    int rc = poll(&pfd, 1, 0);
    if (rc == -1) {
        std::cerr << "ExecModule: poll() failed: " << strerror(errno) << "\n";
    }
    if (pfd.revents & POLLERR) {
        std::cerr << "ExecModule: write pipe indicated error. Shutting down child\n";
        stopChild();
        return false;
    }
    return pfd.revents & POLLOUT;
}

namespace Csdr {
    template class ExecModule<complex<short>, short>;
    template class ExecModule<short, short>;
    template class ExecModule<short, unsigned char>;
    template class ExecModule<unsigned char, unsigned char>;
    template class ExecModule<unsigned char, float>;
    template class ExecModule<unsigned char, short>;
    template class ExecModule<complex<short>, unsigned char>;
    template class ExecModule<complex<float>, unsigned char>;
    template class ExecModule<complex<unsigned char>, short>;
}
