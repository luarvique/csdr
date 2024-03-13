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

#pragma once

#include "module.hpp"

#include <vector>

namespace Csdr {

    class UntypedExecModule {
        public:
            virtual void reload() = 0;
            virtual void restart() = 0;
            virtual void setArgs(const std::vector<std::string>& args) = 0;
    };

    template<typename T, typename U>
    class ExecModule: public UntypedExecModule, public Module<T, U> {
        public:
            explicit ExecModule(std::vector<std::string> args, size_t flushSize = 0, bool doNotKill = false);
            ~ExecModule();
            bool canProcess() override;
            void process() override;
            void setWriter(Writer<U>* writer) override;
            void reload() override;
            void restart() override;
            void setArgs(const std::vector<std::string>& args) override;
        private:
            void startChild();
            void stopChild();
            void readLoop();
            void closePipes();
            bool isPipeWriteable();
            std::vector<std::string> args;
            size_t flushSize = 0;
            bool doNotKill = false;
            std::mutex childMutex;
            pid_t child_pid = 0;
            int readPipe = -1;
            int writePipe = -1;
            std::thread* readThread = nullptr;
            bool run = true;
            int readOffset = 0;
            int writeOffset = 0;
    };

}
