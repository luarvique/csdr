#pragma once

#include "module.hpp"

#include <vector>

namespace Csdr {

    class UntypedExecModule {
        public:
            virtual void reload() = 0;
            virtual void restart() = 0;
    };

    template<typename T, typename U>
    class ExecModule: public UntypedExecModule, public Module<T, U> {
        public:
            ExecModule(std::vector<std::string> args, size_t flushSize);
            explicit ExecModule(std::vector<std::string> args);
            ~ExecModule();
            bool canProcess() override;
            void process() override;
            void setWriter(Writer<U>* writer) override;
            void reload() override;
            void restart() override;
        private:
            void startChild();
            void stopChild();
            void readLoop();
            void closePipes();
            bool isPipeWriteable();
            std::vector<std::string> args;
            size_t flushSize = 0;
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