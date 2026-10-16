// Exercise real module buffers with deliberately small read/write spans.
#include "fskuart.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <vector>

class Input: public Csdr::Reader<float> {
    public:
        Input(std::vector<float>& samples, size_t chunk): samples(samples), chunk(chunk) {}
        size_t available() override { return std::min(chunk, samples.size() - position); }
        float* getReadPointer() override { return samples.data() + position; }
        void advance(size_t count) override {
            if (count > available()) throw std::runtime_error("input overrun");
            position += count;
        }
        void wait() override {}
        void unblock() override {}
    private:
        std::vector<float>& samples;
        size_t chunk, position = 0;
};

class Output: public Csdr::Writer<unsigned char> {
    public:
        explicit Output(size_t capacity): capacity(capacity), bytes(capacity + 16, 0xcc) {}
        bool blocked = true;
        size_t writeable() override { return blocked ? 0 : capacity; }
        unsigned char* getWritePointer() override { return bytes.data(); }
        void advance(size_t count) override {
            if (count > writeable()) throw std::runtime_error("output overrun");
            for (size_t i = capacity; i < bytes.size(); ++i)
                if (bytes[i] != 0xcc) throw std::runtime_error("output guard overwritten");
            if (std::fwrite(bytes.data(), 1, count, stdout) != count)
                throw std::runtime_error("output write failed");
        }
    private:
        size_t capacity;
        std::vector<unsigned char> bytes;
};

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "invalid") {
            unsigned int rejected = 0;
            for (double baud: {0.0, -1.0, 12001.0, 1e-10,
                    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
                try { Csdr::FskUartDecoder decoder(12000, baud); }
                catch (const std::invalid_argument&) { ++rejected; }
            }
            return rejected == 6 ? 0 : 1;
        }
        if (argc != 4) return 2;
        const size_t chunk = std::stoul(argv[1]), capacity = std::stoul(argv[2]);
        const unsigned int rate = std::stoul(argv[3]);
        if (!chunk || !capacity) return 2;
        std::vector<float> audio;
        float block[4096];
        size_t count;
        while ((count = std::fread(block, sizeof(float), 4096, stdin)))
            audio.insert(audio.end(), block, block + count);
        Input reader(audio, chunk);
        Output writer(capacity);
        Csdr::FskUartDecoder decoder(rate);
        decoder.setReader(&reader);
        decoder.setWriter(&writer);
        if (decoder.canProcess()) throw std::runtime_error("processed blocked output");
        decoder.process();
        writer.blocked = false;
        while (decoder.canProcess()) decoder.process();
        if (reader.available()) throw std::runtime_error("input left unread");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
