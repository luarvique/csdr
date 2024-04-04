/*
Copyright (c) 2024 Fernando Sanchez, Jr. <fernando.sanchez.jr@gmail.com>

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

#include <mutex>

#include <fftw3.h>
#include <vulkan/vulkan.h>
#include <vkFFT.h>

namespace Csdr {

    class VkFFTBackend {
        public:
            explicit VkFFTBackend(uint64_t size);
            ~VkFFTBackend();

            VkFFTResult fft(fftwf_complex *input, fftwf_complex *output);

        private:
            // glslang management
            static uint64_t instances;
            static std::mutex setupMtx;

            // Vulkan resources
            VkInstance instance = nullptr;
            VkPhysicalDevice physicalDevice = nullptr;
            uint64_t queueIndex = 0;
            VkDevice device = nullptr;
            VkQueue queue = nullptr;
            VkFence fence = nullptr;
            VkCommandPool commandPool = nullptr;


            // VkFFT resources
            uint64_t bufferSize = 0;

            VkBuffer outputBuffer = nullptr;
            VkDeviceMemory outputBufferMemory = nullptr;

            VkBuffer cpuSourceBuffer = nullptr;
            VkDeviceMemory cpuSourceMemory = nullptr;

            VkBuffer cpuDestBuffer = nullptr;
            VkDeviceMemory cpuDestMemory = nullptr;

            VkFFTConfiguration configuration = {};
            VkFFTApplication   application = {};

            // Backend management
            uint64_t size = 0;
            bool ready;

            // Vulkan initialization & cleanup
            static void initializeGlslang();
            static void cleanupGlslang();

            VkResult createVkInstance();
            void cleanupVkInstance();

            VkResult detectPhysicalDevice();

            VkResult getVkComputeQueueIndex();

            VkResult createVkDevice();
            void cleanupVkDevice();

            VkResult createVkFence();
            void cleanupVkFence();

            VkResult createVkCommandPool();
            void cleanupVkCommandPool();

            VkFFTResult createVkBuffer(
                VkBuffer *buffer,
                VkDeviceMemory *deviceMemory,
                VkBufferUsageFlags usage,
                VkMemoryPropertyFlags properties
            );
            void cleanupVkBuffer(VkBuffer buffer, VkDeviceMemory deviceMemory);

            VkFFTResult findMemoryType(
                uint64_t memoryTypeBits,
                uint64_t memorySize,
                uint32_t* memoryTypeIndex,
                VkMemoryPropertyFlags properties
            );

            // VkFFT initialization & cleanup
            void createConfiguration();

            VkFFTResult createApplication();
            void cleanupApplication();

            VkResult transferFromCPU(void *source);
            VkResult transferToCPU(void *dest);
    };

}
