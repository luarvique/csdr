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

#include "vulkan/vulkan.h"
#include "vkFFT.h"

namespace Csdr {

    class VkFFTBackend {
        public:
            VkFFTBackend(uint64_t size);
            ~VkFFTBackend();

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
            uint64_t bufferSize = 0;
            VkBuffer buffer = nullptr;
            VkDeviceMemory deviceMemory = nullptr;

            // VkFFT resources
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

            VkResult createVkBuffer();
            void cleanupVkBuffer();

            VkFFTResult findMemoryType(uint64_t memoryTypeBits, uint64_t memorySize, uint32_t* memoryTypeIndex);

            VkFFTResult allocateVkMemory();
            void cleanupVkMemory();

            // VkFFT initialization & cleanup
            void createConfiguration();

            VkFFTResult createApplication();
            void cleanupApplication();
    };

}
