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

#include "vkfft.hpp"

#include <iostream>
#include <mutex>

#include <glslang_c_interface.h>
#include <vulkan/vulkan.h>

using namespace Csdr;

uint64_t VkFFTBackend::instances = 0;
std::mutex VkFFTBackend::setupMtx;

VkFFTBackend::VkFFTBackend(uint64_t size) {
    Csdr::VkFFTBackend::initializeGlslang();

    this->size = size;

    VkResult res = this->createVkInstance();
    if (res != VK_SUCCESS) {
        std::cerr << "instance creation error " << res << "\n";

        return;
    }

    res = this->detectPhysicalDevice();
    if (res != VK_SUCCESS) {
        std::cerr << "device detection error " << res << "\n";

        return;
    }

    res = this->createVkDevice();
    if (res != VK_SUCCESS) {
        std::cerr << "device queue creation error " << res << "\n";

        return;
    }

    res = this->createVkFence();
    if (res != VK_SUCCESS) {
        std::cerr << "fence creation error " << res << "\n";

        return;
    }

    res = this->createVkCommandPool();
    if (res != VK_SUCCESS) {
        std::cerr << "command pool creation error " << res << "\n";

        return;
    }

    VkFFTResult resFFT = this->createVkBuffer(
        &this->outputBuffer,
        &this->outputBufferMemory,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_HEAP_DEVICE_LOCAL_BIT
    );
    if (resFFT != VKFFT_SUCCESS) {
        std::cerr << "output buffer creation error " << resFFT << "\n";

        return;
    }

    resFFT = this->createVkBuffer(
        &this->cpuSourceBuffer,
        &this->cpuSourceMemory,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
    );
    if (resFFT != VKFFT_SUCCESS) {
        std::cerr << "cpu source buffer creation error " << resFFT << "\n";

        return;
    }

    resFFT = this->createVkBuffer(
            &this->cpuDestBuffer,
            &this->cpuDestMemory,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
    );
    if (resFFT != VKFFT_SUCCESS) {
        std::cerr << "cpu source buffer creation error " << resFFT << "\n";

        return;
    }

    this->createConfiguration();

    resFFT = this->createApplication();
    if (resFFT != VKFFT_SUCCESS) {
        std::cerr << "application creation error " << resFFT << "\n";

        return;
    }

    this->ready = true;
}

VkFFTBackend::~VkFFTBackend() {
    this->ready = false;

    this->cleanupVkBuffer(this->outputBuffer, this->outputBufferMemory);
    this->cleanupVkBuffer(this->cpuSourceBuffer, this->cpuSourceMemory);
    this->cleanupVkBuffer(this->cpuDestBuffer, this->cpuDestMemory);
    this->cleanupApplication();
    this->cleanupVkCommandPool();
    this->cleanupVkFence();
    this->cleanupVkDevice();
    this->cleanupVkInstance();
    Csdr::VkFFTBackend::cleanupGlslang();
}

void VkFFTBackend::initializeGlslang() {
    // initialize glslang if no instances exist
    Csdr::VkFFTBackend::setupMtx.lock();
    if (Csdr::VkFFTBackend::instances == 0) {
        glslang_initialize_process();
    }
    Csdr::VkFFTBackend::instances++;
    Csdr::VkFFTBackend::setupMtx.unlock();
}

void VkFFTBackend::cleanupGlslang() {
    // clean up glslang after last instance deleted
    Csdr::VkFFTBackend::setupMtx.lock();
    Csdr::VkFFTBackend::instances--;
    if (Csdr::VkFFTBackend::instances == 0) {
        glslang_finalize_process();
    }
    Csdr::VkFFTBackend::setupMtx.unlock();
}

VkResult VkFFTBackend::createVkInstance() {
    VkApplicationInfo applicationInfo = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
    applicationInfo.pApplicationName = "csdr++";
    applicationInfo.applicationVersion = (uint32_t) 1821;
    applicationInfo.pEngineName = "vkfft";
    applicationInfo.engineVersion = 1;
    applicationInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    createInfo.flags = 0;
    createInfo.pApplicationInfo = &applicationInfo;
    createInfo.enabledExtensionCount = 0;
    createInfo.enabledLayerCount = 0;
    createInfo.pNext = nullptr;

    return vkCreateInstance(&createInfo, nullptr, &this->instance);
}

void VkFFTBackend::cleanupVkInstance() {
    if (this->instance != nullptr) {
        vkDestroyInstance(this->instance, nullptr);
    }
}

VkResult VkFFTBackend::detectPhysicalDevice() {
    uint32_t deviceCount;
    VkResult res = vkEnumeratePhysicalDevices(this->instance, &deviceCount, nullptr);
    if (res != VK_SUCCESS) {
        return res;
    }
    if (deviceCount == 0) {
        return VK_ERROR_DEVICE_LOST;
    }

    // free devices before return
    auto *devices = (VkPhysicalDevice *) malloc(sizeof(VkPhysicalDevice) * deviceCount);
    if (!devices) {
        // malloc'ed magical null pointer of destiny
        return VK_INCOMPLETE;
    }

    res = vkEnumeratePhysicalDevices(this->instance, &deviceCount, devices);
    if (res != VK_SUCCESS) {
        free(devices);
        return res;
    }

    VkPhysicalDeviceProperties properties;
    for (int i = 0; i < deviceCount; i++) {
        vkGetPhysicalDeviceProperties(devices[i], &properties);

        if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
            this->physicalDevice = devices[i];

            free(devices);
            return VK_SUCCESS;
        }
    }

    free(devices);
    return VK_INCOMPLETE;
}

VkResult VkFFTBackend::getVkComputeQueueIndex() {
    uint32_t queueFamilyCount;
    vkGetPhysicalDeviceQueueFamilyProperties(this->physicalDevice, &queueFamilyCount, nullptr);

    if (queueFamilyCount == 0) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }

    // free queueFamilies before return
    auto *queueFamilies = (VkQueueFamilyProperties *) malloc(sizeof(VkQueueFamilyProperties) * queueFamilyCount);
    if (!queueFamilies) {
        // nullptr means today is a special day
        return VK_INCOMPLETE;
    }

    vkGetPhysicalDeviceQueueFamilyProperties(this->physicalDevice, &queueFamilyCount, queueFamilies);

    uint64_t i = 0;
    for (; i < queueFamilyCount; i++) {
        VkQueueFamilyProperties props = queueFamilies[i];

        if (props.queueCount > 0 && (props.queueFlags & VK_QUEUE_COMPUTE_BIT)) {
            this->queueIndex = i;

            free(queueFamilies);
            return VK_SUCCESS;
        }
    }

    free(queueFamilies);
    return VK_ERROR_INITIALIZATION_FAILED;
}

VkResult VkFFTBackend::createVkDevice() {
    VkResult res = this->getVkComputeQueueIndex();
    if (res != VK_SUCCESS) {
        return res;
    }

    float queuePriorities = 1.0;
    VkDeviceQueueCreateInfo queueCreateInfo = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueCreateInfo.queueFamilyIndex = (uint32_t) this->queueIndex;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &queuePriorities;

    VkPhysicalDeviceFeatures deviceFeatures = {};

    VkDeviceCreateInfo deviceCreateInfo = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    deviceCreateInfo.pQueueCreateInfos = &queueCreateInfo;
    deviceCreateInfo.queueCreateInfoCount = 1;
    deviceCreateInfo.pEnabledFeatures = nullptr;
    deviceCreateInfo.pEnabledFeatures = &deviceFeatures;

    res = vkCreateDevice(this->physicalDevice, &deviceCreateInfo, nullptr, &this->device);
    if (res != VK_SUCCESS) {
        return res;

    }

    vkGetDeviceQueue(this->device, (uint32_t) this->queueIndex, 0, &this->queue);

    return VK_SUCCESS;
}

void VkFFTBackend::cleanupVkDevice() {
    if (this->device != nullptr) {
        vkDestroyDevice(this->device, nullptr);
    }
}

VkResult VkFFTBackend::createVkFence() {
    VkFenceCreateInfo fenceCreateInfo = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceCreateInfo.flags = 0;

    return vkCreateFence(this->device, &fenceCreateInfo, nullptr, &this->fence);
}

void VkFFTBackend::cleanupVkFence() {
    if (this->fence != nullptr) {
        vkDestroyFence(this->device, this->fence, nullptr);
    }
}

VkResult VkFFTBackend::createVkCommandPool() {
    VkCommandPoolCreateInfo commandPoolCreateInfo = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    commandPoolCreateInfo.queueFamilyIndex = (uint32_t) queueIndex;

    return vkCreateCommandPool(this->device, &commandPoolCreateInfo, nullptr, &this->commandPool);
}

void VkFFTBackend::cleanupVkCommandPool() {
    if (this->commandPool != nullptr) {
        vkDestroyCommandPool(this->device, this->commandPool, nullptr);
    }
}

VkFFTResult VkFFTBackend::findMemoryType(
    uint64_t memoryTypeBits,
    uint64_t memorySize,
    uint32_t *memoryTypeIndex,
    VkMemoryPropertyFlags properties
) {
    VkPhysicalDeviceMemoryProperties memoryProperties = {0};

    vkGetPhysicalDeviceMemoryProperties(this->physicalDevice, &memoryProperties);

    for (uint64_t i = 0; i < memoryProperties.memoryTypeCount; ++i) {
        if ((memoryTypeBits & ((uint64_t) 1 << i)) &&
            ((memoryProperties.memoryTypes[i].propertyFlags & properties) == properties) &&
            (memoryProperties.memoryHeaps[memoryProperties.memoryTypes[i].heapIndex].size >= memorySize)) {
            memoryTypeIndex[0] = (uint32_t) i;
            return VKFFT_SUCCESS;
        }
    }
    return VKFFT_ERROR_FAILED_TO_FIND_MEMORY;
}

VkFFTResult VkFFTBackend::createVkBuffer(
    VkBuffer *buffer,
    VkDeviceMemory *deviceMemory,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties
) {
    this->bufferSize = sizeof(float) * 2 * this->size;
    this->bufferSize += this->bufferSize % 4;
    uint32_t queueFamilyIndices;
    VkBufferCreateInfo bufferCreateInfo = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    bufferCreateInfo.queueFamilyIndexCount = 1;
    bufferCreateInfo.pQueueFamilyIndices = &queueFamilyIndices;
    bufferCreateInfo.size = this->bufferSize;
    bufferCreateInfo.usage = usage;
    VkResult res = vkCreateBuffer(this->device, &bufferCreateInfo, nullptr, buffer);
    if (res != VK_SUCCESS) {
        return VKFFT_ERROR_FAILED_TO_ALLOCATE_MEMORY;
    }

    VkMemoryRequirements memoryRequirements = {0};
    vkGetBufferMemoryRequirements(this->device, *buffer, &memoryRequirements);
    VkMemoryAllocateInfo memoryAllocateInfo = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    memoryAllocateInfo.allocationSize = memoryRequirements.size;

    VkFFTResult resFFT = findMemoryType(
            memoryRequirements.memoryTypeBits,
            memoryRequirements.size,
            &memoryAllocateInfo.memoryTypeIndex,
            properties
    );
    if (resFFT != VKFFT_SUCCESS) {
        return resFFT;
    }
    res = vkAllocateMemory(this->device, &memoryAllocateInfo, nullptr, deviceMemory);
    if (res != VK_SUCCESS) {
        return VKFFT_ERROR_FAILED_TO_ALLOCATE_MEMORY;
    }
    res = vkBindBufferMemory(this->device, *buffer, *deviceMemory, 0);
    if (res != VK_SUCCESS) {
        return VKFFT_ERROR_FAILED_TO_BIND_BUFFER_MEMORY;
    }

    return VKFFT_SUCCESS;
}

void VkFFTBackend::cleanupVkBuffer(VkBuffer buffer, VkDeviceMemory deviceMemory) {
    if (buffer != nullptr) {
        vkDestroyBuffer(this->device, buffer, nullptr);
    }
    if (deviceMemory != nullptr) {
        vkFreeMemory(this->device, deviceMemory, nullptr);
    }
}

void VkFFTBackend::createConfiguration() {
    this->configuration.FFTdim = 1;
    this->configuration.size[0] = this->size;
    this->configuration.device = &this->device;
    this->configuration.queue = &this->queue;
    this->configuration.fence = &this->fence;
    this->configuration.commandPool = &this->commandPool;
    this->configuration.physicalDevice = &this->physicalDevice;
    this->configuration.isCompilerInitialized = 1;
    this->configuration.buffer = &this->outputBuffer;
    this->configuration.bufferSize = &this->bufferSize;
    this->configuration.outputBuffer = &this->outputBuffer;
    this->configuration.outputBufferSize = &this->bufferSize;
    this->configuration.makeForwardPlanOnly = 1;
}

VkFFTResult VkFFTBackend::createApplication() {
    return initializeVkFFT(&this->application, this->configuration);
}

void VkFFTBackend::cleanupApplication() {

    deleteVkFFT(&this->application);
}

VkFFTResult VkFFTBackend::fft(fftwf_complex *input, fftwf_complex *output) {
    VkResult res = this->transferFromCPU(input);
    if (res != VK_SUCCESS) {
        // any errors mark this backend as not ready
        this->ready = false;
        std::cerr << "transfer from cpu source buffer error " << res << "\n";

        return VKFFT_ERROR_FAILED_TO_COPY;
    }


    res = this->transferToCPU(output);
    if (res != VK_SUCCESS) {
        // any errors mark this backend as not ready
        this->ready = false;
        std::cerr << "transfer to cpu source buffer error " << res << "\n";

        return VKFFT_ERROR_FAILED_TO_COPY;
    }

    return VKFFT_SUCCESS;
};

VkResult VkFFTBackend::transferFromCPU(void *source) {
    // copy data to memory mapped to cpu source buffer
    void* data;
    VkResult res = vkMapMemory(this->device, this->cpuSourceMemory, 0, this->bufferSize, 0, &data);
    if (res != VK_SUCCESS) {
        return res;
    }
    memcpy(data, source, this->bufferSize);
    vkUnmapMemory(this->device, this->cpuSourceMemory);

    // copy from cpu source buffer to output buffer - fft will be performed in-place
    VkCommandBufferAllocateInfo commandBufferAllocateInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    commandBufferAllocateInfo.commandPool = this->commandPool;
    commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandBufferAllocateInfo.commandBufferCount = 1;
    VkCommandBuffer commandBuffer = {};
    res = vkAllocateCommandBuffers(this->device, &commandBufferAllocateInfo, &commandBuffer);
    if (res != VK_SUCCESS) {
        return res;
    }
    VkCommandBufferBeginInfo commandBufferBeginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    res = vkBeginCommandBuffer(commandBuffer, &commandBufferBeginInfo);
    if (res != VK_SUCCESS) {
        return res;
    }
    VkBufferCopy copyRegion = {};
    copyRegion.srcOffset = 0;
    copyRegion.dstOffset = 0;
    copyRegion.size = this->bufferSize;
    vkCmdCopyBuffer(commandBuffer, this->cpuSourceBuffer, this->outputBuffer, 1, &copyRegion);
    res = vkEndCommandBuffer(commandBuffer);
    if (res != VK_SUCCESS) {
        return res;
    }
    VkSubmitInfo submitInfo = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    res = vkQueueSubmit(this->queue, 1, &submitInfo, this->fence);
    if (res != VK_SUCCESS) {
        return res;
    }
    res = vkWaitForFences(this->device, 1, &this->fence, VK_TRUE, 100000000000);
    if (res != VK_SUCCESS) {
        return res;
    }
    res = vkResetFences(this->device, 1, &this->fence);
    if (res != VK_SUCCESS) {
        return res;
    }
    vkFreeCommandBuffers(this->device, this->commandPool, 1, &commandBuffer);

    return VK_SUCCESS;
}

VkResult VkFFTBackend::transferToCPU(void *dest) {
    // copy from output buffer to cpu destination buffer
    VkCommandBufferAllocateInfo commandBufferAllocateInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    commandBufferAllocateInfo.commandPool = this->commandPool;
    commandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandBufferAllocateInfo.commandBufferCount = 1;
    VkCommandBuffer commandBuffer = {};
    VkResult res = vkAllocateCommandBuffers(this->device, &commandBufferAllocateInfo, &commandBuffer);
    if (res != VK_SUCCESS) {
        return res;
    }
    VkCommandBufferBeginInfo commandBufferBeginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    commandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    res = vkBeginCommandBuffer(commandBuffer, &commandBufferBeginInfo);
    if (res != VK_SUCCESS) {
        return res;
    }
    VkBufferCopy copyRegion = {};
    copyRegion.srcOffset = 0;
    copyRegion.dstOffset = 0;
    copyRegion.size = this->bufferSize;
    vkCmdCopyBuffer(commandBuffer, this->outputBuffer, this->cpuDestBuffer, 1, &copyRegion);
    vkEndCommandBuffer(commandBuffer);
    VkSubmitInfo submitInfo = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    res = vkQueueSubmit(this->queue, 1, &submitInfo, this->fence);
    if (res != VK_SUCCESS) {
        return res;
    }
    res = vkWaitForFences(this->device, 1, &this->fence, VK_TRUE, 100000000000);
    if (res != VK_SUCCESS) {
        return res;
    }
    res = vkResetFences(this->device, 1, &this->fence);
    if (res != VK_SUCCESS) {
        return res;
    }
    vkFreeCommandBuffers(this->device, this->commandPool, 1, &commandBuffer);

    void* data;
    res = vkMapMemory(this->device, this->cpuDestMemory, 0, this->bufferSize, 0, &data);
    if (res != VK_SUCCESS) {
        return res;
    }
    memcpy(dest, data, this->bufferSize);
    vkUnmapMemory(this->device, this->cpuDestMemory);

    return VK_SUCCESS;
}
