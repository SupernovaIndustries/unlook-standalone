/**
 * @file VulkanSGMAccelerator.cpp
 * @brief Vulkan GPU acceleration for SGM - Implementation
 */

#include "unlook/stereo/VulkanSGMAccelerator.hpp"
#include "unlook/core/Logger.hpp"
#include <fstream>
#include <cstring>
#include <chrono>

namespace unlook {
namespace stereo {

using namespace std::chrono;

VulkanSGMAccelerator::VulkanSGMAccelerator() {
}

VulkanSGMAccelerator::~VulkanSGMAccelerator() {
    cleanup();
}

void VulkanSGMAccelerator::setConfig(const Config& config) {
    config_ = config;
}

VulkanSGMAccelerator::Result VulkanSGMAccelerator::process(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    Result result;

    if (!initialized_) {
        result.success = false;
        result.errorMessage = "VulkanSGMAccelerator not initialized";
        return result;
    }

    auto startTime = high_resolution_clock::now();

    try {
        // For now, fall back to CPU implementation
        // TODO: Implement full GPU pipeline with census transform and SGM

        result.success = false;
        result.errorMessage = "GPU processing not yet fully implemented";

        // Report timing
        result.gpuTime = duration_cast<milliseconds>(high_resolution_clock::now() - startTime);

        // Report memory usage (simplified)
        VkPhysicalDeviceMemoryProperties memProps;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memProps);
        size_t totalMemory = 0;
        for (uint32_t i = 0; i < memProps.memoryHeapCount; ++i) {
            if (memProps.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) {
                totalMemory = memProps.memoryHeaps[i].size;
                break;
            }
        }
        result.memoryUsedMB = totalMemory / (1024 * 1024);

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "GPU processing exception: " + std::string(e.what());
    }

    return result;
}

bool VulkanSGMAccelerator::initialize() {
    if (initialized_) return true;

    core::Logger::getInstance().log(core::LogLevel::INFO,
        "Initializing Vulkan GPU acceleration for SGM...");

    if (!createInstance()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to create Vulkan instance");
        return false;
    }

    if (!selectPhysicalDevice()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to select Vulkan physical device");
        cleanup();
        return false;
    }

    if (!createDevice()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to create Vulkan device");
        cleanup();
        return false;
    }

    if (!createCommandPool()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to create command pool");
        cleanup();
        return false;
    }

    if (!createDescriptorPool()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to create descriptor pool");
        cleanup();
        return false;
    }

    if (!createPipeline()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Failed to create compute pipeline");
        cleanup();
        return false;
    }

    initialized_ = true;
    core::Logger::getInstance().log(core::LogLevel::INFO,
        "Vulkan GPU acceleration initialized successfully");

    return true;
}

bool VulkanSGMAccelerator::createInstance() {
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Unlook 3D Scanner";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "UnlookStereo";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;

    VkResult result = vkCreateInstance(&createInfo, nullptr, &instance_);
    return result == VK_SUCCESS;
}

bool VulkanSGMAccelerator::selectPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr);

    if (deviceCount == 0) {
        return false;
    }

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data());

    // Select first device with compute queue
    for (const auto& device : devices) {
        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

        for (uint32_t i = 0; i < queueFamilyCount; i++) {
            if (queueFamilies[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                physicalDevice_ = device;
                queueFamilyIndex_ = i;

                VkPhysicalDeviceProperties props;
                vkGetPhysicalDeviceProperties(device, &props);
                core::Logger::getInstance().log(core::LogLevel::INFO,
                    "Selected GPU: " + std::string(props.deviceName));

                return true;
            }
        }
    }

    return false;
}

bool VulkanSGMAccelerator::createDevice() {
    float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = queueFamilyIndex_;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &queuePriority;

    VkPhysicalDeviceFeatures deviceFeatures{};

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.pEnabledFeatures = &deviceFeatures;

    VkResult result = vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_);
    if (result != VK_SUCCESS) {
        return false;
    }

    vkGetDeviceQueue(device_, queueFamilyIndex_, 0, &computeQueue_);
    return true;
}

bool VulkanSGMAccelerator::createCommandPool() {
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = queueFamilyIndex_;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    VkResult result = vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_);
    return result == VK_SUCCESS;
}

bool VulkanSGMAccelerator::createDescriptorPool() {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSize.descriptorCount = 2;  // Input + Output buffers

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = 1;

    VkResult result = vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_);
    return result == VK_SUCCESS;
}

bool VulkanSGMAccelerator::createPipeline() {
    // Initialize all handles to NULL for safe cleanup
    descriptorSetLayout_ = VK_NULL_HANDLE;
    pipelineLayout_ = VK_NULL_HANDLE;
    computePipeline_ = VK_NULL_HANDLE;

    // Create descriptor set layout
    VkDescriptorSetLayoutBinding bindings[2] = {};

    // Input cost volume
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    // Output cost volume
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 2;
    layoutInfo.pBindings = bindings;

    if (vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &descriptorSetLayout_) != VK_SUCCESS) {
        return false;
    }

    // Create pipeline layout with push constants
    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(uint32_t) * 3 + sizeof(float) * 2 + sizeof(uint32_t); // width, height, disparities, P1, P2, pathDir

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS) {
        return false;
    }

    // Load compute shader (SPIR-V)
    VkShaderModule shaderModule;
    if (!loadShaderModule("sgm_path_lr.spv", shaderModule)) {
        core::Logger::getInstance().log(core::LogLevel::WARNING,
            "Failed to load compiled SPIR-V shader, GPU acceleration disabled");

        // Cleanup partial Vulkan objects before returning
        if (pipelineLayout_ != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
            pipelineLayout_ = VK_NULL_HANDLE;
        }
        if (descriptorSetLayout_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(device_, descriptorSetLayout_, nullptr);
            descriptorSetLayout_ = VK_NULL_HANDLE;
        }

        return false;
    }

    // Create compute pipeline
    VkPipelineShaderStageCreateInfo shaderStageInfo{};
    shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStageInfo.module = shaderModule;
    shaderStageInfo.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.layout = pipelineLayout_;
    pipelineInfo.stage = shaderStageInfo;

    VkResult result = vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &computePipeline_);

    vkDestroyShaderModule(device_, shaderModule, nullptr);

    return result == VK_SUCCESS;
}

bool VulkanSGMAccelerator::loadShaderModule(const std::string& filename, VkShaderModule& module) {
    // Try multiple possible shader locations
    std::vector<std::string> searchPaths = {
        "build/src/stereo/vulkan/" + filename,
        "src/stereo/vulkan/" + filename,
        "/home/alessandro/unlook-standalone/build/src/stereo/vulkan/" + filename,
        "/home/alessandro/unlook-standalone/src/stereo/vulkan/" + filename
    };

    std::ifstream file;
    std::string successPath;

    for (const auto& path : searchPaths) {
        file.open(path, std::ios::ate | std::ios::binary);
        if (file.is_open()) {
            successPath = path;
            core::Logger::getInstance().log(core::LogLevel::INFO,
                "Found SPIR-V shader at: " + path);
            break;
        }
    }

    if (!file.is_open()) {
        core::Logger::getInstance().log(core::LogLevel::ERROR,
            "Could not find SPIR-V shader " + filename + " in any search path");
        return false;
    }

    size_t fileSize = (size_t)file.tellg();
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();

    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = buffer.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(buffer.data());

    return vkCreateShaderModule(device_, &createInfo, nullptr, &module) == VK_SUCCESS;
}

bool VulkanSGMAccelerator::aggregateSGM(const cv::Mat& costVolume,
                                         cv::Mat& aggregatedCost,
                                         float P1, float P2) {
    if (!initialized_) {
        return false;
    }

    auto startTotal = high_resolution_clock::now();

    const uint32_t height = costVolume.size[0];
    const uint32_t width = costVolume.size[1];
    const uint32_t disparities = costVolume.size[2];

    // Create or recreate buffers if size changed
    if (width != width_ || height != height_ || disparities != disparities_) {
        destroyBuffers();
        if (!createBuffers(width, height, disparities)) {
            return false;
        }
    }

    // Upload cost volume to GPU
    auto startUpload = high_resolution_clock::now();
    void* data;
    vkMapMemory(device_, costInputMemory_, 0, width * height * disparities * sizeof(float), 0, &data);
    memcpy(data, costVolume.data, width * height * disparities * sizeof(float));
    vkUnmapMemory(device_, costInputMemory_);
    auto endUpload = high_resolution_clock::now();

    // TODO: Execute compute shader
    // For now, return false to fall back to CPU

    lastStats_.uploadTimeMs = duration_cast<microseconds>(endUpload - startUpload).count() / 1000.0;
    lastStats_.computeTimeMs = 0.0;
    lastStats_.downloadTimeMs = 0.0;
    lastStats_.totalTimeMs = duration_cast<microseconds>(high_resolution_clock::now() - startTotal).count() / 1000.0;

    return false;  // Fall back to CPU for now
}

bool VulkanSGMAccelerator::createBuffers(uint32_t width, uint32_t height, uint32_t disparities) {
    width_ = width;
    height_ = height;
    disparities_ = disparities;

    VkDeviceSize bufferSize = width * height * disparities * sizeof(float);

    // Create input buffer
    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = bufferSize;
    bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device_, &bufferInfo, nullptr, &costInputBuffer_) != VK_SUCCESS) {
        return false;
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(device_, costInputBuffer_, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    if (vkAllocateMemory(device_, &allocInfo, nullptr, &costInputMemory_) != VK_SUCCESS) {
        return false;
    }

    vkBindBufferMemory(device_, costInputBuffer_, costInputMemory_, 0);

    // Create output buffer
    if (vkCreateBuffer(device_, &bufferInfo, nullptr, &costOutputBuffer_) != VK_SUCCESS) {
        return false;
    }

    if (vkAllocateMemory(device_, &allocInfo, nullptr, &costOutputMemory_) != VK_SUCCESS) {
        return false;
    }

    vkBindBufferMemory(device_, costOutputBuffer_, costOutputMemory_, 0);

    return true;
}

void VulkanSGMAccelerator::destroyBuffers() {
    if (costInputBuffer_ != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, costInputBuffer_, nullptr);
        costInputBuffer_ = VK_NULL_HANDLE;
    }
    if (costInputMemory_ != VK_NULL_HANDLE) {
        vkFreeMemory(device_, costInputMemory_, nullptr);
        costInputMemory_ = VK_NULL_HANDLE;
    }
    if (costOutputBuffer_ != VK_NULL_HANDLE) {
        vkDestroyBuffer(device_, costOutputBuffer_, nullptr);
        costOutputBuffer_ = VK_NULL_HANDLE;
    }
    if (costOutputMemory_ != VK_NULL_HANDLE) {
        vkFreeMemory(device_, costOutputMemory_, nullptr);
        costOutputMemory_ = VK_NULL_HANDLE;
    }
}

uint32_t VulkanSGMAccelerator::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }

    return 0;
}

void VulkanSGMAccelerator::cleanup() {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);

        destroyBuffers();

        if (computePipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, computePipeline_, nullptr);
        }
        if (pipelineLayout_ != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
        }
        if (descriptorSetLayout_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(device_, descriptorSetLayout_, nullptr);
        }
        if (descriptorPool_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
        }
        if (commandPool_ != VK_NULL_HANDLE) {
            vkDestroyCommandPool(device_, commandPool_, nullptr);
        }

        vkDestroyDevice(device_, nullptr);
    }

    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
    }

    initialized_ = false;
}

} // namespace stereo
} // namespace unlook
