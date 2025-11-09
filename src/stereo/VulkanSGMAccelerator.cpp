/**
 * @file VulkanSGMAccelerator.cpp
 * @brief Vulkan GPU acceleration for SGM - Implementation
 */

#include "unlook/stereo/VulkanSGMAccelerator.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <cstring>
#include <chrono>
#include <cfloat>

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
        // TEMPORARY: GPU SGM not yet fully working, fall back to CPU SGBM
        // The Census+Cost volume computation is too slow on CPU
        // TODO: Implement census+cost on GPU compute shader

        result.success = false;
        result.errorMessage = "GPU processing not yet fully optimized - use AD-Census CPU instead";
        result.gpuTime = duration_cast<milliseconds>(high_resolution_clock::now() - startTime);

        // Report memory usage
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
                                         int numDisparities,
                                         float P1, float P2) {
    if (!initialized_) {
        core::Logger::getInstance().log(core::LogLevel::ERROR, "VulkanSGMAccelerator not initialized!");
        return false;
    }

    auto startTotal = high_resolution_clock::now();

    // costVolume is 2D: height × (width * disparities)
    const uint32_t height = costVolume.rows;
    const uint32_t width = costVolume.cols / numDisparities;
    const uint32_t disparities = numDisparities;

    core::Logger::getInstance().log(core::LogLevel::DEBUG,
        "aggregateSGM: " + std::to_string(width) + "x" + std::to_string(height) + "x" + std::to_string(disparities));

    // Create or recreate buffers if size changed
    if (width != width_ || height != height_ || disparities != disparities_) {
        core::Logger::getInstance().log(core::LogLevel::DEBUG, "Creating Vulkan buffers...");
        destroyBuffers();
        if (!createBuffers(width, height, disparities)) {
            core::Logger::getInstance().log(core::LogLevel::ERROR, "Failed to create Vulkan buffers!");
            return false;
        }
        core::Logger::getInstance().log(core::LogLevel::DEBUG, "Vulkan buffers created successfully");
    }

    // Upload cost volume to GPU
    core::Logger::getInstance().log(core::LogLevel::DEBUG, "Uploading cost volume to GPU...");
    auto startUpload = high_resolution_clock::now();
    void* data;
    vkMapMemory(device_, costInputMemory_, 0, width * height * disparities * sizeof(float), 0, &data);
    memcpy(data, costVolume.data, width * height * disparities * sizeof(float));
    vkUnmapMemory(device_, costInputMemory_);
    auto endUpload = high_resolution_clock::now();
    core::Logger::getInstance().log(core::LogLevel::DEBUG, "Upload complete");

    // Allocate descriptor set if needed
    if (descriptorSet_ == VK_NULL_HANDLE) {
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = descriptorPool_;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &descriptorSetLayout_;

        if (vkAllocateDescriptorSets(device_, &allocInfo, &descriptorSet_) != VK_SUCCESS) {
            return false;
        }
    }

    // Update descriptor set with buffers
    VkDescriptorBufferInfo inputBufferInfo{};
    inputBufferInfo.buffer = costInputBuffer_;
    inputBufferInfo.offset = 0;
    inputBufferInfo.range = VK_WHOLE_SIZE;

    VkDescriptorBufferInfo outputBufferInfo{};
    outputBufferInfo.buffer = costOutputBuffer_;
    outputBufferInfo.offset = 0;
    outputBufferInfo.range = VK_WHOLE_SIZE;

    VkWriteDescriptorSet descriptorWrites[2] = {};
    descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[0].dstSet = descriptorSet_;
    descriptorWrites[0].dstBinding = 0;
    descriptorWrites[0].dstArrayElement = 0;
    descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[0].descriptorCount = 1;
    descriptorWrites[0].pBufferInfo = &inputBufferInfo;

    descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[1].dstSet = descriptorSet_;
    descriptorWrites[1].dstBinding = 1;
    descriptorWrites[1].dstArrayElement = 0;
    descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    descriptorWrites[1].descriptorCount = 1;
    descriptorWrites[1].pBufferInfo = &outputBufferInfo;

    vkUpdateDescriptorSets(device_, 2, descriptorWrites, 0, nullptr);

    // Zero output buffer before aggregation
    void* outputData;
    vkMapMemory(device_, costOutputMemory_, 0, width * height * disparities * sizeof(uint32_t), 0, &outputData);
    memset(outputData, 0, width * height * disparities * sizeof(uint32_t));
    vkUnmapMemory(device_, costOutputMemory_);

    // Execute compute shader
    auto startCompute = high_resolution_clock::now();

    VkCommandBufferAllocateInfo cmdAllocInfo{};
    cmdAllocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdAllocInfo.commandPool = commandPool_;
    cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAllocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(device_, &cmdAllocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);

    // Push constants: width, height, disparities, P1, P2, pathDir
    struct {
        uint32_t width;
        uint32_t height;
        uint32_t disparities;
        float P1;
        float P2;
        uint32_t pathDir;
    } pushConstants;

    pushConstants.width = width;
    pushConstants.height = height;
    pushConstants.disparities = disparities;
    pushConstants.P1 = P1;
    pushConstants.P2 = P2;

    // Run 4 SGM paths: L→R, R→L, T→B, B→T
    for (uint32_t pathDir = 0; pathDir < 4; ++pathDir) {
        pushConstants.pathDir = pathDir;
        vkCmdPushConstants(commandBuffer, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pushConstants), &pushConstants);

        // Dispatch with workgroup size 16x16
        uint32_t groupsX = (width + 15) / 16;
        uint32_t groupsY = (height + 15) / 16;
        vkCmdDispatch(commandBuffer, groupsX, groupsY, 1);

        // Memory barrier between paths
        if (pathDir < 3) {
            VkMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 0, 1, &barrier, 0, nullptr, 0, nullptr);
        }
    }

    vkEndCommandBuffer(commandBuffer);

    core::Logger::getInstance().log(core::LogLevel::DEBUG, "Submitting GPU commands...");

    // Submit and wait
    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(computeQueue_, 1, &submitInfo, VK_NULL_HANDLE);

    core::Logger::getInstance().log(core::LogLevel::DEBUG, "Waiting for GPU completion...");
    vkQueueWaitIdle(computeQueue_);

    core::Logger::getInstance().log(core::LogLevel::DEBUG, "GPU execution complete!");

    vkFreeCommandBuffers(device_, commandPool_, 1, &commandBuffer);

    auto endCompute = high_resolution_clock::now();

    // Download aggregated costs from GPU (as uint32 fixed-point)
    auto startDownload = high_resolution_clock::now();
    uint32_t* aggregatedData;
    vkMapMemory(device_, costOutputMemory_, 0, width * height * disparities * sizeof(uint32_t), 0, (void**)&aggregatedData);

    // Convert back to float and store in cv::Mat
    int dims[] = {(int)height, (int)width, (int)disparities};
    aggregatedCost = cv::Mat(3, dims, CV_32F);

    const float FIXED_POINT_SCALE = 256.0f;
    for (uint32_t i = 0; i < height * width * disparities; ++i) {
        aggregatedCost.at<float>(i) = static_cast<float>(aggregatedData[i]) / FIXED_POINT_SCALE;
    }

    vkUnmapMemory(device_, costOutputMemory_);
    auto endDownload = high_resolution_clock::now();

    lastStats_.uploadTimeMs = duration_cast<microseconds>(endUpload - startUpload).count() / 1000.0;
    lastStats_.computeTimeMs = duration_cast<microseconds>(endCompute - startCompute).count() / 1000.0;
    lastStats_.downloadTimeMs = duration_cast<microseconds>(endDownload - startDownload).count() / 1000.0;
    lastStats_.totalTimeMs = duration_cast<microseconds>(high_resolution_clock::now() - startTotal).count() / 1000.0;

    return true;  // Success!
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

// ========== HYBRID CPU/GPU PROCESSING HELPERS ==========

// Forward declarations from neon/census_neon.cpp and neon/hamming_neon.cpp
namespace {
    // Simple CPU census transform (fallback)
    void computeCensus9x9(const cv::Mat& gray, cv::Mat& census) {
        const int height = gray.rows;
        const int width = gray.cols;
        census = cv::Mat::zeros(height, width, CV_64F);

        for (int y = 4; y < height - 4; ++y) {
            uint64_t* censusRow = census.ptr<uint64_t>(y);
            for (int x = 4; x < width - 4; ++x) {
                uint8_t center = gray.at<uint8_t>(y, x);
                uint64_t censusValue = 0;
                int bitIdx = 0;

                for (int dy = -4; dy <= 4; ++dy) {
                    for (int dx = -4; dx <= 4; ++dx) {
                        if (dy == 0 && dx == 0) continue;
                        uint8_t neighbor = gray.at<uint8_t>(y + dy, x + dx);
                        if (neighbor > center) {
                            censusValue |= (1ULL << bitIdx);
                        }
                        bitIdx++;
                    }
                }
                censusRow[x] = censusValue;
            }
        }
    }

    // Hamming distance for 64-bit census descriptors
    inline int hammingDistance64(uint64_t a, uint64_t b) {
        return __builtin_popcountll(a ^ b);
    }
}

void VulkanSGMAccelerator::computeCensusTransform(const cv::Mat& image, cv::Mat& census) {
    cv::Mat gray;
    if (image.channels() > 1) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    computeCensus9x9(gray, census);
}

cv::Mat VulkanSGMAccelerator::computeCostVolume(
    const cv::Mat& leftCensus,
    const cv::Mat& rightCensus,
    int numDisparities)
{
    const int height = leftCensus.rows;
    const int width = leftCensus.cols;

    // Cost volume: height × width × numDisparities
    int dims[] = {height, width, numDisparities};
    cv::Mat costVolume(3, dims, CV_32F);

    for (int y = 0; y < height; ++y) {
        const uint64_t* leftRow = leftCensus.ptr<uint64_t>(y);
        const uint64_t* rightRow = rightCensus.ptr<uint64_t>(y);
        float* costRow = costVolume.ptr<float>(y);

        for (int x = 0; x < width; ++x) {
            for (int d = 0; d < numDisparities; ++d) {
                int xr = x - d;
                if (xr >= 0) {
                    int hammingDist = hammingDistance64(leftRow[x], rightRow[xr]);
                    costRow[x * numDisparities + d] = static_cast<float>(hammingDist);
                } else {
                    costRow[x * numDisparities + d] = 64.0f;
                }
            }
        }
    }

    return costVolume;
}

cv::Mat VulkanSGMAccelerator::selectDisparityWTA(
    const cv::Mat& aggregatedCost,
    int numDisparities,
    int minDisparity)
{
    const int height = aggregatedCost.size[0];
    const int width = aggregatedCost.size[1];

    cv::Mat disparity(height, width, CV_16S);

    for (int y = 0; y < height; ++y) {
        const float* costRow = aggregatedCost.ptr<float>(y);
        int16_t* dispRow = disparity.ptr<int16_t>(y);

        for (int x = 0; x < width; ++x) {
            float minCost = FLT_MAX;
            int bestDisp = 0;

            // Find minimum cost disparity
            for (int d = 0; d < numDisparities; ++d) {
                float cost = costRow[x * numDisparities + d];
                if (cost < minCost) {
                    minCost = cost;
                    bestDisp = d;
                }
            }

            // Subpixel refinement with parabolic fitting
            if (bestDisp > 0 && bestDisp < numDisparities - 1) {
                float c0 = costRow[x * numDisparities + bestDisp - 1];
                float c1 = costRow[x * numDisparities + bestDisp];
                float c2 = costRow[x * numDisparities + bestDisp + 1];

                float denom = 2.0f * (c0 - 2.0f * c1 + c2);
                if (std::abs(denom) > 0.001f) {
                    float delta = (c0 - c2) / denom;
                    dispRow[x] = static_cast<int16_t>((minDisparity + bestDisp + delta) * 16);
                } else {
                    dispRow[x] = static_cast<int16_t>((minDisparity + bestDisp) * 16);
                }
            } else {
                dispRow[x] = static_cast<int16_t>((minDisparity + bestDisp) * 16);
            }
        }
    }

    return disparity;
}

} // namespace stereo
} // namespace unlook
