#include "../core/compute_abi.h"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <shaderc/shaderc.h>
#include <stdexcept>
#include <vector>
#include <vulkan/vulkan.h>
namespace {
void check(VkResult status, const char *text) {
    if (status != VK_SUCCESS)
        throw std::runtime_error(std::string(text) + " (Vulkan " + std::to_string(status) + ")");
}
struct Instance {
    VkInstance value = VK_NULL_HANDLE;
    std::vector<VkPhysicalDevice> physical;
    Instance() {
#ifdef _WIN32
        // RTSS's implicit graphics overlay crashes compute-only instances on the qualified
        // Windows host. Scope the loader filter to this process and respect explicit user
        // overrides; no system/overlay configuration is changed.
        static std::once_flag filter;
        std::call_once(filter, [] {
            if (GetEnvironmentVariableW(L"VK_LOADER_LAYERS_DISABLE", nullptr, 0) == 0)
                SetEnvironmentVariableW(L"VK_LOADER_LAYERS_DISABLE", L"*RTSS*");
        });
#endif
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "FiveAxisLab compute";
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        info.pApplicationInfo = &app;
        check(vkCreateInstance(&info, nullptr, &value), "Create compute instance");
        uint32_t count = 0;
        check(vkEnumeratePhysicalDevices(value, &count, nullptr), "List devices");
        physical.resize(count);
        check(vkEnumeratePhysicalDevices(value, &count, physical.data()), "List devices");
    }
    ~Instance() {
        if (value)
            vkDestroyInstance(value, nullptr);
    }
};
int devices() {
    try {
        Instance instance;
        return instance.physical.size();
    } catch (...) {
        return 0;
    }
}
int device_name(int index, char *buffer, size_t size) {
    try {
        Instance instance;
        if (index < 0 || index >= int(instance.physical.size()))
            return 1;
        VkPhysicalDeviceProperties p;
        vkGetPhysicalDeviceProperties(instance.physical[index], &p);
        std::snprintf(buffer, size, "%s", p.deviceName);
        return 0;
    } catch (...) {
        return 1;
    }
}
struct State {
    Instance instance;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t family = 0;
    VkShaderModule shader = VK_NULL_HANDLE;
    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    double fraction = .6;
    ~State() {
        if (device) {
            vkDeviceWaitIdle(device);
            if (commandPool)
                vkDestroyCommandPool(device, commandPool, nullptr);
            if (pipeline)
                vkDestroyPipeline(device, pipeline, nullptr);
            if (layout)
                vkDestroyPipelineLayout(device, layout, nullptr);
            if (setLayout)
                vkDestroyDescriptorSetLayout(device, setLayout, nullptr);
            if (shader)
                vkDestroyShaderModule(device, shader, nullptr);
            vkDestroyDevice(device, nullptr);
        }
    }
};
const char source[] = R"glsl(
#version 450
#extension GL_ARB_gpu_shader_fp64 : require
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer Input { double inputValues[]; };
layout(std430,binding=1) writeonly buffer Output { double outputValues[]; };
layout(std430,binding=2) readonly buffer Coefficients { double matrix[]; };
layout(push_constant) uniform Parameters { uint count; uint workload; };
void main(){uint i=gl_GlobalInvocationID.x;if(i>=count)return;
 if(workload==3){uint base=i*9;dvec3 a=dvec3(inputValues[base],inputValues[base+1],inputValues[base+2]),b=dvec3(inputValues[base+3],inputValues[base+4],inputValues[base+5]),c=dvec3(inputValues[base+6],inputValues[base+7],inputValues[base+8]);dvec3 n=cross(b-a,c-a);double magnitude=length(n);n=magnitude>1e-12?n/magnitude:dvec3(0);for(uint j=0;j<3;j++)outputValues[i*3+j]=n[j];return;}
 if(workload==1){double lo=inputValues[i*3],hi=inputValues[i*3+1],level=inputValues[i*3+2];outputValues[i*3]=lo;outputValues[i*3+1]=hi;outputValues[i*3+2]=(lo<=level&&hi>=level)?1.0:0.0;return;}
 if(workload==2){for(uint c=0;c<3;c++){double value=0.0;for(uint node=0;node<4;node++)value+=inputValues[i*16+node]*inputValues[i*16+4+node*3+c];outputValues[i*3+c]=value;}return;}
for(uint row=0;row<3;row++){uint r=row*4;outputValues[i*3+row]=inputValues[i*3]*matrix[r]+inputValues[i*3+1]*matrix[r+1]+inputValues[i*3+2]*matrix[r+2]+matrix[r+3];}}
)glsl";
void *create(int index, double fraction, char *error, size_t size) {
    State *s = nullptr;
    try {
        s = new State;
        if (index < 0 || index >= int(s->instance.physical.size()))
            throw std::runtime_error("Requested Vulkan device unavailable");
        s->physical = s->instance.physical[index];
        s->fraction = fraction;
        VkPhysicalDeviceFeatures supported;
        vkGetPhysicalDeviceFeatures(s->physical, &supported);
        if (!supported.shaderFloat64)
            throw std::runtime_error("FP64 compute feature unavailable; CPU fallback required");
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(s->physical, &count, nullptr);
        std::vector<VkQueueFamilyProperties> queues(count);
        vkGetPhysicalDeviceQueueFamilyProperties(s->physical, &count, queues.data());
        bool found = false;
        for (uint32_t i = 0; i < count; ++i)
            if (queues[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                s->family = i;
                found = true;
                break;
            }
        if (!found)
            throw std::runtime_error("No compute queue");
        float priority = 1;
        VkDeviceQueueCreateInfo queue{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queue.queueFamilyIndex = s->family;
        queue.queueCount = 1;
        queue.pQueuePriorities = &priority;
        VkPhysicalDeviceFeatures features{};
        features.shaderFloat64 = VK_TRUE;
        VkDeviceCreateInfo device{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        device.queueCreateInfoCount = 1;
        device.pQueueCreateInfos = &queue;
        device.pEnabledFeatures = &features;
        check(vkCreateDevice(s->physical, &device, nullptr, &s->device), "Create compute device");
        vkGetDeviceQueue(s->device, s->family, 0, &s->queue);
        auto compiler = shaderc_compiler_initialize();
        auto result = shaderc_compile_into_spv(compiler, source, std::strlen(source), shaderc_compute_shader,
                                               "fal_transform.comp", "main", nullptr);
        if (shaderc_result_get_compilation_status(result) != shaderc_compilation_status_success) {
            std::string message = shaderc_result_get_error_message(result);
            shaderc_result_release(result);
            shaderc_compiler_release(compiler);
            throw std::runtime_error(message);
        }
        VkShaderModuleCreateInfo shader{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        shader.codeSize = shaderc_result_get_length(result);
        shader.pCode = reinterpret_cast<const uint32_t *>(shaderc_result_get_bytes(result));
        check(vkCreateShaderModule(s->device, &shader, nullptr, &s->shader), "Create shader");
        shaderc_result_release(result);
        shaderc_compiler_release(compiler);
        VkDescriptorSetLayoutBinding bindings[3]{};
        for (uint32_t i = 0; i < 3; ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo descriptors{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        descriptors.bindingCount = 3;
        descriptors.pBindings = bindings;
        check(vkCreateDescriptorSetLayout(s->device, &descriptors, nullptr, &s->setLayout),
              "Create descriptors");
        VkPushConstantRange push{VK_SHADER_STAGE_COMPUTE_BIT, 0, 8};
        VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        layout.setLayoutCount = 1;
        layout.pSetLayouts = &s->setLayout;
        layout.pushConstantRangeCount = 1;
        layout.pPushConstantRanges = &push;
        check(vkCreatePipelineLayout(s->device, &layout, nullptr, &s->layout), "Create pipeline layout");
        VkComputePipelineCreateInfo pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        pipeline.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
        pipeline.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        pipeline.stage.module = s->shader;
        pipeline.stage.pName = "main";
        pipeline.layout = s->layout;
        check(vkCreateComputePipelines(s->device, VK_NULL_HANDLE, 1, &pipeline, nullptr, &s->pipeline),
              "Create pipeline");
        VkCommandPoolCreateInfo commands{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        commands.queueFamilyIndex = s->family;
        check(vkCreateCommandPool(s->device, &commands, nullptr, &s->commandPool), "Create command pool");
        return s;
    } catch (const std::exception &e) {
        std::snprintf(error, size, "%s", e.what());
        delete s;
        return nullptr;
    }
}
struct Buffer {
    State *s;
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    size_t bytes = 0;
    Buffer(State *state, size_t length) : s(state), bytes(length) {
        VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        info.size = bytes;
        info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        check(vkCreateBuffer(s->device, &info, nullptr, &buffer), "Allocate buffer");
        VkMemoryRequirements requirements;
        vkGetBufferMemoryRequirements(s->device, buffer, &requirements);
        VkPhysicalDeviceMemoryProperties properties;
        vkGetPhysicalDeviceMemoryProperties(s->physical, &properties);
        uint32_t type = UINT32_MAX;
        for (uint32_t i = 0; i < properties.memoryTypeCount; ++i)
            if ((requirements.memoryTypeBits & (1u << i)) &&
                (properties.memoryTypes[i].propertyFlags &
                 (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) ==
                    (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
                type = i;
                break;
            }
        if (type == UINT32_MAX)
            throw std::runtime_error("No coherent host-visible compute memory");
        if (requirements.size >
            properties.memoryHeaps[properties.memoryTypes[type].heapIndex].size * s->fraction / 3)
            throw std::runtime_error("Vulkan memory budget exceeded");
        VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocation.allocationSize = requirements.size;
        allocation.memoryTypeIndex = type;
        check(vkAllocateMemory(s->device, &allocation, nullptr, &memory), "Allocate GPU memory");
        check(vkBindBufferMemory(s->device, buffer, memory, 0), "Bind buffer");
    }
    ~Buffer() {
        if (buffer)
            vkDestroyBuffer(s->device, buffer, nullptr);
        if (memory)
            vkFreeMemory(s->device, memory, nullptr);
    }
    void write(const void *value) {
        void *mapped = nullptr;
        check(vkMapMemory(s->device, memory, 0, bytes, 0, &mapped), "Map input");
        std::memcpy(mapped, value, bytes);
        vkUnmapMemory(s->device, memory);
    }
    void read(void *value) {
        void *mapped = nullptr;
        check(vkMapMemory(s->device, memory, 0, bytes, 0, &mapped), "Map output");
        std::memcpy(value, mapped, bytes);
        vkUnmapMemory(s->device, memory);
    }
};
int transform(void *value, const double *input, size_t count, const double *matrix, double *output,
              char *error, size_t size, uint32_t workload) {
    auto s = static_cast<State *>(value);
    VkDescriptorPool pool = VK_NULL_HANDLE;
    VkCommandBuffer command = VK_NULL_HANDLE;
    try {
        if (!s || !input || !output || !matrix || count == 0 || count > 65536 || workload > 3)
            throw std::runtime_error("Invalid Vulkan batch");
        Buffer src(s, count * (workload == 2   ? 128
                               : workload == 3 ? 72
                                               : 24)),
            dst(s, count * 24), coefficients(s, 96);
        src.write(input);
        coefficients.write(matrix);
        VkDescriptorPoolSize capacity{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &capacity;
        check(vkCreateDescriptorPool(s->device, &poolInfo, nullptr, &pool), "Descriptor pool");
        VkDescriptorSet set;
        VkDescriptorSetAllocateInfo allocation{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocation.descriptorPool = pool;
        allocation.descriptorSetCount = 1;
        allocation.pSetLayouts = &s->setLayout;
        check(vkAllocateDescriptorSets(s->device, &allocation, &set), "Descriptor set");
        VkDescriptorBufferInfo buffers[] = {
            {src.buffer, 0, src.bytes}, {dst.buffer, 0, dst.bytes}, {coefficients.buffer, 0, 96}};
        VkWriteDescriptorSet writes[3]{};
        for (uint32_t i = 0; i < 3; ++i) {
            writes[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            writes[i].dstSet = set;
            writes[i].dstBinding = i;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &buffers[i];
        }
        vkUpdateDescriptorSets(s->device, 3, writes, 0, nullptr);
        VkCommandBufferAllocateInfo allocate{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocate.commandPool = s->commandPool;
        allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocate.commandBufferCount = 1;
        check(vkAllocateCommandBuffers(s->device, &allocate, &command), "Command buffer");
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        check(vkBeginCommandBuffer(command, &begin), "Begin compute");
        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, s->pipeline);
        vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_COMPUTE, s->layout, 0, 1, &set, 0, nullptr);
        uint32_t n = count, parameters[] = {n, workload};
        vkCmdPushConstants(command, s->layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, 8, parameters);
        vkCmdDispatch(command, (n + 127) / 128, 1, 1);
        VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(command, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 1,
                             &barrier, 0, nullptr, 0, nullptr);
        check(vkEndCommandBuffer(command), "Finish compute");
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &command;
        check(vkQueueSubmit(s->queue, 1, &submit, VK_NULL_HANDLE), "Dispatch compute");
        check(vkQueueWaitIdle(s->queue), "Wait compute");
        dst.read(output);
        vkFreeCommandBuffers(s->device, s->commandPool, 1, &command);
        vkDestroyDescriptorPool(s->device, pool, nullptr);
        return 0;
    } catch (const std::exception &e) {
        if (s && s->device) {
            vkDeviceWaitIdle(s->device);
            if (command)
                vkFreeCommandBuffers(s->device, s->commandPool, 1, &command);
            if (pool)
                vkDestroyDescriptorPool(s->device, pool, nullptr);
        }
        std::snprintf(error, size, "%s", e.what());
        return 1;
    }
}
void release(void *value) {
    delete static_cast<State *>(value);
}
FalComputeApi api{FAL_COMPUTE_ABI, devices, device_name, create, transform, release};
} // namespace
extern "C" FAL_COMPUTE_EXPORT const FalComputeApi *fal_get_api(uint32_t version) {
    return version == FAL_COMPUTE_ABI ? &api : nullptr;
}
