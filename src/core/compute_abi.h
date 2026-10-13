#pragma once
#include <stddef.h>
#include <stdint.h>
#define FAL_COMPUTE_ABI 2u
#ifdef _WIN32
#define FAL_COMPUTE_EXPORT __declspec(dllexport)
#else
#define FAL_COMPUTE_EXPORT __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif
typedef struct FalComputeApi {
    uint32_t abi;
    int (*device_count)(void);
    int (*device_name)(int, char *, size_t);
    void *(*create)(int, double, char *, size_t);
    // Workloads: 0 affine XYZ, 1 conservative interval candidates,
    // 2 four-node interpolation of three scalar components (16 doubles/record).
    int (*transform)(void *, const double *, size_t, const double *, double *, char *, size_t, uint32_t);
    void (*destroy)(void *);
} FalComputeApi;
typedef const FalComputeApi *(*FalGetApi)(uint32_t);
#ifdef __cplusplus
}
#endif
