#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "../core/compute_abi.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <windows.h>
namespace {
using Context = void *;
using Module = void *;
using Function = void *;
using Pointer = unsigned long long;
struct Driver {
    HMODULE library = nullptr;
    int(__stdcall *init)(unsigned) = nullptr;
    int(__stdcall *count)(int *) = nullptr;
    int(__stdcall *name)(char *, int, int) = nullptr;
    int(__stdcall *create)(Context *, unsigned, int) = nullptr;
    int(__stdcall *destroy)(Context) = nullptr;
    int(__stdcall *setCurrent)(Context) = nullptr;
    int(__stdcall *moduleLoad)(Module *, const void *) = nullptr;
    int(__stdcall *moduleFree)(Module) = nullptr;
    int(__stdcall *function)(Function *, Module, const char *) = nullptr;
    int(__stdcall *allocate)(Pointer *, size_t) = nullptr;
    int(__stdcall *freeMemory)(Pointer) = nullptr;
    int(__stdcall *upload)(Pointer, const void *, size_t) = nullptr;
    int(__stdcall *download)(void *, Pointer, size_t) = nullptr;
    int(__stdcall *launch)(Function, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned,
                           void *, void **, void **) = nullptr;
    int(__stdcall *sync)() = nullptr;
    int(__stdcall *memoryInfo)(size_t *, size_t *) = nullptr;
    std::once_flag initialized;
    bool ready() {
        std::call_once(initialized, [&] {
            library = GetModuleHandleW(L"nvcuda.dll");
            if (!library)
                library = LoadLibraryW(L"nvcuda.dll");
            if (!library)
                return;
#define LOAD(member, symbol) member = reinterpret_cast<decltype(member)>(GetProcAddress(library, symbol))
            LOAD(init, "cuInit");
            LOAD(count, "cuDeviceGetCount");
            LOAD(name, "cuDeviceGetName");
            LOAD(create, "cuCtxCreate_v2");
            LOAD(destroy, "cuCtxDestroy_v2");
            LOAD(setCurrent, "cuCtxSetCurrent");
            LOAD(moduleLoad, "cuModuleLoadData");
            LOAD(moduleFree, "cuModuleUnload");
            LOAD(function, "cuModuleGetFunction");
            LOAD(allocate, "cuMemAlloc_v2");
            LOAD(freeMemory, "cuMemFree_v2");
            LOAD(upload, "cuMemcpyHtoD_v2");
            LOAD(download, "cuMemcpyDtoH_v2");
            LOAD(launch, "cuLaunchKernel");
            LOAD(sync, "cuCtxSynchronize");
            LOAD(memoryInfo, "cuMemGetInfo_v2");
#undef LOAD
        });
        return init && count && name && create && destroy && setCurrent && moduleLoad && moduleFree &&
               function && allocate && freeMemory && upload && download && launch && sync && memoryInfo &&
               init(0) == 0;
    }
} driver;
// Portable PTX is JIT-compiled by the installed NVIDIA driver. The plugin exports only
// a C ABI; no CUDA/Qt/STL allocation crosses the application's compiler boundary.
const char kernel[] = R"ptx(
.version 7.0
.target sm_80
.address_size 64
.visible .entry fal_transform(.param .u64 src,.param .u64 dst,.param .u64 matrix,.param .u64 count,.param .u32 workload){
 .reg .pred %p<4>;
 .reg .b32 %r<8>;
 .reg .b64 %d<16>;
 .reg .f64 %f<12>;
 ld.param.u64 %d0,[src];ld.param.u64 %d1,[dst];ld.param.u64 %d2,[matrix];ld.param.u64 %d3,[count];
 mov.u32 %r0,%tid.x;mov.u32 %r1,%ctaid.x;mov.u32 %r2,%ntid.x;mad.lo.u32 %r3,%r1,%r2,%r0;
 cvt.u64.u32 %d4,%r3;setp.ge.u64 %p0,%d4,%d3;@%p0 bra END;
 ld.param.u32 %r6,[workload];setp.eq.u32 %p0,%r6,2;@%p0 bra SAMPLE;setp.eq.u32 %p0,%r6,3;@%p0 bra NORMAL;
 mul.wide.u32 %d5,%r3,24;add.u64 %d6,%d0,%d5;add.u64 %d7,%d1,%d5;
 ld.global.f64 %f0,[%d6];ld.global.f64 %f1,[%d6+8];ld.global.f64 %f2,[%d6+16];
 setp.eq.u32 %p0,%r6,1;@%p0 bra RANGE;
 mov.u32 %r4,0;
 ROW:
 mul.wide.u32 %d8,%r4,32;add.u64 %d9,%d2,%d8;
 ld.global.f64 %f3,[%d9];ld.global.f64 %f4,[%d9+8];ld.global.f64 %f5,[%d9+16];ld.global.f64 %f6,[%d9+24];
 mul.rn.f64 %f7,%f0,%f3;fma.rn.f64 %f7,%f1,%f4,%f7;fma.rn.f64 %f7,%f2,%f5,%f7;add.rn.f64 %f7,%f7,%f6;
 mul.wide.u32 %d10,%r4,8;add.u64 %d11,%d7,%d10;st.global.f64 [%d11],%f7;
 add.u32 %r4,%r4,1;setp.lt.u32 %p0,%r4,3;@%p0 bra ROW;
 bra END;
 RANGE:
 st.global.f64 [%d7],%f0;st.global.f64 [%d7+8],%f1;
 setp.le.f64 %p1,%f0,%f2;setp.ge.f64 %p2,%f1,%f2;and.pred %p0,%p1,%p2;
 selp.f64 %f3,0d3ff0000000000000,0d0000000000000000,%p0;st.global.f64 [%d7+16],%f3;bra END;
 SAMPLE:
 mul.wide.u32 %d5,%r3,128;add.u64 %d6,%d0,%d5;mul.wide.u32 %d5,%r3,24;add.u64 %d7,%d1,%d5;mov.u32 %r4,0;
 SAMPLE_ROW:
 mov.f64 %f7,0d0000000000000000;mov.u32 %r5,0;
 SAMPLE_NODE:
 mul.wide.u32 %d8,%r5,8;add.u64 %d9,%d6,%d8;ld.global.f64 %f0,[%d9];
 mul.wide.u32 %d8,%r5,24;mul.wide.u32 %d10,%r4,8;add.u64 %d8,%d8,%d10;add.u64 %d8,%d8,32;add.u64 %d9,%d6,%d8;ld.global.f64 %f1,[%d9];fma.rn.f64 %f7,%f0,%f1,%f7;
 add.u32 %r5,%r5,1;setp.lt.u32 %p0,%r5,4;@%p0 bra SAMPLE_NODE;
 mul.wide.u32 %d10,%r4,8;add.u64 %d11,%d7,%d10;st.global.f64 [%d11],%f7;
 add.u32 %r4,%r4,1;setp.lt.u32 %p0,%r4,3;@%p0 bra SAMPLE_ROW;
 bra END;
 NORMAL:
 mul.wide.u32 %d5,%r3,72;add.u64 %d6,%d0,%d5;mul.wide.u32 %d5,%r3,24;add.u64 %d7,%d1,%d5;
 ld.global.f64 %f0,[%d6];ld.global.f64 %f1,[%d6+8];ld.global.f64 %f2,[%d6+16];
 ld.global.f64 %f3,[%d6+24];ld.global.f64 %f4,[%d6+32];ld.global.f64 %f5,[%d6+40];ld.global.f64 %f6,[%d6+48];ld.global.f64 %f7,[%d6+56];ld.global.f64 %f8,[%d6+64];
 sub.rn.f64 %f3,%f3,%f0;sub.rn.f64 %f4,%f4,%f1;sub.rn.f64 %f5,%f5,%f2;sub.rn.f64 %f6,%f6,%f0;sub.rn.f64 %f7,%f7,%f1;sub.rn.f64 %f8,%f8,%f2;
 mul.rn.f64 %f0,%f4,%f8;mul.rn.f64 %f9,%f5,%f7;sub.rn.f64 %f0,%f0,%f9;
 mul.rn.f64 %f1,%f5,%f6;mul.rn.f64 %f9,%f3,%f8;sub.rn.f64 %f1,%f1,%f9;
 mul.rn.f64 %f2,%f3,%f7;mul.rn.f64 %f9,%f4,%f6;sub.rn.f64 %f2,%f2,%f9;
 mul.rn.f64 %f9,%f0,%f0;fma.rn.f64 %f9,%f1,%f1,%f9;fma.rn.f64 %f9,%f2,%f2,%f9;sqrt.rn.f64 %f9,%f9;setp.le.f64 %p0,%f9,0d3d719799812dea11;@%p0 bra DEGENERATE;
 div.rn.f64 %f0,%f0,%f9;div.rn.f64 %f1,%f1,%f9;div.rn.f64 %f2,%f2,%f9;bra WRITE_NORMAL;
 DEGENERATE:mov.f64 %f0,0d0000000000000000;mov.f64 %f1,0d0000000000000000;mov.f64 %f2,0d0000000000000000;
 WRITE_NORMAL:st.global.f64 [%d7],%f0;st.global.f64 [%d7+8],%f1;st.global.f64 [%d7+16],%f2;
 END:ret;
})ptx";
struct State {
    Context context = nullptr;
    Module module = nullptr;
    Function function = nullptr;
    double fraction = .6;
};
int devices() {
    int count = 0;
    return driver.ready() && driver.count(&count) == 0 ? count : 0;
}
int device_name(int index, char *buffer, size_t size) {
    return driver.ready() ? driver.name(buffer, int(size), index) : 1;
}
void message(char *error, size_t size, const char *text, int code) {
    if (size)
        std::snprintf(error, size, "%s (CUDA %d)", text, code);
}
void release(void *value) {
    auto s = static_cast<State *>(value);
    if (!s)
        return;
    if (s->context) {
        driver.setCurrent(s->context);
        if (s->module)
            driver.moduleFree(s->module);
        driver.destroy(s->context);
    }
    delete s;
}
void *create(int index, double fraction, char *error, size_t size) {
    if (!driver.ready()) {
        message(error, size, "NVIDIA driver unavailable", 1);
        return nullptr;
    }
    auto s = new State;
    s->fraction = fraction;
    int status = driver.create(&s->context, 0, index);
    if (!status)
        status = driver.moduleLoad(&s->module, kernel);
    if (!status)
        status = driver.function(&s->function, s->module, "fal_transform");
    if (status) {
        message(error, size, "Cannot initialize CUDA kernel", status);
        release(s);
        return nullptr;
    }
    return s;
}
int transform(void *value, const double *input, size_t count, const double *matrix, double *output,
              char *error, size_t size, uint32_t workload) {
    auto s = static_cast<State *>(value);
    if (!s || !input || !matrix || !output || count > 65536 || workload > 3) {
        message(error, size, "Invalid/big CUDA batch", 1);
        return 1;
    }
    driver.setCurrent(s->context);
    size_t available = 0, total = 0;
    driver.memoryInfo(&available, &total);
    size_t bytes = count * 3 * sizeof(double), inputBytes = count *
                                                            (workload == 2   ? 16
                                                             : workload == 3 ? 9
                                                                             : 3) *
                                                            sizeof(double);
    if (bytes + inputBytes + 96 > available * s->fraction) {
        message(error, size, "GPU memory budget exceeded", 2);
        return 2;
    }
    Pointer src = 0, dst = 0, coefficients = 0;
    int status = driver.allocate(&src, inputBytes);
    if (!status)
        status = driver.allocate(&dst, bytes);
    if (!status)
        status = driver.allocate(&coefficients, 96);
    if (!status)
        status = driver.upload(src, input, inputBytes);
    if (!status)
        status = driver.upload(coefficients, matrix, 96);
    unsigned long long n = count;
    void *arguments[] = {&src, &dst, &coefficients, &n, &workload};
    if (!status)
        status = driver.launch(s->function, unsigned((count + 255) / 256), 1, 1, 256, 1, 1, 0, nullptr,
                               arguments, nullptr);
    if (!status)
        status = driver.sync();
    if (!status)
        status = driver.download(output, dst, bytes);
    if (src)
        driver.freeMemory(src);
    if (dst)
        driver.freeMemory(dst);
    if (coefficients)
        driver.freeMemory(coefficients);
    if (status)
        message(error, size, "CUDA batch failed; discard output", status);
    return status;
}
FalComputeApi api{FAL_COMPUTE_ABI, devices, device_name, create, transform, release};
} // namespace
extern "C" FAL_COMPUTE_EXPORT const FalComputeApi *fal_get_api(uint32_t version) {
    return version == FAL_COMPUTE_ABI ? &api : nullptr;
}
