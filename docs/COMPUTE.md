# CPU and GPU execution

The Performance tab selects **Auto / CPU / NVIDIA CUDA / Vulkan**, device, CPU budget and memory fraction. CPU mode requires no GPU. Auto CPU budget is logical processors minus one, with a minimum of one. A bounded persistent pool processes independent objects, element assembly, sampling and collision batches. Nested pool work runs serially to avoid multiplying the configured worker count. Open CASCADE meshing remains controlled independently.

CUDA and Vulkan are optional libraries loaded through **C ABI 2**: plain buffers, version checks and plugin-owned contexts. No C++/Qt object or allocator crosses the plugin boundary. One process-wide dispatch lock serializes compute dispatch, and one selected device handles a job. Batch size is bounded to 65,536 records and memory fraction to the configured budget.

Implemented kernels evaluate affine geometry transforms, conservative interval/bounding candidates, four-node field-component interpolation and triangle-normal preparation for preview. Tetrahedron correspondence, quadratic-tetrahedron reference interpolation, topology, eigensystems, FCL/containment, controller semantics and final certification remain on CPU. Every returned batch is compared with an FP64 CPU reference; non-finite input, overflow and parity failure cannot certify geometry. Ambiguous range boundaries retain conservative CPU candidates.

CUDA uses original embedded FP64 PTX, compiled by the installed NVIDIA driver. This avoids a dependency on a separately installed NVCC/MSVC toolkit while preserving the isolated ABI. The current PTX target requires Ampere-or-newer NVIDIA hardware; unsupported targets fall back. A future NVCC implementation can replace the plugin without changing the application ABI. [NVIDIA driver JIT documentation](https://docs.nvidia.com/cuda/cuda-programming-guide/03-advanced/driver-api.html).

Vulkan uses shaderc/SPIR-V, a compute queue, FP64 feature qualification and coherent host-visible buffers. It is the cross-vendor route intended for AMD as well as NVIDIA. Devices lacking FP64 support fall back. The Windows host's RTSS overlay crashed the initial compute-only Vulkan instance; the plugin now filters only `*RTSS*` implicit layers for its process when the user has not provided a loader filter. No system overlay setting is changed. [Vulkan overview](https://docs.vulkan.org/guide/latest/what_is_vulkan.html), [loader filters](https://vulkan.lunarg.com/doc/view/latest/windows/LoaderLayerInterface.html).

Cancellation discards the incomplete batch and cancels the job without a certificate. Allocation, driver/device and ABI failures discard GPU output and use CPU for the affected batch. Reports record requested/actual backend, device, precision, fallback, batches, CPU reference time, GPU initialization/dispatch/transfers and end-to-end time.

Auto compares measured GPU cost against CPU cost, including transfers and initialization. Cached measurements are scoped by workload, device and batch-size bucket. The current RTX 5090 benchmarks favor CPU for these verified kernels, so Auto keeps CPU. Selecting CUDA or Vulkan explicitly permits reproducible qualification and benchmarking; GPU availability alone is not treated as a speedup.

```powershell
./fiveaxis_cli.exe --list-compute-devices
./fiveaxis_cli.exe --compute-benchmark compute.json --threads 1
./fiveaxis_cli.exe --validate plate.5axis --reslice --threads 8 --compute cpu --report plate-report.json
./fiveaxis_cli.exe --validate plate.5axis --reslice --compute cuda --report cuda-report.json
```

`examples/reports/compute-v04.json` contains actual NVIDIA CUDA and Vulkan measurements. AMD hardware has not been available for qualification. Driver-loss and memory failures use the same fallback path; induced allocation-budget failure, missing device, cancellation and non-finite cases are tested. Real driver-reset behavior and actual AMD performance remain separate qualification tasks.
