# FiveAxisLab 0.4.0 release evidence

Windows x64 research build with schema-2 plates, materials, per-tool CAD/dummies, installed nozzles, explicit recipes, purge history, complete-object scheduling and optional compute plugins.

## Automated checks

| Suite | Passed | Skipped |
|---|---:|---:|
| Core geometry, FEM, slicing and controllers | 55 | 0 |
| Plate, tool/material, scheduling and GPU cases | 33 | 0 |
| Native Qt desktop workflows | 12 | 0 |
| Python Klipper flow/tool guards | 11 | 0 |

Qt counts include initialization and cleanup cases. Machine-readable evidence is in `examples/reports/0.4.0-*.xml` and `0.4.0-release.json`. The desktop editor and simulation were captured from the running native application in `plate-editor.png` and `plate-preview.png`.

Coverage includes schema migration/round trips, STEP deduplication and corruption, installed-nozzle preservation, TCP changes without carriage jumps, scoped shared-component identities, disconnected-component containment, material volume splitting and tiny-feature rejection, load/unload/purge state, per-cell material FEA, thin obstacles, material birth including the last bead, backward state restoration, and bounded deterministic travel/order repair. One complete-object order collides and the reversed order passes. Impossible/budget-exhausted schedules remain blocked.

## Controller and whole-job evidence

The sequential, shared-nozzle and two-tool fixtures pass configured geometric validation and exported semantic replay. Their reports explicitly retain `nativeFirmwareValidated: false` and `physicalPrintingValidated: false`. CUDA-selected and Vulkan-selected sequential jobs also pass replay on the actual NVIDIA device; final collision certification remains CPU-owned.

Controller replay checks declared state commands, startup offsets, motion, relative E, waits and timing. This does not prove GPIO polarity, latch reliability, heater calibration, feeder behavior or native firmware execution on a printer. The native Klipper rerun timed out at startup after 120 seconds; it provides no new native qualification. Earlier native firmware evidence remains historical.

## Performance evidence

The available GPU was an **NVIDIA GeForce RTX 5090**, driver **595.97**, approximately 32 GB VRAM. Both CUDA driver-JIT and Vulkan FP64 kernels executed. No AMD device was available. `compute-v04.json` records affine, range/candidate, interpolation and normal workloads at several batch sizes, with CPU reference, initialization/dispatch/transfers and total timing.

Every result retains the CPU reference. These verified kernels currently add cost to CPU execution, so Auto keeps CPU. Explicit CUDA/Vulkan modes support reproducible qualification rather than a claimed speedup. Missing device, memory-budget failure, cancellation and non-finite inputs are covered. Actual AMD behavior and induced driver reset/device loss require hardware testing.

`plate-benchmarks.json` includes end-to-end generation/validation/replay durations and explicit budgets. Existing slow curved fixtures continue to use a bounded benchmark; a timeout is not a certificate. Transfers and final checking are included in whole-job measurements.

## Research boundaries

Mixed-material FEA assumes perfect bonds and dense material; it does not resolve sparse infill, adhesion, thermal distortion or fracture. Material feature/coverage checks are geometric approximations. Orientation retry is bounded and requires a deposition recheck; profile-normal constraints remain enforced. Search exhaustion does not establish impossibility.

The station waste model is its fully occupied envelope; the tower uses accumulated deposited layers. Contact allowances are limited to named, calibrated interfaces and the active nozzle/object. CAD/contact calibration and real hardware evidence are still required for a physical machine. IDEX, continuous mixing, proprietary feeder macros and globally optimal decomposition are outside this release.
