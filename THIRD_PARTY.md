# Third-party sources and attribution

FiveAxisLab is distributed under GNU AGPL version 3 or later. See `LICENSE`. The portable build dynamically links its runtime dependencies; replacement DLLs may be used. The complete FiveAxisLab and modified FCL / OMPL / libccd sources are provided in the adjacent source ZIP. Dependency versions and checksums are recorded in `dependencies.lock.json` and `upstream.lock.json`; upstream license texts are shipped under `licenses/`.

| Dependency | Reuse | License / source |
|---|---|---|
| OrcaSlicer, `f0e9cdc470f3eba904a9a6cc2109e3793522b103` | Rounded-bead cross-section, spacing and width math adapted in `src/core/orca_adapter.cpp`; original Flow.cpp/hpp preserved | AGPL-3.0-or-later; [pinned source](https://github.com/OrcaSlicer/OrcaSlicer/tree/f0e9cdc470f3eba904a9a6cc2109e3793522b103). Orca derives from Bambu Studio, PrusaSlicer and Slic3r; attribution in retained files |
| Open CASCADE 7.9.3 | STEPCAF assembly translation and tessellation | LGPL-2.1 with OCCT exception; [source](https://dev.opencascade.org/resources/download) |
| Qt 6.11.2 | Widgets, OpenGL, workers, XML, tests | LGPL-3.0 / GPL / commercial as applicable; [source archives](https://download.qt.io/archive/qt/6.11/) |
| FCL 0.7.0 | Triangle BVH, dynamic broad phase, collision / distance | BSD; [source](https://github.com/flexible-collision-library/fcl/tree/0.7.0) |
| libccd 2.1 | FCL primitive narrow phase | BSD; [source](https://github.com/danfis/libccd/tree/v2.1) |
| OMPL 1.7.0 | RRT-Connect joint-space travel repair | BSD; [source](https://github.com/ompl/ompl/tree/1.7.0) |
| Eigen 5.0.1 | Linear algebra / sparse FEA | MPL-2.0; [source](https://gitlab.com/libeigen/eigen) |
| VTK 9.6.2 | ASCII, appended, binary and compressed VTU reading | BSD; [source](https://gitlab.kitware.com/vtk/vtk) |
| TetGen 1.6.0 | Closed-volume tetrahedral meshing | AGPL-3.0-or-later / commercial; [source](https://wias-berlin.de/software/tetgen/) |
| libzip and Boost | Project ZIP and OMPL dependencies | BSD / Boost Software License; runtime notices included |
| Vulkan loader/headers | Optional cross-vendor compute plugin | Apache-2.0 / MIT as applicable; [Khronos source](https://github.com/KhronosGroup/Vulkan-Loader) |
| shaderc, glslang and SPIR-V tools | Compile embedded compute GLSL to SPIR-V | Apache-2.0 / BSD as applicable; [shaderc source](https://github.com/google/shaderc); SDK notices shipped |
| Klipper, `e2b99a9159f992169c1ba53029dec020e850cc99` | Native host / Linux-MCU test runner; source included with build artifacts | GPL-3.0; [pinned source](https://github.com/Klipper3d/klipper/tree/e2b99a9159f992169c1ba53029dec020e850cc99) |

MSYS2 build recipes and corresponding dependency source packages are available from [MSYS2 MINGW-packages](https://github.com/msys2/MINGW-packages) and [the official source-package archive](https://repo.msys2.org/mingw/sources/). The lock lists every SDK package; only transitively imported runtime DLLs enter the portable application. Their bundled license notices include the MinGW runtime, compression and text-rendering dependencies.

The optional CUDA plugin contains original PTX and uses the NVIDIA driver supplied by the user; the NVIDIA driver is not redistributed. Its API boundary is a versioned plain-buffer C interface. The current implementation uses driver JIT compilation rather than linking a CUDA runtime/toolkit.

Vendor compatibility edits: OMPL uses consistent CMake linking and a unique uninstall target; FCL is built as C++17 with explicit `<cassert>` includes and Eigen configuration compatible with Eigen 5. MinGW inline-template export annotations are neutralized for the consuming translation units. These changes are reproduced by `tools/patch_vendor.py`.

Names Autodesk Fusion, OrcaSlicer, Duet, RepRapFirmware and Klipper identify compatibility targets. No affiliation or endorsement is claimed.
