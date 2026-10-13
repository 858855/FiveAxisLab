# Validation ledger

This document distinguishes core / geometric checks, controller interpretation and physical evidence. Individual results and durations are stored in `examples/reports/` after the release checks. No physical print or real machine calibration has been performed.

| Area | Evidence implemented | Qualification boundary |
|---|---|---|
| STEP | Generated nested assemblies, repeated instance IDs, inch conversion, malformed input, cached source / meshes, calibrated links | Generated CAD only; real vendor assemblies remain to be qualified |
| Kinematics | 100 poses per layout, inverse / forward agreement, TCP compensation, stale calibration rejection | Ideal rigid calibrated machine; backlash, compliance and encoder offsets not modeled |
| Collision | Thin obstacle between endpoints, rotary sweep, containment, bounded interfaces, conical nozzle contact vs heater / neighboring material, deposited-only material, clearance / mesh uncertainty | Triangle / bounding-box approximation; all modeled bodies listed. Accuracy budget exhaustion blocks export |
| Simulation | Backward deposition restoration, playback-invariant validation, native UI smoke test for playback, scrubbing, controller switching and background replay; desktop screenshot | Broader GUI interaction and stress-overlay qualification remain pending |
| FEA | Analytic constant-strain tetra compression and singular fixture rejection | Linear static 4-node isotropic / oriented dense-material elasticity; sparse infill, nonlinear plasticity, fracture and thermal effects excluded |
| Fusion | Synthetic tensor / six component VTU, rotated units, correspondence and embedded-step rejection, scalar-only gating, curved quadratic-tetra mapping | Real Fusion static-study files with known values have not been supplied. Step labels are user confirmed when VTU metadata is absent |
| Curved slicing | Stress-weighted field produces tilted layer normals and coordinated rotary motion; finite reachable paths; sampled coverage / support dependencies | Monotone volumetric field and surface charts. Global arbitrary-topology decomposition, exact volumetric coverage and guaranteed removable supports remain research work |
| Avoidance | Successful deterministic RRT travel repair preserves retraction; successful bounded orientation repair; failure blocks export | Broad complex-part and decomposition-regeneration qualification remains pending |
| Duet | Generated output semantic replay through the core adapter | No native RepRapFirmware run or physical Duet machine test |
| Klipper | Eight extension unit tests; stock / extension synchronized native-host batches; expected flow rejection; actual generated bed and compensated-head jobs | Pinned native `GCODE_AXIS` version required. Batch mode bypasses heater readiness; cold guard has unit evidence only. No hardware tested |
| Packaging | Portable runtime resolves its own DLLs, command-line run and native UI capture without SDK PATH | Windows x64 verified locally; Windows installer signing and other operating systems pending |

The roadmap is not fully qualified by this initial release. It implements the desktop foundation, STEP setup, collision checking, growing-material simulation, FEM core, curved slicing and controller adapters. Production claims require reference Fusion studies, larger complex-part coverage / repair fixtures, native Duet firmware checks and calibrated hardware trials.

Release checks passed: 33 core cases, one native desktop workflow case, eight Python flow cases, and five native Klipper batch cases. Init / cleanup rows in Qt's logs are not included in those counts. The reference bed and head jobs pass semantic replay. The torus fixture is blocked by its report. The larger curved-stress fixture generates approximately 26,000 coordinated movements and reaches the 180-second validation budget; its saved preview is explicitly unvalidated. See `examples/reports/benchmarks.json` and the individual logs.

Features still requiring implementation beyond this alpha: global arbitrary-topology decomposition and deformation optimization, full Orca geometry / configuration / slicing integration, exact material coverage and guaranteed removable supports, richer machine-link attachments for imported rotary-stage bodies, controller-specific saved configuration editing, and machine-specific modeled startup / shutdown sequences. Current code uses connected isosurface charts, bounded region-orientation retries, simple support columns and sampled coverage.
