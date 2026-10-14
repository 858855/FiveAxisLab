# FiveAxisLab 0.4.4 — slicing corrections, surface-mesh planar slicing and parallel validation

This release corrects slicing and output defects found by comparing the slicer with OrcaSlicer and by slicing two user STEP parts (a 112 × 82 × 271 mm twisted part and 3DBenchy) in every mode.

## Slicing corrections

- **Top and bottom shells follow every exposed surface.** Shell layers were assigned only at the part's overall top and bottom, so shelves, steps and overhang undersides were printed with sparse infill. Each infill point is now classified by probing the solid above and below it.
- **Walls are placed as in Orca.** Planar walls are exact polygon offsets of the layer outline, with the outer wall centreline half a line width inside the contour. Previously wall positions came from a path-length distance on a coarse volume mesh; a 20 mm test part printed about 0.2 mm oversize with 0.09 mm waviness. It now measures 19.550 mm between outer wall centrelines, as expected.
- **Planar layers are cut directly from the surface mesh** at mid-layer, like Orca. Only curved layer fields need the tetrahedral volume. CAD files whose tessellation the volume mesher rejects (3DBenchy contains tiny self-intersecting triangles) now slice in planar XYZ, rotary-bed and tilting-head modes; curved modes report the meshing error.
- **Travel** lifts only when a move leaves the part or changes layer, along the nozzle axis. Straight segments are no longer cut into 0.7 mm pieces unless nozzle orientation or bead height changes along them (the 271 mm part went from 1.38 M to 275 k moves).
- **Supports** are continuous lines instead of isolated dashes, and overhangs are probed along each bead. One skirt is drawn around all islands of a layer.
- XYZ non-planar extrusion uses the interpolated bead height. Its rejection message now explains that the mode needs one top surface above every point.

## Motion and G-code

- **Feedrates match the requested speeds.** Each short segment was timed as a stop-start move, which capped printing near 2·√(length/acceleration), about 11 mm/s in the shipped reference G-code. Segments are now planned at cruise speed and acceleration is left to firmware look-ahead; the workspace shows an acceleration-aware time estimate next to the cruise time.
- **Single-object G-code heats before printing** (M140/M104/M190/M109), enables part cooling after the first layer and switches heaters off at the end. Nozzle and bed temperature and fan are set in the filament panel. Machine start/end scripts may contain heater, fan, acceleration and message commands; a start script may home with G28, after which the export re-enters the modeled pose from above.

## Import

- Production 3MF files (Orca, Bambu Studio, PrusaSlicer) that keep meshes in `3D/Objects/*.model` parts load completely.
- Orca line widths given as a percentage of the nozzle diameter are accepted; `small_perimeter_threshold` is treated as a radius, as in Orca.
- STL, OBJ and 3MF vertices are welded at 1e-5 mm; STEP seams keep the 1e-7 mm tolerance.

## Performance

- **Collision validation runs in parallel** on all hardware threads but two. Steps are processed in blocks against the material committed before each block plus staged earlier material of the block, so results match a sequential pass exactly (tested). 3DBenchy with an Orca-like profile validates in 19 minutes; the previous single-thread run had not finished after 3.5 hours.
- Consecutive collinear bead pieces are grouped behind exact group bounds in the broad phase (about 20% less validation time, identical results).
- Stress-field lookups and layer surface queries use spatial indexes; stress-guided slicing of the 271 mm part completes in about 7 minutes instead of over 80 CPU-minutes.

## Compatibility

Projects from earlier versions load unchanged. Heater and fan settings are written only when changed and are excluded from the plate planning signature, so saved plates keep their toolpaths current. STEP imports are not re-tessellated.

## Evidence

Recorded suites under `examples/reports/0.4.4-*.xml` pass: 62 core, 34 plate, 15 desktop, 7 machine and 18 STEP mesh entries, plus 11 Python controller tests (147 counted entries including Qt setup/cleanup). New tests cover shell detection on stepped parts, wall offsets, surface-mesh slicing of a self-intersecting mesh, export heating and feedrates, Orca width import, production 3MF parts, parallel/sequential validation equivalence and saved-plate signature stability.

`examples/reports/0.4.4-validation.json`: the CLI validates and semantically replays the qualified sequential plate fixture across 13,307 checked intervals. Reports retain `nativeFirmwareValidated: false` and `physicalPrintingValidated: false`.

## Known limits

- GPU backends do not accelerate collision validation. Profiling shows the remaining work is exact mesh-to-bead distances near the nozzle; a perfect pre-filter could skip about a third of them, and the RTX 5090 runs the required FP64 arithmetic at 1/64 of its FP32 rate.
- Curved modes generate about 2.4 times the support columns of planar slicing on the 271 mm test part; the cause is not yet identified.
- Validating very large parts remains slow (millions of collision queries even in parallel).
