# Orca-style desktop workspace, version 0.3

The ten supplied screenshots informed the native Qt layout and the process-control inventory. FiveAxisLab uses its own machine profiles and research engine. It does not connect to Bambu printers or provide a physical Print button.

**Prepare** shows the original part and a metric grid. **Preview** shows the articulated toolhead, deposited material, toolpath lines, collision markers and the motion timeline. The left sidebar contains the controller, machine layout, explicit dummy-toolhead option, one filament, presets, search, an Advanced switch and five process tabs: Quality, Strength, Speed, Support and Other. Machine and FEM tools remain accessible from the navigation and menus.

The bundled **0.30 mm / 0.6 mm nozzle / screenshot reference** preset reproduces the implemented numeric settings visible in the references. It is a research preset, not an X1 Carbon calibration. Requested print speeds are capped by the selected machine's motion and flow limits. Load accepts flattened Orca JSON or FiveAxisLab JSON; Save writes the complete native process. Invalid width/height combinations remain visible and block generation and export until corrected.

## Controls that affect generated geometry and motion

| Tab | Implemented controls |
|---|---|
| Quality | First/default heights; nozzle diameter; default, first, outer, inner, top, sparse, solid and support widths; seam placement; bounded path simplification; XY outer/hole offsets; first-layer elephant-foot correction; precise final scalar-field height; wall order; infill-first scheduling |
| Strength | Wall count; alternating extra wall; top/bottom shell counts and minimum thickness; top/bottom density; rectilinear, monotonic and concentric surface paths; sparse density; stress-aligned, rectilinear and grid infill; direction; stress-guided curved layers; deposition tilt and search cone |
| Speed | Default, first, first-infill, wall, sparse, solid, top, support and travel speeds; small wall percentage or absolute cap and diameter threshold; sampled overhang slowdown; Z hop |
| Support | Normal, Reduced or No supports; support-column pitch; reduction multiplier; angle-based candidate filtering; bed-only routing filter; removal of isolated columns; geometric bridge-span limit |
| Other | Skirt loops, layer count and distance; automatic, outer, inner or combined brims; width and object gap |

Zero role widths, speeds and first-layer height inherit the default. First-infill speed overrides first-layer speed for fill strokes. Small-wall threshold zero disables that optional slowdown. Overhang speed zero preserves the ordinary speed. Minimum shell thickness is rounded up to whole nominal scalar-field layers; the first and last layers can have different heights.

XY offsets use Boost.Geometry polygon buffers and preserve the surface tangent normal. Offsets that collapse or split a contour block slicing. Positive contour/hole values enlarge the relevant enclosed boundary. Elephant-foot correction shrinks the outer first-layer contour and enlarges holes. This is a geometric approximation on a curved chart and does not establish dimensional calibration.

Monotonic fill orders strokes consistently across a surface chart. Concentric fill follows interior distance contours. Shells refer to the beginning/end of the volumetric layer sequence; fully general locally exposed shells in changing topology remain unqualified. Automatic brim uses the part's height relative to its XY extent. Skirts currently surround individual chart outlines rather than the combined hull of multiple objects.

Support-angle filtering probes potential lower material around a candidate. Bed-only mode removes columns whose route intersects the part; isolated-overhang filtering removes unconnected candidate columns. These are research column heuristics, not Orca's tree-support algorithms. Support curves and part curves are scheduled together in layer order. Reduced and No supports never suppress deposition violations or bypass export blocking.

## Pictured algorithms that remain unavailable

Disabled fields show a reason in their tooltip and do not alter settings or generation. These include mixed-color sublayers and shell colors; scarf/wipe algorithms and overhang-aware seam relocation; gap closing, G2/G3 arc fitting, automatic hole correction, ironing and Arachne; embedded/thin walls; specialized bridge/counterbore flow and regional single-wall controls; wall-crossing routing and regional/Z speed smoothing; vertical-shell speed classification; tree supports, rafts, support interfaces and their densities/gaps; multi-material prime towers and wipe-into-infill.

These fields record the requested UI inventory. They are not implemented algorithms. Full Orca configuration and slicing integration remains outstanding. Changing an unsupported key in an imported Orca preset produces an explicit diagnostic; unavailable enum patterns or unsupported numeric formats reject import. Inherited Orca presets need flattening first.

## Preview and evidence

Version 0.3.2 adds **Strength → Nozzle perpendicular to profile**, **Process → Import profile centerline guide JSON** and **Simulation → Open spiral five-axis test**. The active cross-section and nozzle-normal error appear in Preview. [Generation behavior and limits](PROFILE_NORMAL.md).

Version 0.3.1 adds the **Axis behavior** inspector: recorded motion, manual poses and animated tilt/roll, with calibrated rotary-axis markers. Manual/animated inspection freezes deposition, leaves planned data unchanged and blocks export. [Controls, layouts and demonstration projects](AXIS_PREVIEW.md).

Line-type checkboxes filter both beads and paths. Two vertical sliders select a layer range; the upper boundary seeks to that layer's last motion. These are display filters and never change collision checking. Horizontal scrubbing and stepping restore the deposited prefix, including backward movement. Hiding the toolhead does not remove it from the collision scene.

The inspector derives time, filament length, mass and material cost from the trajectory. Density is in g/cm³ and cost in EUR/kg; estimates exclude heater warmup, cooling pauses and calibrated firmware overhead. Filament name/density/cost are stored in projects and are independent of geometric validation. FEM material properties are edited separately in Strength / FEM.

The movement pane displays rounded controller coordinates, relative E, controller feed metric and requested process speed. Planned motion remains labeled as a preview. Exported replay passes through the selected controller adapter and then geometric validation; a coordinate display alone is not controller verification. Duet firmware and physical printing remain untested.

STEP import provides linear deflection in millimetres, angular deflection in radians, separate rendering tolerance, Compound/CompSolid splitting, component identities, source units and a live facet count. Tessellation runs in a cancellable worker; changed settings generate a new cache key. Collision meshes retain their own error allowance. The part importer combines solids into one globally configured printable object; per-object process overrides are not implemented.

Native desktop tests exercise all five tabs, the bundled preset, input rejection, filters, dummy geometry, controller replay, STEP parameters and two window sizes. Screenshots and test reports are in `examples/reports/0.3.0-*` and `workspace-*.png`. The S-curve example is a synthetic model sliced with the screenshot preset; it is saved as unvalidated planned motion with blocked export. It illustrates geometry/toolpaths, not qualified continuous curved-layer or physical printing evidence.
