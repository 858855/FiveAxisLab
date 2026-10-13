# Standard XYZ non-planar printing

Choose **Kinematics → Standard XYZ · 3-axis**. In **Strength → Standard XYZ · non-planar**, enable **Conform layers to top surface**. XYZ follows curved layers while the nozzle stays upright at +Z and both virtual rotary positions remain zero. Select **Generic / Marlin · XYZ**, Duet or Klipper for output. This mode requires no rotary motors or rotary-aware Klipper extension.

Use **Simulation → Open 3-axis non-planar test** for a ready-generated example, then Play in Preview. The HUD reports Standard XYZ and a fixed nozzle. Rotary pose controls and rotary markers are disabled. Timeline scrubbing restores the deposited prefix. The movement pane shows XYZ/E rather than rotary words.

**Planar base height** keeps the bottom portion flat. Above it, a scalar field progressively morphs layers toward the highest model surface at each XY position. **Maximum surface slope** rejects steeper deposition surfaces rather than rotating the nozzle. Defaults are a 2 mm planar base and a 20 degree slope limit; these are research defaults, not a hardware clearance qualification. Disabling conformal layers produces ordinary planar XYZ paths.

Cells intersecting the requested base remain fully planar. Coarse volume meshes can therefore extend the flat region beyond the requested height; if that region spans the entire part, generation remains planar. The transition does not warp the requested base or reverse build progression.

The layer's surface normal is stored separately from the actual nozzle axis. Local normal spacing is derived from the tetrahedral field and used for bead height and extrusion along the actual 3D path. Rendering, growing-material collision geometry and deposition checks use that same surface normal. The machine/toolhead remains rigid and upright. Nominal layer height parameterizes the scalar field; local bead thickness can be smaller in the conformal portion.

Import and calibrate the complete STEP toolhead, or use **Generate with dummy toolhead** for synthetic geometry. Nozzle, heater block, fan shroud, fixtures and deposited material remain in the collision scene. A slope limit alone cannot establish clearance: protrusion length, body width, modeled fixtures and previously deposited material all matter. Travel repair searches three Cartesian coordinates; it cannot tilt the nozzle to escape a conflict. Existing Normal / Reduced / No supports controls and deposition export gates remain in effect.

Set the actual XYZ minimum/maximum in Machine setup and model the bed/fixtures in the same coordinate system. The included research fixture uses a centered XY origin. New XYZ part imports are centered within the configured XY travel limits and placed at Z=0; reimport or reposition geometry after changing those limits. This lets a conventional front-corner origin and a centered origin use their own calibrated profiles.

Generic/Marlin output uses G21, G90, M83, G1 XYZ/E/F and M400. Duet XYZ output omits rotary axis mapping. Klipper XYZ output uses stock synchronized XYZ/E and stock extrusion constraints, without MANUAL_STEPPER, FIVEAXIS_ENABLE or FIVEAXIS_DISABLE. Controller replay rejects rotary words and rotary extension commands in XYZ mode. Heating, homing, pressure advance and machine startup must use the actual printer's calibrated procedures. No firmware or physical printer qualification is claimed by semantic replay.

```powershell
fiveaxis_cli.exe --nonplanar-demo --output nonplanar-xyz.5axis
fiveaxis_cli.exe --slice part.stl --three-axis --dummy-toolhead --output preview.5axis --plan-only
fiveaxis_cli.exe --slice part.stl --xyz-planar --dummy-toolhead --output planar.5axis --plan-only
fiveaxis_cli.exe --validate nonplanar-xyz.5axis --klipper --replay --report report.json
```

Use an imported native process JSON to set `nonPlanar3Axis`, `nonPlanarBase` (mm) and `nonPlanarSlope` (radians) from the CLI. Profile-normal five-axis slicing is a different mode and cannot be combined with fixed-nozzle XYZ generation. Switching between XYZ and a rotary layout clears stale paths and requires slicing again. Geometry, STEP assets, FEM and guide data remain in the project.

This implementation uses a top height field. Undercuts, cavities, branching shapes, steep walls, coarse meshes and very thin regions are not automatically qualified by conformal generation. Sparse infill, adhesion, heating, sagging and dimensional accuracy remain research limits. The default roof fixture passes the sampled deposition check, but its full collision-validation attempt was canceled before completion. Its project and report remain unresolved with blocked export. Functional tests and controller adapter fixtures do not constitute a complete-job collision certificate.

Version 0.3.3 evidence covers changing Z during extrusion, a planar base, local bead spacing/flow, upright toolhead transforms, slope rejection, project round trips, three controller adapters, unexpected rotary command rejection, three-dimensional travel repair, full heater-block collision detection and native workspace switching. Reports are in `examples/reports/0.3.3-*`.
