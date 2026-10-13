# Tilt and roll in Preview

Open **Preview** and use **Axis behavior** in the right inspector. The **Rotary axes** checkbox shows or hides the red A and blue B/C rings, pivot stems, angle pointers and labels. **Fit view** includes these markers; drag to orbit and scroll to zoom.

| Machine layout | First rotary joint | Second rotary joint | Moving bodies |
|---|---|---|---|
| Tilting and rotating bed | A, rotation around calibrated X axis | C, rotation around the A stage's local Z axis | Bed, part, deposited beads and attached fixtures |
| Tilting toolhead | A, roll around calibrated X axis | B, tilt around the A stage's local Y axis | Components assigned to the A stage or final nozzle assembly |

Pivots, mounting transform, CAD nozzle tip and component attachments come from the active machine/toolhead profile. The displayed carriage XYZ includes mounted-tip compensation. Axis markers use the same core transforms as rendering, motion planning and collision checking. The synthetic dummy assembly is supported too.

**Follow toolpath** displays actual interpolated joint positions from planned motion or exported G-code replay. Play, scrub and step work as before. A planar path whose rotary positions are zero keeps the bed/head stationary. Directional slicing or a coordinated example supplies changing rotary angles.

**Manual tilt / roll** lets you drag either slider or enter degrees. This pauses normal playback and holds the current TCP point in the workpiece frame. XYZ moves to compensate the changed orientation. The stored path, settings, validation fingerprint and deposited prefix remain unchanged; backward timeline scrubbing returns to Follow toolpath and restores the recorded pose and prefix. Joint ranges use the profile limits; a requested pose beyond Cartesian carriage limits retains the last reachable pose and displays a message.

**Animated tilt / roll** sweeps both axes on a wall-clock timer. Animate/Play starts or pauses it; Zero axes selects a manual zero pose. The sweep amplitude and nominal angular rate are bounded by the profile, and each pose is checked for carriage reach. This animation is a pose illustration: it does not certify a continuous trajectory, acceleration at mode changes, extrusion or collision clearance. The HUD and movement pane say so, old contact markers are suppressed, and export stays disabled until Follow toolpath is selected. Saving a project preserves the original planned motion. Saving a preview image retains the currently inspected pose.

Open **Simulation → Open rotary-bed axis demonstration** or **Open tilting-head axis demonstration** for an immediately playable example. Both use synthetic geometry, contain travel moves with zero extrusion and are deliberately unvalidated with blocked export. The included `examples/axes-bed.5axis` and `examples/axes-head.5axis` are equivalent files. They can also be generated with:

```powershell
fiveaxis_cli.exe --axis-demo bed --output axes-bed.5axis
fiveaxis_cli.exe --axis-demo head --output axes-head.5axis
```

Version 0.3.1 tests cover calibrated pivot/axis frames, fixed workpiece TCP compensation, both recorded sweeps, limits, project round trips, native manual/animated controls, timeline restoration and export gating. Screenshots and reports are in `examples/reports/0.3.1-*` and `axes-*.png`. These are native GUI and geometric evidence. No physical machine or new firmware qualification is claimed.
