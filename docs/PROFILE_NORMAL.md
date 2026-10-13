# Five-axis profile-normal slicing

The spiral is a printing-path test, rather than an arbitrary rotary-axis animation. **Strength → Nozzle perpendicular to profile** slices the model with successive planes along an explicit centerline. The nozzle axis follows the normal of each plane, which is the local growth direction of the swept profile. No FEM result is required. This mode takes precedence over the stress-derived scalar field.

Open **Simulation → Open spiral five-axis test · rotary bed / tilting head**, then press Play in Preview. The active section is outlined in purple; the green line follows the nozzle axis. The HUD shows the nozzle-normal error during extrusion. Bed A/C or head A/B and compensated XYZ come from inverse kinematics, using the calibrated mounted nozzle axis. Follow toolpath displays the generated motion; manual axis inspection remains a separate, unvalidated pose mode.

Within each section, extrusion keeps both rotary positions fixed and the TCP in the section plane. Travel changes orientation between sections without extrusion. Generation, collision validation and export check the profile constraint; avoidance can use alternative inverse-kinematic poses but cannot tilt an extrusion away from its required normal. Joint limits, process tilt limits, deposition and collisions still apply. The supplied spiral projects have pending collision validation and blocked export; the deposition heuristic also identifies unresolved movements. Correct orientation alone does not qualify a print.

For an imported part, choose **Process → Import profile centerline guide JSON**, then slice again. A guide contains ordered section centers and normals in the source model's millimetres. The saved placement aligns both geometry and guide. Normals are normalized on import; zero normals, duplicate/reversed stations and unsupported units are rejected. Profiles must progress along their normals and stay within the configured tilt range. The guide chooses one connected section at each station; arbitrary branching shapes need separate guides/decomposition. Automatic centerline extraction is not implemented.

```json
{
  "units": "mm",
  "stations": [
    {"center": [0, 0, 0], "normal": [0, 0, 1]},
    {"center": [1, 0, 10], "normal": [0.2, 0, 0.98]},
    {"center": [4, 0, 20], "normal": [0.3, 0, 0.95]}
  ]
}
```

Layer spacing is measured along the guide's piecewise centerline arc, with interpolated normals. Endpoints define the sliced span. Guide detail and the source mesh determine how well the planes approximate a CAD sweep. Closely adjacent or overlapping sections and sparse guides are not automatically qualified. Existing deposition coverage and collision checks remain required.

The former `workspace-s-curve.5axis` was a horizontal-layer illustration. Version 0.3.2 regenerates it with `geometry/s-curve.profile.json` and `process/profile-s-curve.json`. That process disables the planar elephant-foot offset and brim: those settings split contours in its inclined first section. The source mesh is preserved; its flat end caps and inclined first section still require deposition/fixture qualification. Older reports remain archived.

```powershell
fiveaxis_cli.exe --spiral-demo head --output spiral-head.5axis
fiveaxis_cli.exe --spiral-demo bed --output spiral-bed.5axis
fiveaxis_cli.exe --slice part.step --profile-guide profile.json --dummy-toolhead --output preview.5axis --plan-only
```

Spiral-demo exit 0 means generation and saving succeeded; it does not authorize export. Native GUI, round-trip, calibrated-axis, playback and wrong-orientation rejection checks are recorded under `examples/reports/0.3.2-*`. Firmware and physical printing remain untested for these jobs.
