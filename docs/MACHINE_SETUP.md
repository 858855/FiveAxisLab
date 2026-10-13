# Printer settings and machine setup

Open **Machine setup** from the top bar or Machine menu. The editor follows the supplied Orca printer-settings layout: a profile name, save/load buttons, search, an Advanced switch, and underlined tabs. English labels and the existing machine configuration are used; the Bambu example values and proprietary scripts in the screenshots are not installed as printer defaults.

Basic view shows **Basic information**, **Motion capability** and **Extruder**. Advanced adds **Machine G-code**, **Notes**, **Axes / STEP** and **Collision**, and reveals additional fields in the basic tabs. Settings and source assets travel with projects and `.5machine` profiles. Cancel discards the entire edit, including changes in nested bed, tool and calibration dialogs. Saving a legacy single-part setup migrates it into the existing one-object, one-material, one-tool plate model and requires fresh slicing and validation.

## Printable space

**Printable area → Set…** opens a bed editor with rectangular, circular and custom polygon shapes, dimensions, origin, a metric preview, texture import/removal and STEP/STL/OBJ bed geometry. The origin values specify the distance from the lower-left bed bounds to machine XY zero. A circular bed has one diameter; its X/Y dimensions stay equal. Custom polygons use `XxY, XxY, XxY` coordinates in millimetres, for example `0x0, 256x0, 256x256, 0x256`. Self-intersections and degenerate outlines are rejected.

Textures are embedded in the project and displayed on the plate in Prepare and Simulation. They are visual assets. Bed meshes are collision geometry; imported models must use calibrated plate coordinates. A generated bed replaces the existing named Bed fixture; other fixtures remain in the scene.

Enable **Enforce printable area** to constrain deposition to the outline and printable height. **Excluded bed area** accepts polygons in the same coordinate format, separated by semicolons. Interval checks include bead footprint and rotary motion and can detect a thin exclusion between movement endpoints. An interval that cannot be certified remains blocked. Printable height also sets the Z travel maximum. Preferred X/Y placement is normalized from 0 to 1 and affects newly imported objects; existing placements are preserved.

## Motion and extrusion

XYZ/E speed and acceleration limits affect planned durations. Extrusion and retraction acceleration limits constrain their corresponding moves. Rotary speed, acceleration, limits and pivots remain separate in **Axes / STEP**. Firmware configuration templates contain the saved limits: Duet M203/M566 values use mm/min; Marlin speed and classic jerk values use mm/s. Classic jerk is kept as a firmware setting and is not mapped to Klipper square-corner velocity.

Select the physical tool in **Extruder**. Each tool stores its extruder type, nozzle volume and retract settings. Nozzle diameter and local bead height/width limits refer to its installed nozzle. Diameter edits fill both minima at half the diameter and both maxima at twice the diameter when auto-fill is enabled. Limits remain editable; disabling auto-fill preserves overrides on later diameter edits. Imported CAD is not resized. Diameter or TCP changes require installation calibration.

TCP offsets are applied in the canonical carriage model, rather than duplicated in firmware tool offsets. **Tools, materials and change recipes…** opens the existing plate editor with per-tool assemblies, routes, installed-nozzle profiles, docks, material assignments and recipes.

Enable **planned retraction** to insert explicit E retraction, optional wipe, Z-hop, lifted travel, descent and recovery. Wipe distance and the percentage retracted before wiping are honored. Z-hop bounds are expressed in carriage Z. Hops follow machine Z in all layouts and receive the same complete-assembly collision checks as other travel. Retraction/recovery does not add deposited material or filament usage. Filament loading/unloading times and hotend/extruder change overheads contribute to corresponding scheduled events.

**Firmware retraction** remains unavailable: this release emits the explicit modeled E and hop sequence. It does not assume an unknown G10/G11 implementation or firmware-side Z-hop. Nozzle volume, air-filtration accessory and the traditional radius/rod/lid clearance dimensions are saved profile reference information. They do not replace the calibrated STEP or dimensioned dummy collision model, drive an unconfigured accessory, or automatically infer purge volume.

## Machine G-code

Separate editors store start, end, between-object, layer-change, timelapse, filament-change, pause and custom-template text. Scheduled hooks accept absolute `G0/G1` carriage travel with an optional positive `F` in mm/min, `G4 P…` milliseconds or `G4 S…` seconds, `M400` synchronization, and comments. The applicable rotary words are A/C for a bed or A/B for a head; XYZ mode rejects rotary commands.

Start, layer, object and filament hooks return to their recorded resume pose. End hooks may leave the machine at their final configured pose. These movements become trajectory samples and are collision checked, timed, simulated and exported through the controller adapter. Timelapse hooks run only when enabled. Pause and custom text are saved templates, not automatically scheduled actions.

Homing, heating/feeder macros, E words, expressions, Bambu proprietary commands and unsupported G-code are rejected during generation with the hook and line identified. Use the existing explicit tool/filament recipes for modeled state changes. No arbitrary hidden macro movement is silently certified. Relative E emits M83; absolute E emits M82/G92 E0 and accumulated E, with replay checking the same extrusion deltas.

## STEP and collision additions

Printable STEP imports stitch coincident face seams before volume slicing. If **Slice + validate** still reports boundary or non-manifold edges after stitching, check that the source contains closed CAD solids rather than open surfaces. The edge counts describe remaining topology defects; changing tessellation deflection does not fill holes. Older saved STEP projects receive the same normalization during volume meshing. See [0.4.3 evidence](RELEASE_0_4_3.md).

**Axes / STEP** selects each physical assembly, imports STEP, switches its dummy option and opens the component/TCP/link/contact calibration preview. Calibration uses that tool's installed assembly, including a separately mounted nozzle. Rotary pivots, machine travel bounds and controller configuration share the same machine transforms as simulation.

**Collision** sets clearance, chord accuracy, angular subdivision accuracy and workpiece-versus-machine-body checks. Parked tools, docks, completed objects, supports, purge structures and accumulated material retain the existing collision rules. Changing any planning input invalidates motion certificates. Machine setup saves trigger a fresh slice when a trajectory already exists.

Geometric validation and controller replay remain distinct from native firmware and physical printer qualification.
