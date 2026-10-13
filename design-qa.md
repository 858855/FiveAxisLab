# Machine setup visual QA

final result: passed

Native Qt implementation: `src/app/machine_dialog.cpp`, `src/app/toolhead_dialog.cpp` and `src/app/viewport.cpp`. Compared the user's Orca/Bambu screenshots against actual Qt widget captures in the same visual inspection calls. This is the existing Windows desktop application, not a browser prototype.

## Reference and viewport

The ten reference images are the user's `codex-clipboard-aaf9cac3-*` through `codex-clipboard-fc5b26ab-*` screenshots in the Windows temporary folder. Key comparisons used `4f63f065-*` for advanced Basic information, `2e4091df-*` for motion, `31535c28-*` for bed shape and `d45fb675-*` for Extruder. Screenshots include OS title bars; Qt captures show client content. Compare normalized client frames rather than treating title-bar or display-DPI differences as layout defects.

Main editor: 1140 × 900 logical pixels, captured at Windows 175% display scaling as 1995 × 1575 pixels. Bed dialog: 1050 × 720 logical pixels. Qt Fusion styling matches the shipped application's style. Native widgets preserve keyboard focus, associated labels, checked-state marks, scrollbars and tab scrolling. The 860 × 600 logical-pixel compact render was also inspected; tabs and long settings pages scroll, and search jumps to the matching tab.

## Intentional adaptations

English labels and the user's actual machine profile replace the German Bambu example. Machine/controller and kinematics controls remain available in Basic view. Advanced adds Axes / STEP and Collision to the five pictured tabs. Per-tool selection, explicit nozzle width/height bounds and XYZ TCP offsets add rows to Extruder, so long pages use vertical scrolling. Firmware retraction is visibly unavailable with an explanatory tooltip because the certified trajectory contains explicit E and hop movements. Traditional clearance/accessory properties are marked as profile reference information.

## Iteration and fixes

The first render had a growing green bed-action button, oversized number fields and filled boxes without clear check marks. The final implementation disables default-button styling on ordinary actions, caps number-field width, uses a compact bed action, restores native checkbox check marks, adds an explicit Advanced switch, and exposes visible scrollbars. Bed polygon labels hide with their fields in non-polygon modes. A scoped light palette and explicit checkbox indicator rendering fix dark-system checkbox backgrounds while retaining Qt checkbox interaction; the final bed button width is fixed after form sizing.

Checked layout/group hierarchy, white surfaces, green active-tab underline, input alignment, readable units, wrapped labels, profile toolbar, Basic/Advanced visibility, source versus dummy identity, editable automatic nozzle limits and bed shape/origin preview. No unresolved P0/P1/P2 visual findings for the agreed native adaptation.

## Evidence

`examples/reports/machine-basic.png`, `machine-advanced.png`, `machine-motion.png`, `machine-extruder.png`, `machine-gcode.png`, `machine-bed.png`, `machine-axes.png`, `machine-compact.png` and `machine-bed-texture.png` are native renders. The desktop suite exercises tab visibility, nested bed editing, diameter auto-fill/manual override, cancel rollback and saved settings. The texture test reads the actual OpenGL framebuffer and verifies texture-colored pixels rather than relying only on a configuration flag.

All final suites pass: 55 core, 33 plate, 15 desktop, 7 printer-settings and 11 Python checks. Qt totals include setup/cleanup. Native firmware and physical printing remain separate qualifications.
