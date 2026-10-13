# FiveAxisLab 0.4.1

Changing an installed nozzle diameter now auto-fills the minimum and maximum local bead height and width. Both minima are half the diameter and both maxima are twice the diameter, as requested: a 0.8 mm nozzle fills 0.4–1.6 mm; a 0.6 mm nozzle fills 0.3–1.2 mm.

All four limits remain editable. Disable **Auto-fill bead limits when nozzle diameter changes** to retain custom values through later diameter changes; re-enable it to restore the calculated ranges. The preference and edited values are saved per physical tool in projects and machine profiles. Loading or switching tools preserves their stored limits. Existing projects retain their ranges and default to auto-fill for the next diameter edit. Imported CAD and TCP still require calibration confirmation after a diameter change.

The new desktop workflow covers automatic filling, editable overrides, disabling/re-enabling auto-fill, switching between two tools, the maximum supported diameter, calibration invalidation, project round trips and older schema-2 files without the new preference. Native captures are in `examples/reports/nozzle-limits-auto.png` and `nozzle-limits.png`.

Verification: 55 core, 33 plate, 13 native desktop and 11 Python checks passed (112 total, including Qt initialization/cleanup; zero failures or skipped rows). Machine-readable reports are `examples/reports/0.4.1-*.xml`. Hardware qualification remains as described in [the 0.4.0 evidence](RELEASE_0_4.md).
