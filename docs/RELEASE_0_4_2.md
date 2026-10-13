# FiveAxisLab 0.4.2 — Printer settings

Machine setup now uses the supplied Orca printer-settings structure with Basic and Advanced views, profile save/load, settings search, Basic information, Machine G-code, Motion capability, Extruder and Notes. Axes / STEP and Collision retain the research machine additions. The bed editor supports rectangle/circle/polygon boundaries, exclusions, an origin, embedded textures and collision-model import.

Per-axis timing, per-tool retraction/wipe/Z-hop, scheduled modeled travel/dwell/synchronization hooks and relative/absolute extrusion participate in generation and replay. Nozzle limits retain editable automatic half-to-double bounds. Traditional accessory/clearance reference information is persisted, with the detailed behavior and remaining firmware-retraction boundary recorded in [Machine setup](MACHINE_SETUP.md).

Validation evidence is recorded under `examples/reports/0.4.2-*`, with native screenshots under `examples/reports/machine-*` and visual comparisons in `design-qa.md`. Hardware and physical-print qualification boundaries from 0.4.0/0.4.1 remain unchanged.
