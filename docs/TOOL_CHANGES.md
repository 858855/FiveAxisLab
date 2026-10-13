# Tool changes, sequential printing and replay

Tool changes are explicit job events: retract, leave the part, approach/release the outgoing tool, enter an empty-carriage state, pick up the incoming tool, confirm the latch, select material, heat/load/purge, return and resume. Each parked hotend retains its own loaded material and retraction state. The outgoing tool receives its material's standby temperature. Returning to a parked tool heats it and restores its retraction.

## Calibrated recipes

Pickup/release recipes use canonical carriage XYZ in millimetres and rotary angles in radians. Each contains motion waypoints, bounded named contact allowances, I/O, synchronization and a controller sensor guard. The advanced JSON editor shows the saved representation:

```json
{"kind":0,"carriage":{"xyz":[-60,0,40],"a":0,"b":0},"argument":"","seconds":0,
 "component":"coupler","fixture":"tool:tool-1/latch","center":[0,0,0],"size":[1,1,1]}
```

Event kinds in recipes are **0 motion**, **3 sensor confirmation**, **9 synchronization**, and **10 I/O**. Attach/detach events are synthesized by the planner. Recipe calibration requires both pickup and release guards. Guard examples are `M577 ...` for Duet, `FAL_TOOL_CONFIRM SENSOR=latch PRESENT=1` for Klipper, and `M226 ...` for qualified Marlin configurations. Names and pin states must match the actual machine.

Import the shared carriage/coupler STEP in the Purge and carriage tab. Empty-carriage motion cannot be validated without its geometry. The dock transform locates the parked CAD origin in machine coordinates. Attach/drop poses must reproduce that transform without a geometry teleport. The active TCP is derived from canonical carriage coordinates, so selecting another TCP creates no carriage jump or double offset compensation.

Contact allowances apply only to the specified component pair near a declared recipe waypoint. `tool:ID/component` targets either the active partner or its parked representation; fixture names target modeled docks. The box belongs to the first component's CAD frame. All other contacts remain checked. Model latch/selector mechanisms conservatively over their full I/O stroke when their articulated motion is not represented.

## Purge and waste

Specify transition volumes in mm³. Missing material transitions block planning. The tower completes each generated layer and reports actual waste, including extra volume required to finish a layer. Its entire footprint must lie on the modeled bed, and it maintains a shared deposition history across complete objects.

A purge station requires a fixed fixture named **purge-waste-envelope**, representing the full occupied collection envelope, plus explicit waste capacity. The planner rejects jobs exceeding that capacity. The envelope participates in collision checks from the beginning; this intentionally models the worst collection state. Station purge events contribute to material/waste usage. The simulation distinguishes these E-only events from deposited tower beads.

## Whole-plate checks

Completed objects, their supports and tower beads remain obstacles during later printing, travel, docking and unwinding. On a rotary bed they move with the plate and are checked against fixed bodies and parked tools. Only the active object's nozzle contact region receives deposition allowance; neighboring objects do not.

Collision workers own private FCL scenes and restore the deposition prefix at their batch start. Conservative bounds prune candidate pairs; exact geometry/containment and adaptive trajectory checks remain on CPU. Reports merge in movement/component order. A bounded report retains conflicts and blocks export when the collision-event limit is reached.

Travel repair uses deterministic joint-space RRT-Connect, followed by continuous checking and complete-job revalidation. A single extrusion-region orientation retry is allowed where one nozzle calibration permits a complete deposition recheck; profile-normal regions keep their constraint. Automatic object ordering evaluates bounded deterministic candidates without moving parts and selects the lowest time/waste cost among validated candidates. Exhaustion means **no validated order found**, not proof that printing is impossible.

## Controller adapters

- **Duet:** planned movements plus `Tn P0` / `T-1 P0` suppress firmware tool-change files. Configure tool drives/heaters using the generated template. Firmware tool XYZ offsets are zero; FiveAxisLab applies TCP in carriage coordinates. [Tool-selection semantics](https://docs.duet3d.com/User_manual/Reference/Gcodes/T).
- **Klipper:** tools map to explicit extruder/heater names. Install `fiveaxis_tools.py` and, for rotary-aware flow, `fiveaxis_flow.py`. `FIVEAXIS_ENABLE EXTRUDER=...` switches the guarded extruder and restores the previous native checker. The tools extension checks actual configured switch sensors and maps feeder slots to one configured output-pin write. [Extruder activation](https://www.klipper3d.org/G-Codes.html#activate_extruder).
- **Generic/Marlin XYZ:** shared-nozzle selectors use configured non-motion I/O. Detachable tools require explicit pin guards, zero firmware offsets and qualified `Tn S1` semantics. Unknown macros are rejected. [Marlin tool selection](https://marlinfw.org/docs/gcode/T.html), [pin guards](https://marlinfw.org/docs/gcode/M226.html).

Exported replay verifies the generated startup, offsets, state commands, heaters, waits, extension configuration, identities, carriage axes, relative E and timing. Unknown commands or changed configuration block complete-job agreement. Pressure advance stays disabled. Heater waits use estimated preview durations; actual waiting time and physical sensor reliability require machine testing.

Simulation colors paths by role, object, tool or material. Blue timeline ticks mark state changes; red ticks mark collision reports. Scrubbing restores active/parked tools, material, completed objects and deposited geometry. Playback speed and rendering do not control validation.
