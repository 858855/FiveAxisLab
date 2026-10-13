# Plate jobs, materials and installed tools (0.4.0)

Open **Plate / tools** to enable the plate workflow. Projects contain stable object, material and physical-tool IDs. Choose **Layer-by-layer** or **Complete objects**. Drag the object IDs into a manual order, or enable bounded automatic order search. Slicing does not rearrange object placement.

The Objects table edits placement, default material and tool override. **Edit selected object's process override** opens the same Quality / Strength / Speed / Support / Other controls as the main sidebar. Objects inherit the plate process by default. The installed nozzle diameter belongs to the physical tool; changing a process preset does not resize imported CAD.

## Materials and assignment

Materials specify filament diameter, print/standby temperature, bed temperature range, cooling, maximum flow, density, cost, display color and optional elastic properties. The shared bed temperature must satisfy every configured material. Missing mechanical properties prevent directional optimization of a mixed-material object. Elementwise material FEA uses a **perfect-bond, dense-material assumption**; thermal incompatibility, adhesion and interfacial failure are not predicted.

Assignments are evaluated in object coordinates. Explicit volumes override painted surface regions, which override role and default object assignments. The priority field breaks ties within a category. Paint applies to walls and skin; the default material fills the interior. In Prepare, choose a material and **Brush** or **Connected fill**, then click the surface. Undo restores the previous assignment set. Volume modifiers import closed geometry. The Assignments editor also exposes exact assignment JSON, including tool overrides.

Support-body and support-interface materials are configured separately. Interfaces are classified geometrically near the model, using the configured number of interface layers. This is a research approximation, not a soluble-support adhesion guarantee.

Paths split at volume intersections and sampled painted boundaries. Generated material fragments smaller than the available nozzle are rejected rather than dropped. Tool selection checks filament routing, local layer height, width and flow, and balances nozzle quality against change time. Explicit tool overrides take precedence. Objects retain a common layer sequence across their tools. The finer nozzle usually supplies walls; wider compatible nozzles may supply infill.

## Physical tools and nozzles

Each Tools entry has its own STEP or dimensioned dummy geometry, CAD TCP, nozzle axis, mounting frame, moving-link assignments, installed nozzle, extruder/heater mapping and dock transform. Imported and dummy tools can coexist. The component table includes every physical component by default; visual-only components are an explicit choice. Select exactly one physical nozzle component.

The nozzle may be a component in the toolhead CAD, a separate STEP assembly, or a parameterized dummy nozzle. Separate/dummy nozzle mounting is independent of the existing assembly geometry. Enter usable bead height/width ranges and maximum flow, then confirm the actual installed geometry and calibration. A nozzle diameter change during a print selects another installed physical tool. Hand replacement is a setup change requiring new generation and validation.

Filament slots sharing a nozzle reference the same tool ID. Its first confirmed route identifies the filament assumed loaded at job start. Additional routes require explicit selector commands and load/unload distances. Arbitrary proprietary feeder macros are not interpreted.

## Storage and invalidation

The `.5axis` ZIP manifest uses schema 2. Source STEP bytes are embedded in a hash-indexed asset dictionary, with identical assets stored once. Meshes, placements, object process settings, FEM, recipes, profiles and events round-trip. Schema-1 files migrate to one object, one material and one tool; their previous validation is discarded.

Generation fingerprints cover geometry, assignment, calibration, profiles and settings. Changing these inputs blocks validation of stale paths. CPU thread/backend selection is excluded from geometry certification. A material/property change invalidates the associated structural solve. Save a fresh project after generation and complete-job validation.

See [tool changes and sequential collision checks](TOOL_CHANGES.md), [compute execution](COMPUTE.md), and the [release evidence](RELEASE_0_4.md).
