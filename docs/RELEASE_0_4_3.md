# FiveAxisLab 0.4.3 — STEP seam connectivity

Closed STEP solids previously failed **Slice + validate** with “Volume slicing requires a closed manifold mesh; repair boundary edges first.” Open CASCADE triangulates each face separately. The importer retained duplicate vertices along shared CAD edges and periodic seams, so the indexed mesh appeared open when the desktop passed it to the volume slicer.

Both collision/part and rendering meshes now share coincident seam vertices within **0.0000001 mm**. This numerical tolerance is independent of the rendering/collision deflection and does not fill missing surfaces or bridge gaps at the tessellation tolerance. STEP cache revision 4 prevents reuse of old unstitched imports.

The volume mesher also normalizes a private copy of its input. Earlier saved projects can be sliced without reimporting the CAD; their stored source meshes, painting identifiers and validation fingerprints are unchanged by this normalization. Generation still requires fresh collision validation. Actual open surfaces and non-manifold meshes remain blocked, with boundary/non-manifold edge counts reported after stitching.

Regression tests reproduce the original failure on closed boxes, cylinders and curved pipe segments at fine/coarse deflections. They cover cached imports, nested/repeated assemblies, inch units, legacy project round trips, numerical seam precision, large coordinates, rejected missing faces and narrow real gaps, invalid input, and direct desktop-style STEP meshes through single-object and plate slicing in XYZ, rotary-bed and tilting-head modes. Geometric and controller-replay evidence is separate from native firmware or physical printing evidence; the user's particular source STEP was not supplied.

Recorded suites under `examples/reports/0.4.3-*.xml` pass: 55 core, 33 plate, 15 desktop, 7 machine, 18 STEP mesh entries, plus 11 Python tests (139 counted entries including Qt setup/cleanup). The Windows package launches without the SDK on PATH. A separate packaged CLI check validates and replays the existing qualified sequential fixture; this replay evidence does not certify every newly imported model.
