# voxels3

Multi-scale binary voxels meshed with a normal-free adaptation of Nielson's
Dual Marching Cubes, rendered with Bevy.

## Hard rules

- **Bevy isolation**: `crates/voxel-bevy` is the only crate allowed to
  reference Bevy. `voxel-core` and `voxel-mesh` must stay engine-agnostic —
  the dependency graph enforces this; keep it that way.
- **Multi-scale is fundamental**: voxels are octree leaves with power-of-two
  sizes and the mesher consumes that structure directly (big voxels → big
  faces, crack-free transitions). Never "flatten to finest and mesh
  uniform".
- **Flat shading**: normals are derived from geometry at render-buffer time
  (`MeshData::flat_from_polys`), duplicated per triangle. Meshing itself
  never computes or stores normals — that is the point of the algorithm
  choice.
- **Modularity**: meshers implement `voxel_mesh::Mesher`; storage is
  consumed through `voxel_core::tree_view::TreeNode`. New algorithms
  (Surface Nets, …) or storage (GridMaps, …) plug into those seams.

## Architecture

- `crates/voxel-core` — `Voxel` (binary for now; struct so richer data can
  be added), sparse `Octree` (split-on-write, merge-on-uniform, leaves at
  any power-of-two scale), 32³ `Chunk`, sparse `World`
  (`HashMap<IVec3, Chunk>`), `TreeNode` view trait, analytic `shapes` for
  demos/tests (`quantized` authors deliberately coarse voxels).
- `crates/voxel-mesh` — the DMC mesher (`dmc.rs`, see its module docs for
  the full construction), generated patch table (`patches.rs` — never
  hand-edit case data; it is derived and invariant-tested), `PolyMesh`
  (logical quad-dominant output), `MeshData` (flat-shaded triangles),
  `analysis` (weld/edge-stats/Euler/volume used heavily by tests).
- `crates/voxel-bevy` — demo binary. `config.rs` holds compile-time
  choices: `SCENE` picks a scene from `scene.rs` (`TestBed`, the small
  hand-placed scratch scene for testing one behaviour at a time — put new
  experiments there; or `Landscape`, the full demo), plus debug flags
  (chunk bounds, full voxel grid, per-voxel boxes/spheres at their own
  scale, active meshing-cell boxes via `dmc::collect_dual_cells`,
  wireframe). Fly camera captures the cursor: mouse to look,
  WASD+QE, scroll for speed, F for wireframe, Esc to quit.

## Algorithm cheat-sheet

Dual cells live at octree vertices; their 8 corners are the surrounding
leaves (a coarse leaf may fill several slots). Per cell the patch table
splits crossed edges into surface patches; one vertex per patch at the
centroid of its crossings (crossing = where the center-to-center segment
pierces the two leaves' shared plane; the single hook where scalar data
would interpolate differently). Surface faces are dual to crossed leaf
adjacencies: a polygon over *all* dual cells on the shared face's boundary
— hanging vertices included, which is what makes scale transitions
watertight. Traversal is the standard cell/face/edge/vert proc recursion,
run inside chunks and across chunk roots ("super-octree" seams); absent
chunks are a canonical empty leaf, which closes the world boundary.
Ambiguous faces separate diagonal actives: material is 6-connected, empty
space diagonal-connected.

## Workflow

- `cargo test --workspace` — the meshing suite's equivalence tests
  (chunked == single-domain) are the acceptance tests for any mesher
  change.
- `cargo run -p voxel-bevy` — demo.
- Automated render check:
  `$env:CI_TESTING_CONFIG="crates/voxel-bevy/ci_screenshot.ron"; cargo run -p voxel-bevy`
  screenshots frame 150 into the working directory and exits.
