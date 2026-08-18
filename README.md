# voxels3

Multi-scale binary voxels meshed with an adaptation of [Nielson's Dual
Marching Cubes](https://doi.org/10.1109/VISUAL.2004.28) (IEEE Vis 2004) and
rendered with [Bevy](https://bevyengine.org).

The adaptation removes the need for normals the same way Surface Nets
simplifies Dual Contouring — dual vertices are centroids of edge crossings
— while keeping DMC's key property: **one vertex per surface patch per
cell**, so thin and diagonal features keep correct topology instead of
pinching. Voxels are genuinely multi-scale (octree leaves at power-of-two
sizes); the mesher operates on the octree structure directly and produces
big polygons for big voxels with watertight transitions to fine ones.

## Crates

| crate | contents | deps |
|---|---|---|
| `voxel-core` | voxel data model: sparse octree with multi-scale leaves, 32³ chunks, sparse world, storage-agnostic tree view, demo shapes | `glam` |
| `voxel-mesh` | the DMC mesher, generated patch table, polygon/render mesh types, topology analysis | `voxel-core` |
| `voxel-bevy` | demo binary + debug renderers — the **only** crate that references Bevy | `bevy` |

## Run

```sh
cargo test --workspace     # meshing invariants: watertightness, Euler,
                           # winding, scale similarity, chunk seams ==
                           # single-domain reference
cargo run -p voxel-bevy    # demo scene
```

Which scene runs is `config::SCENE`. `DiagonalTree` (the default) and `Tree`
grow the same branching tree one finest-scale voxel at a time, running
diagonally to the voxel axes so nearly every surface is stair-stepped;
`Tree` fills in each diagonal step to keep the tree one connected surface,
while `DiagonalTree` leaves them, so branches break into 56 pieces touching
only along edges — which the mesher then joins back up, so the rendered
tree is whole apart from one corner-only contact it cannot see.
`TestBed` is a small hand-placed scene for examining one behaviour at a time
— currently one 4³ voxel with two 2³ voxels on its +x face, the minimal
octree scale transition — and `Landscape` is the full terrain-and-objects
demo. Add new experiments to `scene::test_bed`.

Demo controls: the cursor is captured — **mouse** to look, **WASD + QE**
(or Space/Shift) to fly, **scroll** to change speed, **F** to toggle
wireframe, **Esc** to quit.

Debug renderers are compile-time flags in `crates/voxel-bevy/src/config.rs`:
chunk bounds, the full voxel grid, a box around each voxel at its own scale,
a solid marker at each voxel center, boxes for the active *meshing* cells
(the dual cells spanning between voxel centers, so those centers sit on
their corners/edges/faces), and the **dual graph** — an edge between the
centers of every pair of face-adjacent leaves, with the crossed ones (each
of which becomes exactly one surface polygon) highlighted. `SURFACE_ALPHA`
makes the meshed surface translucent so the structure inside stays visible.

Automated render verification (screenshots frame 150, then exits):

```powershell
$env:CI_TESTING_CONFIG="crates/voxel-bevy/ci_screenshot.ron"; cargo run -p voxel-bevy
```

## How the meshing works

Voxels are volumes and their centers are the sample points. The meshing
cells are the octree's **dual grid**: one cell per octree vertex, its 8
combinatorial corners being the surrounding leaves (a coarser leaf fills
several corner slots). Per cell, a generated 256-case table groups the
crossed edges into surface patches — two crossed edges join when a face's
surface segment connects them, with ambiguous faces resolved so material
is joined through face and edge contacts (a pair meeting along an edge
meshes as one pinched surface; corner-only contacts stay separate). Each
patch gets one dual vertex at the centroid of its
crossings; a crossing sits at the center of the face the two leaves share
(the one interpolation richer voxel data would change). Using the shared
face keeps the surface independent of how *empty* space happens to be
partitioned, so an isolated voxel meshes symmetrically, a shape meshes the
same way wherever it sits in the octree, and the edge of the world behaves
like ordinary empty space. Each crossed leaf adjacency then becomes a polygon over the dual
cells along its shared face's boundary — quads in uniform regions, n-gons
at scale transitions (hanging vertices), which is exactly what keeps those
transitions crack-free.

The traversal is the classic four-proc octree recursion from dual
contouring (`cell_proc`/`face_proc`/`edge_proc`/`vert_proc`), with a leaf
standing in for its own children when its partners are finer. Chunk seams
run the same procs across chunk roots (a "super-octree" walk without
materializing one); a missing neighbour chunk is a canonical all-inactive
leaf, which also closes meshes at the edge of the world. Tests assert the
chunked result is *bit-identical* to meshing the same data as one big
octree.

Swappability: meshers implement `voxel_mesh::Mesher`, and voxel storage is
consumed through the `voxel_core::tree_view::TreeNode` trait, so other
algorithms (Surface Nets, …) and other storage (GridMaps, …) can slot in.
