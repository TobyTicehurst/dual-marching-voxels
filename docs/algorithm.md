# How the mesher works — a worked walkthrough

A concrete, numbers-included companion to the module docs in
`crates/voxel-mesh/src/dmc.rs` and `patches.rs` and the summary in
`README.md`. Those explain *why*; this explains *what happens, in order,
with real coordinates*, so a future session can reason about vertex
placement without re-deriving it. It describes the code after the joint
split of November 2026 (see §6); if the rules change, update this file.

Positions are in world voxel units: finest voxel = 1.0. A finest voxel at
integer position `p` occupies `[p, p+1)³` and its center is `p + 0.5`.

## 1. Inputs

- `voxel-core`: a `World` is a sparse `HashMap<IVec3, Chunk>`; a `Chunk`
  is a 32³ `Octree` whose leaves are voxels of power-of-two size. A
  uniform region is one big leaf. Absent chunks are `EMPTY_NODE`, a single
  all-inactive leaf, which is what closes the surface at the world edge.
- The mesher reads storage only through `TreeNode` (`children()`,
  `voxel()`), and carries position/size itself in `CellRef { node, min,
  size }`.

## 2. Dual cells (where the 256-case table runs)

A **dual cell** sits at every octree vertex `q` (an integer corner shared
by leaves). Its 8 combinatorial corners — "slots", indexed by octant bits
`x=1, y=2, z=4` — are the leaves touching `q` from each octant direction.
A coarse leaf fills every slot it touches, so a cell between scales is a
degenerate cube (several slots hold the same leaf). The cell's `mask` has
bit `o` set when slot `o` is active; cells with mask 0 or 255 produce
nothing.

Code: `Builder::vert_proc` → `emit_dual_cell` in `dmc.rs`.

## 3. Crossings

A cell edge joins two slots that differ in one octant bit — two leaves that
are face-adjacent across the plane through `q`. The edge is **crossed**
when their states differ. The crossing point is the **center of the
rectangle the two leaves share** (`shared_face` → `FaceKey::center`): the
whole face when they are the same size, the smaller leaf's face at a scale
transition. It is *not* the midpoint between the two leaf centers; using
the shared face is what makes the result independent of how empty space
is partitioned (an isolated voxel comes out symmetric, placement does not
matter, the world edge behaves like empty voxels). This is the single
place richer voxel data would change the interpolation.

`FaceKey { axis, rmin, rsize }` identifies an adjacency uniquely, so every
construct that refers to the same adjacency (a cell's crossed edge, the
polygon dual to it, a segment vertex) agrees on it.

## 4. Patches and the patch vertex

`patches.rs` generates, for each of the 256 masks:

- **Segments**: on each cell face, the marching-cubes surface restricted to
  that face is a set of line segments joining the face's crossed edges in
  pairs. A face with 2 crossings has one segment. A face with 4 crossings
  (diagonal actives = ambiguous) gets two segments, paired so that the two
  *empty* corners are separated — the active diagonal pair is thereby
  joined. The rule depends only on the face's own corner states, so the
  two cells sharing a face agree.
- **Patches**: connected components of crossed edges under "joined by a
  segment". One patch = one connected piece of MC surface inside the cell.
  Every patch is a disk (tested).
- **Joint split**: in exactly 36 masks one patch would own all four
  crossings of an ambiguous face — the edge contacts: a diagonal pair of
  actives on that face, alone (12 masks) or with one of them extended one
  step along the face normal (24). The table cuts that patch along the
  plane through the contact edge (§6), so those masks end up with 2 or 3
  patches, joined by `Chord`s, and a crossing on a chord belongs to more
  than one patch (`CaseInfo::patches_of`).

The mesher gives **one vertex per patch**, at the **centroid of the
patch's crossing points**, where a crossing shared by `n` patches counts
`1/n` toward each. No normals, no QEF. Consequences worth knowing:

| configuration | cell patch | vertex offset from the voxel center |
|---|---|---|
| lone voxel, corner cell (1 active slot) | 3 crossings (3 face centers) | ±1/6 on every axis → a lone voxel renders as a **1/3-size cube** |
| straight axis line, cell seeing 2 face-adjacent voxels | 4 crossings | 0.25 across → a line renders as a **0.5-wide bar** |
| flat slab | 4 crossings in one plane | 0 → the surface sits exactly on the voxel boundary |

So thin features are "shrink-wrapped": the thinner the feature, the further
inside the voxels its surface sits. That is inherent to centroid placement
with binary data and is not a bug.

## 5. Polygons

Every crossed **minimal face adjacency** (two leaves of differing state
sharing a rectangle) becomes one polygon (`emit_face` records it;
`finish` builds it). Its corners are the dual cells on the boundary of the
shared rectangle: the four rectangle corners always, plus any *hanging*
octree vertices along its sides (those exist only at scale transitions,
and adding them is what keeps transitions crack-free). For each boundary
cell, `face_vertex_at` picks that cell's patch containing this adjacency's
edge and uses its vertex. Winding: counter-clockwise seen from the active
side.

## 6. The joint split

Output = the full dual of the MC surface: patch → vertex, face segment →
mesh edge, crossing → polygon. At an edge contact the patch table would
naturally give one patch wrapping the contact edge like a tube, in each
of the two cells on the contact. Two things go wrong with one vertex for
it: the centroid of its crossings lies *on* the contact line, so the two
voxels' faces pinch to a point there (the bow tie seen from above), and
the ambiguous face's two segments both run between the same two vertices,
so two mesh edges coincide and four polygons share one edge — a pinch,
odd Euler characteristic, exactly what Manifold Dual Contouring's
Proposition 1 condition (2) forbids.

So `patches::split_joint` cuts that patch along the plane through the
contact edge and the two active corners:

- Walk the patch's boundary cycle. Label each crossing by the side of the
  plane it is on; the crossings on the two normal-direction edges at the
  active corners lie *on* it.
- The cycle crosses the plane twice. Where it does so at an on-plane
  crossing, that crossing is a cut end. Where it does so through a
  segment (the extended cases, where the extension's edge is uncrossed),
  both of that segment's crossings are cut ends.
- Each side's crossings plus the on-plane ones form a half patch. With
  one segment-crossing the cut fans from the single end to both of the
  segment's crossings and the triangle between is a third patch.
- The cuts are `Chord`s: surface edges on no cell face, dually mesh edges
  between two of the same cell's vertices.

The mesher (`emit_dual_cell`, `face_vertices_at`) honours two
consequences: a crossing on a chord weighs `1/n` in each of its `n`
patches' centroids, and the polygon dual to it visits all `n` vertices
in the order the table gives around the crossing, entered from the side
of the polygon's previous corner. At a hanging ring point (scale
transition) two cell edges stand for one adjacency and may now sit in
different patches; the ring walks the first edge's patches up to the
face both edges share and the second's from there on.

The joint becomes a four-sided ring of two vertices per contact cell,
each the centroid of its half's crossings. In the bare case that lands
exactly where the bodies' corners are, so a diagonal chain is one
straight bar. No segment vertices exist any more.

## 7. Traversal

Classic four-proc octree dual contouring (`cell_proc` → 8 children, 12
internal faces, 6 internal edges, 1 center vertex; `face_proc`,
`edge_proc`, `vert_proc` recurse until every participant is a leaf; a leaf
reached early stands in for its own children). Chunk seams: the same procs
are seeded at the chunk-lattice level (faces/edges/vertices between chunk
roots), with absent chunks as `EMPTY_NODE`. Jobs are sorted so output is
deterministic; the tests assert chunked output is *bit-identical* to
meshing the same data as one octree (`analysis::weld` compares bits, not
with a tolerance). Any new vertex position must therefore be computed
from the same inputs in the same order regardless of which polygon or
chunk creates it first.

## 8. Rendering

`MeshData::flat_from_polys` triangulates: quads by the non-folding
diagonal, n-gons as a fan around the centroid (dual faces are neither
reliably convex nor planar). Normals per triangle, duplicated vertices.
Meshing never touches normals.

## 9. Worked example: two voxels across an edge (the TestBed)

Voxels `A = [3,4)×[3,4)×[4,5)` and `B = [4,5)×[4,5)×[4,5)` (same as the
TestBed's `(15,15,16)`/`(16,16,16)` shifted by 12). They share only the
line `x = y = 4, z ∈ [4,5]`. Welded output: 16 vertices, 12 polygons,
Euler 2, no non-manifold edges.

**Corner cells** (see one voxel in one slot): 6 per voxel, e.g. the cell
at `q = (3,3,5)` has A in slot `0b011` (A lies at +x, +y, −z of `q`).
Patch = the three faces A shares with the cell's other slots: A's −x, −y
and +z face centers `(3,3.5,4.5)`, `(3.5,3,4.5)`, `(3.5,3.5,5)` → vertex
`(3.333, 3.333, 4.667)`. All twelve corner-cell
vertices are at `voxel center ± 1/6` per axis:

```
A: (3.333|3.667, 3.333|3.667, 4.333|4.667)   minus the two slots the contact cells take
B: (4.333|4.667, 4.333|4.667, 4.333|4.667)   likewise
```

**Contact cells** at `(4,4,4)` and `(4,4,5)`: A in slot 0, B in slot 3 →
mask `0b1001`, the bare edge-contact case. The tube patch's crossings
are A's +x `(4,3.5,·)`, A's +y `(3.5,4,·)`, B's −x `(4,4.5,·)`, B's −y
`(4.5,4,·)` (all at `z = 4.5`), plus A's and B's +z faces at
`(3.5,3.5,5)` and `(4.5,4.5,5)` for the upper cell. The cutting plane is
`x = y`; the two z-face crossings lie on it and are the cut's ends. The
upper cell's two patches are therefore

| half | crossings (weight) | vertex |
|---|---|---|
| +x−y side | A+x (1), B−y (1), A+z (½), B+z (½) | `(4.167, 3.833, 4.667)` |
| −x+y side | A+y (1), B−x (1), A+z (½), B+z (½) | `(3.833, 4.167, 4.667)` |

and the lower cell's are the same at `z = 4.333`. Each vertex is 1/6 off
the contact line in x and y and 1/6 off the voxel-center plane in z —
exactly the offsets of a corner-cell vertex from its voxel center.

**The joint** is the rectangle through those four vertices: 0.471 wide
across the chain, 0.333 tall, the same cross-section as the bodies.

**Polygons** (12): A's −x and −y faces and B's +x and +y faces are plain
quads of corner-cell vertices. The four faces that touch the contact
line (A+x, A+y, B−x, B−y) are quads too, each ending on one side edge of
the joint rectangle. A's and B's ±z faces are pentagons: three body
corners plus both joint vertices on that side, the edge between those
two being the chord's dual. Seen from above the two tops meet along a
full edge, not at a point.

### Widths along a diagonal chain

For a staircase `(i, i, z)` every joint is this configuration repeated.
Distances from the chain axis (the line through the voxel centers):

| part | across `(1,−1,0)` | vertical z |
|---|---|---|
| body (corner-cell vertices, `±1/6` per axis) | 0.236 | 0.167 |
| joint (half-patch vertices, `±1/6` per axis) | 0.236 | 0.167 |

Identical, so the chain is one straight bar of 0.471 × 0.333
cross-section; `tests/meshing.rs` asserts every vertex lies on it. For
how it looked with one vertex per contact cell plus segment vertices
(a lump 50 % wider than the bodies, and a bow tie), see
`docs/plan-smooth-joints.md`.

## 10. Invariants to keep (tests in `crates/voxel-mesh/tests/meshing.rs`)

- Closed, 2-manifold: `boundary == 0`, `non_manifold == 0`, correct Euler
  characteristic on every shape (an odd value means a pinch is back).
- Chunked == single-domain, bit-identical.
- Placement invariance: a shape meshes congruently wherever it sits.
- No triangle faces away from its polygon.
- Patch table: every patch a disk (segments plus chords make one cycle);
  no patch owns both segments of a face; exactly 36 split cases, 12 with
  one chord and two patches, 24 with two chords and three.
- Every 2×2×2 block of voxels (all 254 non-trivial masks) is closed and
  manifold, and every scene in `voxel-bevy` is too.
