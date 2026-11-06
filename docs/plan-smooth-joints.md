# Smoother edge-contact joints — analysis and plan

Status (November 2026): **done, by Option C** — the joint patch is split
in the patch table, giving each contact cell two vertices, which removes
both the lump and the bow tie; see `docs/algorithm.md` §6 and the
"Outcome" section at the end. Option A was prototyped first and is kept
below for the record; it removed the lump but not the bow tie.

Context: the goal is drawing trees whose thin branches run diagonally, so
a branch is a chain of finest-scale voxels touching along edges. The
mesher joins those into one manifold surface (see `docs/algorithm.md`
§6), but the joint has a visible lump: each pair of voxels tapers to the
contact and a flat diamond sticks out sideways there (`images/
joint-before-zoom.png`, `images/staircase-before-zoom.png`).

## Why the lump is there

Read `docs/algorithm.md` §9 for the full coordinates. The short version:

- Every **patch vertex** is the centroid of 3–6 crossing points, which
  pulls it *inside* the marching-cubes surface. Along a diagonal chain the
  body vertices end up 1/6 off the voxel center on each axis: the body is
  a bar 0.236 wide (half-width, across the chain) and 0.167 tall (half).
- The **segment vertex** that opens the pinch is the midpoint of *two*
  crossings. Both lie on the MC surface and the midpoint of a straight MC
  segment is still on it, so this vertex gets **no inward shrink at all**:
  it sits 1/4 off the contact line on each axis, half-width 0.354 across.
- So the throat (0.354) is 50 % wider than the bodies it joins (0.236).
  The side faces flare out from the body to the throat vertex and back:
  that is the lump. Meanwhile the joint's patch vertices sit *on* the
  contact line, so the top and bottom faces pinch to a point there — the
  bow-tie seen from above.

Root cause in one line: the patch rule and the segment rule shrink by
different amounts, so the throat does not fit inside the bodies.

## What "smooth" can mean with this topology

One vertex per patch is the algorithm's defining property, and the joint
cell has one 6-crossing patch, so the top face of a chain necessarily
pinches to a single vertex at each joint. The best achievable without
changing topology is a bar whose **silhouette** is constant from every
direction (constant width from above, constant height from the side) and
whose cross-section alternates between the body rectangle and a diamond
inscribed in it. That is a mild faceted twist, not a lump. Anything
rounder needs more vertices at the joint (Option C below).

## Options

### A. Pull the segment vertex toward the pinch point (recommended)

Rule: **the segment vertex is the centroid of its two crossings and the
pinch point**, where the pinch point is the centroid of the merged face's
four crossings (the point the collapsed edge would have passed through).
Equivalently, with `p` = segment midpoint and `m` = pinch point,
`v = m + (2/3)(p − m)`.

Why 2/3 is not a magic number: it is what "one more point in the
centroid" gives, the same kind of rule as the patch vertex (a centroid of
crossing-derived points), and it lands the throat vertex at exactly the
body's offset (1/6 per axis) — so a diagonal chain becomes a bar of
constant silhouette width. Prototyped (then reverted):

| | before | after |
|---|---|---|
| throat segment vertices (pair at 3,3,4 / 4,4,4) | `(4.25, 3.75, 4.5)`, `(3.75, 4.25, 4.5)` | `(4.167, 3.833, 4.5)`, `(3.833, 4.167, 4.5)` |
| throat half-width across the chain | 0.354 | 0.236 (= body) |
| test suite | all pass | all pass except the one asserting throat width > 0.5 |
| render | `images/joint-before-zoom.png`, `images/staircase-before-zoom.png` | `images/joint-after-candidate-zoom.png`, `images/staircase-after-candidate-zoom.png` |

Properties preserved: still no normals, still only crossing points as
input, manifoldness untouched (positions only), chunked == single-domain
still bit-identical in the prototype.

Variant A′: pinch point = midpoint of the two patch vertices the segment
keeps apart, instead of the merged face's crossing centroid. Identical for
equal scales; differs slightly at a coarse/fine contact. Pick by looking
at the mixed-scale case (step 4 below). A is preferable on principle
(local to the face, crossing-only).

### B. Centroid of the two crossings and the two patch vertices

`v = m + (1/2)(p − m)`: half-width 0.177, i.e. the throat is 75 % of the
body width — a slight waist at every joint. Reads as a gentle necklace of
segments rather than a bar. Cheap to A/B against A (it is one constant
in the same code path); only worth trying if A looks too "twisted".

### C. A wider throat with more vertices (topology change)

Make the throat a rectangle matching the body cross-section by giving the
joint two top and two bottom vertices. That means a 6-crossing patch gets
two vertices, i.e. abandoning one-vertex-per-patch at merged cells, or
inserting a bridge element that is not part of the dual. It would need
new manifoldness reasoning and new tests, and departs from DMC. Not now;
revisit only if A/B are judged not smooth enough.

### D. Post-process smoothing (not recommended)

A Laplacian or Catmull–Clark pass on the throat rings (or the whole mesh)
would round the joints but also soften the crisp big faces of coarse
voxels and the flat-shaded look; it also hides the mesher's own
placement, which is what this repo is studying.

### E. Widen the bodies instead (rejected)

Changing the patch-vertex rule so thin features render wider would move
every vertex in the world and break placement invariance / the
shared-face reasoning in `shared_face`. The body width is a property of
the whole approach, not of the joint.

## Plan (Option A)

1. **Pinch point per merged cell.** In `emit_dual_cell` (`dmc.rs`), when
   a case is merged, compute `pinch: Vec3` = centroid of the merged face's
   four `FaceKey::center()`s and store it on `DualCell` next to
   `merged_edges`. Sum the four in a canonical order (sort the keys first —
   `FaceKey` derives `Ord`) so both cells sharing the face, and every chunk
   configuration, produce bit-identical floats. Do not compute it lazily
   from whichever polygon arrives first.
2. **Segment vertex rule.** In `segment_vertex`, place the vertex at
   `(key.0.center() + key.1.center() + pinch) / 3.0`. Keep the
   `segment_verts` cache keyed by the sorted `FaceKey` pair as now.
3. **Tests** (`crates/voxel-mesh/tests/meshing.rs`):
   - `edge_touching_voxels_mesh_as_one_surface_with_a_manifold_throat`:
     replace the `> 0.5` width check with the actual expected positions
     and an aesthetic invariant — the throat is no wider than the body:
     every throat vertex's per-axis offset from the contact line ≤ 1/6
     (+ epsilon).
   - New: `a_diagonal_staircase_has_constant_width` — for the 6-voxel
     staircase, the maximum distance of any vertex from the chain axis
     equals the body's (0.236 ± 1e-5) and every vertex lies within it.
   - Add the pair and the staircase to the no-folded-triangles checks
     (`assert_no_folded_triangles`), since the pentagons are non-planar.
   - Everything else must stay green, especially chunked == single-domain
     and placement invariance.
4. **Mixed scales.** `edge_touching_voxels_of_different_scales_stay_manifold`
   must still pass. Put that configuration (4-wide coarse voxel + one fine
   voxel on its edge) in the TestBed, screenshot, and compare A against
   A′; pick by eye, record the choice in the docs.
5. **Docs.** Update the "Segment vertices" section of `dmc.rs`, the
   `segment_vertex` doc comment, the matching paragraph in `patches.rs`,
   `README.md`, the CLAUDE.md cheat-sheet sentence ("at the centroid of
   their two crossings"), and `docs/algorithm.md` §6/§9.
6. **Render check.** `cargo run -p voxel-bevy` on `TestBed` (pair and a
   staircase), then `DiagonalTree`; CI screenshots into `images/` beside
   the before/after pairs already there.
7. **Commit** as one mesher change with the tests, mirroring the style of
   `7d159b8`.

Follow-up only if needed: try B (one constant), then consider C.

## Outcome

Option A was prototyped and rendered (`images/*-after-candidate-zoom.png`):
the sideways lump went, but the top faces still met at a single vertex
on the contact line — the bow tie — because the contact cell's tube
patch had one vertex and any centroid of its crossings lies on the
contact line. Only a second vertex per contact cell removes that, so
Option C was implemented after all, in a form that stays inside the
dual-of-MC framework:

- `patches::split_joint` cuts the tube patch along the plane through
  the contact edge, between the crossings where the patch boundary
  crosses that plane. Enumerating the 36 cases showed only two shapes:
  the bare pair (both cut ends are crossings; one chord, two patches)
  and the pair with one voxel extended along the contact edge (one end
  is a segment; the cut fans to both of its crossings and a third,
  triangular patch sits between — symmetric, no tie-break).
- A crossing on a cut belongs to every patch along it, weighted `1/n`
  in each centroid. With that weighting the bare case's joint vertices
  land at exactly the bodies' offsets, so a diagonal chain is a straight
  bar (`images/*-after-split-zoom.png`).
- Segment vertices and everything around them were deleted from the
  mesher; the polygon ring instead visits a cell's patches in the
  table's order around the crossing, and at hanging ring points merges
  the two cell edges that stand for one adjacency.
- Tests: the pair and staircase assert the bar geometry; the extended
  cases (four arrangements) and all 254 non-trivial 2×2×2 blocks assert
  closed manifold output with no folded triangles; every demo scene is
  checked the same way.

## Reproducing the numbers

```rust
// scratch example, e.g. crates/voxel-mesh/examples/dump_pair.rs
let mut tree = Octree::uniform(3, Voxel::INACTIVE);
tree.set(IVec3::new(3, 3, 4), Voxel::ACTIVE);
tree.set(IVec3::new(4, 4, 4), Voxel::ACTIVE);
let mesh = weld(&mesh_octree(&tree, IVec3::ZERO));
// print mesh.positions / mesh.faces
```

Screenshots: `CI_TESTING_CONFIG=crates/voxel-bevy/ci_screenshot.ron cargo
run -p voxel-bevy` writes `screenshot-150.png` to the working directory.
