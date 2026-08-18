//! The Dual Marching Cubes mesher over multi-scale octrees.
//!
//! ## Construction
//!
//! Voxels are volumes; their centers are the sample points. The meshing
//! cells ("dual cells") are the cells of the octree's dual grid: one dual
//! cell per octree **vertex** (a point where up to 8 leaves meet), whose 8
//! combinatorial corners are the surrounding leaves — a coarser leaf can
//! occupy several corner slots, making the cell degenerate but still
//! consistently handled. On a uniform grid this is exactly Nielson's DMC
//! grid offset by half a voxel.
//!
//! Per dual cell, the patch table ([`crate::patches`]) splits the crossed
//! edges into surface patches; each patch gets one dual vertex at the
//! centroid of its edge crossings. A crossing sits where the segment
//! between the two leaf centers pierces the plane the two leaves share —
//! for equal sizes, the midpoint. No normals, no QEF; scalar voxel data
//! would only change that one interpolation.
//!
//! The surface faces are dual to the crossed **minimal faces** of the
//! octree: for face-adjacent leaves of differing state, every dual cell on
//! the boundary of their shared face contributes the vertex of the patch
//! containing that adjacency. With equal-scale neighbours the shared
//! face's boundary carries exactly its 4 corner cells — a quad. At scale
//! transitions, hanging octree vertices on the boundary add their cells'
//! vertices to the polygon, which is precisely what keeps the mesh
//! watertight there (a plain quad would leave T-junction cracks against
//! the smaller neighbouring faces).
//!
//! ## Traversal
//!
//! The four standard mutually recursive walks from octree dual contouring,
//! adapted to cell-centered data: `cell_proc` (1 node) recurses into
//! children and spawns `face_proc` (2 nodes) / `edge_proc` (4) /
//! `vert_proc` (8) for its internal faces/edges/center. A leaf reached
//! before its partners stands in for its own children ("pseudo-child"), so
//! mixed scales need no special cases. `face_proc` emits crossed minimal
//! faces; `vert_proc` emits dual cells. The same procs run across trees:
//! the world driver seeds them between chunk roots (the "super-octree"
//! traversal), with absent chunks represented by [`EMPTY_NODE`] — which is
//! also what closes surfaces at the edge of the world.
//!
//! Interior octree vertices/faces each lie in the interior of exactly one
//! construct (cell, face or edge) at exactly one scale, so every dual cell
//! and every minimal face is emitted exactly once — the emission functions
//! `debug_assert` the geometric invariants, and the equivalence tests
//! (chunked vs single-domain) pin down the rest.

use std::collections::{HashMap, HashSet};

use glam::{IVec3, Vec3};

use voxel_core::chunk::CHUNK_SIZE;
use voxel_core::octree::{Node, Octree, octant_offset};
use voxel_core::tree_view::{EMPTY_NODE, TreeNode};
use voxel_core::world::World;

use crate::cube::{EDGE_ENDPOINTS, edge_axis, edge_between};
use crate::patches::{NO_PATCH, case_info};
use crate::poly_mesh::PolyMesh;

/// Meshes every chunk of the world, seams included, into one mesh with
/// shared dual vertices. Positions are in world voxel units.
pub fn mesh_world(world: &World) -> PolyMesh {
    let side = CHUNK_SIZE as i32;
    let coords = world.chunk_coords();
    mesh_lattice(
        &|c| {
            let node: &Node = world
                .chunk(c)
                .map_or(&EMPTY_NODE, |chunk| chunk.tree().root());
            CellRef {
                node,
                min: c * side,
                size: side,
            }
        },
        &coords,
        side,
    )
}

/// Meshes one octree (world position `origin`, any depth) surrounded by
/// emptiness. Used by tests as the single-domain reference and handy for
/// meshing isolated objects.
pub fn mesh_octree(tree: &Octree, origin: IVec3) -> PolyMesh {
    let side = tree.side() as i32;
    mesh_lattice(
        &|c| {
            let node = if c == IVec3::ZERO {
                tree.root()
            } else {
                &EMPTY_NODE
            };
            CellRef {
                node,
                min: origin + c * side,
                size: side,
            }
        },
        &[IVec3::ZERO],
        side,
    )
}

/// One node plus its actual box. For a pseudo-child (a leaf standing in
/// for its own children during recursion) the box stays the leaf's own —
/// that is what makes a coarse voxel's dual vertex sit at the coarse
/// center.
#[derive(Clone, Copy)]
struct CellRef<N> {
    node: N,
    min: IVec3,
    size: i32,
}

impl<N: TreeNode> CellRef<N> {
    fn is_leaf(self) -> bool {
        self.node.is_leaf()
    }

    fn active(self) -> bool {
        self.node.voxel().is_active()
    }

    fn center(self) -> Vec3 {
        self.min.as_vec3() + Vec3::splat(self.size as f32 * 0.5)
    }

    fn children(self) -> Option<[CellRef<N>; 8]> {
        let half = self.size / 2;
        self.node.children().map(|ch| {
            std::array::from_fn(|o| CellRef {
                node: ch[o],
                min: self.min + octant_offset(o) * half,
                size: half,
            })
        })
    }

    /// The child in `octant`, or the cell itself if it is a leaf.
    fn child_or_self(self, octant: usize) -> CellRef<N> {
        match self.children() {
            Some(ch) => ch[octant],
            None => self,
        }
    }
}

fn tangents(axis: usize) -> (usize, usize) {
    ((axis + 1) % 3, (axis + 2) % 3)
}

fn unit(axis: usize) -> IVec3 {
    IVec3::AXES[axis]
}

/// A crossed minimal face (the shared face between two leaves of differing
/// state), recorded during traversal and turned into a polygon afterwards
/// — its boundary dual cells may not all exist yet while traversing.
struct FaceRec {
    axis: usize,
    /// Min corner of the shared rectangle; `rmin[axis]` is the plane.
    rmin: IVec3,
    rsize: i32,
    neg_active: bool,
}

/// A dual cell's contribution: its corner-state mask and one mesh vertex
/// per patch.
struct DualCell {
    mask: u8,
    patch_verts: Vec<u32>,
}

#[derive(Default)]
struct Builder {
    mesh: PolyMesh,
    dual_cells: HashMap<IVec3, DualCell>,
    faces: Vec<FaceRec>,
}

fn mesh_lattice<'a>(
    root_at: &dyn Fn(IVec3) -> CellRef<&'a Node>,
    coords: &[IVec3],
    side: i32,
) -> PolyMesh {
    let mut b = Builder::default();

    // Job keys: every lattice cell/face/edge/vertex incident to an
    // existing chunk, deduplicated. Face (c, n) sits between c and c+eₙ;
    // edge (c, m) runs along m where the 4 chunks c + {0,1}·eₐ + {0,1}·e_b
    // meet; vertex c is where the 8 chunks c + {0,1}³ meet.
    let mut faces: HashSet<(IVec3, usize)> = HashSet::new();
    let mut edges: HashSet<(IVec3, usize)> = HashSet::new();
    let mut verts: HashSet<IVec3> = HashSet::new();
    for &c in coords {
        for n in 0..3 {
            faces.insert((c, n));
            faces.insert((c - unit(n), n));
        }
        for m in 0..3 {
            let (a, bb) = tangents(m);
            for i in 0..2 {
                for j in 0..2 {
                    edges.insert((c - i * unit(a) - j * unit(bb), m));
                }
            }
        }
        for o in 0..8 {
            verts.insert(c - octant_offset(o));
        }
    }

    let key = |c: IVec3| (c.z, c.y, c.x);
    let mut cell_jobs: Vec<IVec3> = coords.to_vec();
    cell_jobs.sort_by_key(|&c| key(c));
    let mut face_jobs: Vec<_> = faces.into_iter().collect();
    face_jobs.sort_by_key(|&(c, n)| (key(c), n));
    let mut edge_jobs: Vec<_> = edges.into_iter().collect();
    edge_jobs.sort_by_key(|&(c, m)| (key(c), m));
    let mut vert_jobs: Vec<_> = verts.into_iter().collect();
    vert_jobs.sort_by_key(|&c| key(c));

    for c in cell_jobs {
        b.cell_proc(root_at(c));
    }
    for (c, n) in face_jobs {
        let neg = root_at(c);
        let pos = root_at(c + unit(n));
        b.face_proc(neg, pos, n, pos.min, side);
    }
    for (c, m) in edge_jobs {
        let (a, bb) = tangents(m);
        let parts = std::array::from_fn(|slot| {
            root_at(c + (slot as i32 & 1) * unit(a) + (slot as i32 >> 1) * unit(bb))
        });
        let mut origin = (c + unit(a) + unit(bb)) * side;
        origin[m] = c[m] * side;
        b.edge_proc(parts, m, origin, side);
    }
    for c in vert_jobs {
        let parts = std::array::from_fn(|o| root_at(c + octant_offset(o)));
        b.vert_proc((c + IVec3::ONE) * side, parts);
    }

    b.finish()
}

impl Builder {
    fn cell_proc<N: TreeNode>(&mut self, c: CellRef<N>) {
        let Some(kids) = c.children() else { return };
        let half = c.size / 2;

        for k in kids {
            self.cell_proc(k);
        }
        // 12 internal faces: child pairs along each axis.
        for n in 0..3 {
            for o in 0..8 {
                if o & (1 << n) == 0 {
                    let pos = kids[o | (1 << n)];
                    self.face_proc(kids[o], pos, n, pos.min, half);
                }
            }
        }
        // 6 internal edges: 2 segments per axis where 4 children meet.
        for m in 0..3 {
            let (a, bb) = tangents(m);
            for s in 0..2usize {
                let parts = std::array::from_fn(|slot| {
                    let (i, j) = (slot & 1, slot >> 1);
                    kids[(i << a) | (j << bb) | (s << m)]
                });
                let mut origin = c.min + IVec3::splat(half);
                origin[m] = c.min[m] + s as i32 * half;
                self.edge_proc(parts, m, origin, half);
            }
        }
        // 1 internal vertex: the center, where all 8 children meet.
        self.vert_proc(c.min + IVec3::splat(half), kids);
    }

    fn face_proc<N: TreeNode>(
        &mut self,
        neg: CellRef<N>,
        pos: CellRef<N>,
        n: usize,
        rmin: IVec3,
        rsize: i32,
    ) {
        if neg.is_leaf() && pos.is_leaf() {
            self.emit_face(neg, pos, n, rmin, rsize);
            return;
        }
        let (u, v) = tangents(n);
        let half = rsize / 2;
        let sub = |side: CellRef<N>, toward: usize, bu: usize, bv: usize| {
            side.child_or_self((toward << n) | (bu << u) | (bv << v))
        };

        // 4 sub-faces
        for bu in 0..2usize {
            for bv in 0..2usize {
                let srmin = rmin + bu as i32 * half * unit(u) + bv as i32 * half * unit(v);
                self.face_proc(sub(neg, 1, bu, bv), sub(pos, 0, bu, bv), n, srmin, half);
            }
        }
        // 4 edge segments: the internal cross on the face. Segments along
        // `u` at the v-midline, and along `v` at the u-midline.
        for (m, other) in [(u, v), (v, u)] {
            let (t1, _) = tangents(m);
            for s in 0..2usize {
                let mut origin = rmin + half * unit(other);
                origin[m] = rmin[m] + s as i32 * half;
                let parts = std::array::from_fn(|slot| {
                    let (b1, b2) = (slot & 1, slot >> 1);
                    // Slot bits are per tangent axis of the edge; one of
                    // them is this face's normal n, the other is `other`.
                    let bn = if t1 == n { b1 } else { b2 };
                    let bo = if t1 == n { b2 } else { b1 };
                    let side = if bn == 0 { neg } else { pos };
                    side.child_or_self(((1 - bn) << n) | (s << m) | (bo << other))
                });
                self.edge_proc(parts, m, origin, half);
            }
        }
        // 1 vertex: the face center.
        let q = rmin + half * (unit(u) + unit(v));
        let parts = std::array::from_fn(|o| {
            let bn = (o >> n) & 1;
            let side = if bn == 0 { neg } else { pos };
            side.child_or_self(((1 - bn) << n) | (o & (1 << u)) | (o & (1 << v)))
        });
        self.vert_proc(q, parts);
    }

    /// `parts` are the 4 cells around a segment along `m`, indexed by
    /// `(side along (m+1)%3) | (side along (m+2)%3) << 1` — bit set means
    /// the cell is on the positive side of the segment's line.
    fn edge_proc<N: TreeNode>(
        &mut self,
        parts: [CellRef<N>; 4],
        m: usize,
        origin: IVec3,
        len: i32,
    ) {
        if parts.iter().all(|p| p.is_leaf()) {
            return;
        }
        let (a, bb) = tangents(m);
        let half = len / 2;
        for s in 0..2usize {
            let sub = std::array::from_fn(|slot| {
                let (i, j) = (slot & 1, slot >> 1);
                parts[slot].child_or_self(((1 - i) << a) | ((1 - j) << bb) | (s << m))
            });
            self.edge_proc(sub, m, origin + s as i32 * half * unit(m), half);
        }
        let q = origin + half * unit(m);
        let vparts = std::array::from_fn(|o| {
            let (oa, ob) = ((o >> a) & 1, (o >> bb) & 1);
            let om = (o >> m) & 1;
            parts[oa | (ob << 1)].child_or_self(((1 - oa) << a) | ((1 - ob) << bb) | (om << m))
        });
        self.vert_proc(q, vparts);
    }

    /// `parts[o]` is the cell touching vertex `q` from octant direction
    /// `o` (bit set = positive side of `q` along that axis).
    fn vert_proc<N: TreeNode>(&mut self, q: IVec3, parts: [CellRef<N>; 8]) {
        if parts.iter().all(|p| p.is_leaf()) {
            self.emit_dual_cell(q, parts);
            return;
        }
        let sub = std::array::from_fn(|o| parts[o].child_or_self(o ^ 7));
        self.vert_proc(q, sub);
    }

    fn emit_dual_cell<N: TreeNode>(&mut self, q: IVec3, slots: [CellRef<N>; 8]) {
        #[cfg(debug_assertions)]
        for (o, s) in slots.iter().enumerate() {
            for axis in 0..3 {
                let (lo, hi) = (s.min[axis], s.min[axis] + s.size);
                // The slot's box must cover the region just off `q` in the
                // slot's octant direction. Coarse leaves may span across
                // `q` on other axes (they then fill several slots).
                if (o >> axis) & 1 == 1 {
                    debug_assert!(lo <= q[axis] && q[axis] < hi, "slot {o} misplaced at {q}");
                } else {
                    debug_assert!(lo < q[axis] && q[axis] <= hi, "slot {o} misplaced at {q}");
                }
            }
        }

        let mut mask = 0u8;
        for (o, s) in slots.iter().enumerate() {
            if s.active() {
                mask |= 1 << o;
            }
        }
        if mask == 0 || mask == 0xFF {
            return;
        }
        let info = case_info(mask);
        let mut patch_verts = Vec::with_capacity(info.patches.len());
        for &patch in &info.patches {
            let mut sum = Vec3::ZERO;
            let mut count = 0;
            for (e, &(a, b)) in EDGE_ENDPOINTS.iter().enumerate() {
                if patch & (1 << e) != 0 {
                    let m = edge_axis(e);
                    let (ca, cb) = (slots[a].center(), slots[b].center());
                    // The two slot leaves of a crossed edge abut the plane
                    // through q along the edge's axis; the crossing is
                    // where the segment between their centers pierces that
                    // shared plane. Equal sizes ⇒ the midpoint. Scalar
                    // voxel data would replace this interpolation.
                    let t = (q[m] as f32 - ca[m]) / (cb[m] - ca[m]);
                    sum += ca.lerp(cb, t);
                    count += 1;
                }
            }
            patch_verts.push(self.mesh.push_vertex(sum / count as f32));
        }
        let prev = self.dual_cells.insert(q, DualCell { mask, patch_verts });
        debug_assert!(prev.is_none(), "dual cell at {q} emitted twice");
    }

    fn emit_face<N: TreeNode>(
        &mut self,
        neg: CellRef<N>,
        pos: CellRef<N>,
        n: usize,
        rmin: IVec3,
        rsize: i32,
    ) {
        if neg.active() == pos.active() {
            return;
        }
        debug_assert_eq!(neg.min[n] + neg.size, rmin[n], "face plane mismatch");
        debug_assert_eq!(pos.min[n], rmin[n], "face plane mismatch");
        debug_assert_eq!(
            rsize,
            neg.size.min(pos.size),
            "face rect isn't the smaller leaf's"
        );
        self.faces.push(FaceRec {
            axis: n,
            rmin,
            rsize,
            neg_active: neg.active(),
        });
    }

    /// The mesh vertex a crossed face uses at boundary point `q` of its
    /// shared rectangle: the vertex of the dual cell's patch containing
    /// this adjacency. Corner points must have a dual cell; interior edge
    /// points ("hanging" octree vertices) contribute only where one
    /// exists.
    fn face_vertex_at(&self, f: &FaceRec, q: IVec3, required: bool) -> Option<u32> {
        let Some(cell) = self.dual_cells.get(&q) else {
            assert!(
                !required,
                "no dual cell at {q} for a crossed face {:?}+{}",
                f.rmin, f.rsize
            );
            return None;
        };
        // Local edge: along the face normal, at tangent slots pointing
        // into the rectangle — both adjacent leaves cover that quadrant,
        // so those two slots are exactly this adjacency.
        let (u, v) = tangents(f.axis);
        let bu = usize::from(q[u] != f.rmin[u] + f.rsize);
        let bv = usize::from(q[v] != f.rmin[v] + f.rsize);
        let o_neg = (bu << u) | (bv << v);
        let le = edge_between(o_neg, o_neg | (1 << f.axis));
        let p = case_info(cell.mask).edge_patch[le];
        assert_ne!(p, NO_PATCH, "dual cell at {q} has no patch on edge {le}");
        Some(cell.patch_verts[p as usize])
    }

    fn finish(mut self) -> PolyMesh {
        let faces = std::mem::take(&mut self.faces);
        for f in &faces {
            let (u, v) = tangents(f.axis);
            // Boundary cycle: the rectangle's corners in counter-clockwise
            // order seen from +axis, with any hanging dual cells between
            // them.
            let corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
                .map(|(bu, bv)| f.rmin + bu * f.rsize * unit(u) + bv * f.rsize * unit(v));
            let mut ids = Vec::with_capacity(4);
            for i in 0..4 {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                let step = (b - a) / f.rsize;
                for k in 0..f.rsize {
                    if let Some(id) = self.face_vertex_at(f, a + step * k, k == 0) {
                        ids.push(id);
                    }
                }
            }
            // neg side active → surface faces +axis; the cycle is CCW from
            // +axis, so keep it; otherwise reverse.
            if !f.neg_active {
                ids.reverse();
            }
            self.mesh.push_face(ids);
        }
        self.mesh
    }
}
