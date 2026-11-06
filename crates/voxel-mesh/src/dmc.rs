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
//! centroid of its edge crossings, and a crossing sits at the center of
//! the face the two leaves share (see `shared_face`, which explains why
//! that and not the midpoint of their centers). No normals, no QEF;
//! richer voxel data would only change that one interpolation.
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
//! ## Joints
//!
//! The output is the full dual of the marching-cubes surface, on three
//! levels: patch → vertex, **segment → mesh edge**, crossing → polygon.
//! At an edge contact — two voxels diagonal across a cell face — the
//! patch table would naturally give one patch wrapping the contact edge,
//! whose single vertex has to sit on that edge: the voxels' faces pinch
//! to a point there and the face's two segments collapse onto one mesh
//! edge shared by four polygons. So the table cuts such a patch in two
//! (three in the extended cases) along the plane through the contact
//! edge, with [`crate::patches::Chord`]s between the pieces (see its
//! module docs). This mesher only has to honour two consequences:
//!
//! - a crossing on a chord belongs to every patch along it, and counts
//!   `1/n` toward each of the `n` vertices (`emit_dual_cell`);
//! - the polygon dual to such a crossing visits all of those vertices,
//!   in the order the table gives around the crossing, entered from the
//!   side of the polygon's previous corner (`face_vertices_at`).
//!
//! Chords then come out as mesh edges between two vertices of one cell,
//! and a joint is a proper four-sided ring whose corners sit where the
//! bodies' corners do — a diagonal chain of voxels meshes as one
//! straight bar. Nothing else changes: flat regions keep their quads.
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

use crate::cube::{EDGE_ENDPOINTS, edge_axis, edge_between, edge_faces, face_index};
use crate::patches::case_info;
use crate::poly_mesh::PolyMesh;

/// Meshes every chunk of the world, seams included, into one mesh with
/// shared dual vertices. Positions are in world voxel units.
pub fn mesh_world(world: &World) -> PolyMesh {
    run_world(world, Inspect::NONE).0
}

/// Inspection data for one *active* dual cell (a meshing cell whose
/// corner voxels are not all the same state — the cells that produce
/// surface). For debug rendering and tooling.
#[derive(Debug, Clone)]
pub struct DualCellInfo {
    /// The octree vertex the cell sits at.
    pub vertex: IVec3,
    /// Corner-state mask, bit `o` = octant-`o` slot is active.
    pub mask: u8,
    /// The 8 slot leaf centers by octant. A coarser leaf filling several
    /// slots repeats its center, so a cell touching mixed scales is a
    /// degenerate cube — its bounding box has voxel centers on faces or
    /// edges rather than corners.
    pub corners: [Vec3; 8],
}

/// One edge of the octree's **dual graph**: a minimal face-adjacency
/// between two leaves, i.e. the graph whose nodes are voxel centers and
/// whose edges join voxels that share a face. The mesher's whole job can
/// be read off this graph — every `crossed` edge (the two leaves differ in
/// state) becomes one surface polygon, and the dual cells are its
/// vertices.
#[derive(Debug, Clone, Copy)]
pub struct DualEdge {
    /// Center of the leaf on the negative side of the shared face.
    pub a: Vec3,
    /// Center of the leaf on the positive side.
    pub b: Vec3,
    /// Whether the two leaves differ in state — the adjacencies that
    /// produce surface.
    pub crossed: bool,
}

/// What extra data an inspection run should collect. Meshing itself never
/// pays for these — [`mesh_world`] passes [`Inspect::NONE`].
#[derive(Debug, Clone, Copy, Default)]
pub struct Inspect {
    pub dual_cells: bool,
    pub dual_graph: bool,
}

impl Inspect {
    pub const NONE: Inspect = Inspect {
        dual_cells: false,
        dual_graph: false,
    };
    pub const ALL: Inspect = Inspect {
        dual_cells: true,
        dual_graph: true,
    };
}

/// What an inspection run collected; requested parts are filled, the rest
/// stay empty.
#[derive(Debug, Default)]
pub struct Inspection {
    pub dual_cells: Vec<DualCellInfo>,
    pub dual_graph: Vec<DualEdge>,
}

/// Runs the same traversal as [`mesh_world`] but returns its internal
/// structure instead of the mesh. Debug/inspection only — not optimised.
pub fn inspect_world(world: &World, what: Inspect) -> Inspection {
    run_world(world, what).1
}

fn run_world(world: &World, what: Inspect) -> (PolyMesh, Inspection) {
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
        what,
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
        Inspect::NONE,
    )
    .0
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

/// Identity of one minimal face-adjacency: the axis it is normal to and
/// the rectangle the two leaves share. Every construct that names the
/// same adjacency — a dual cell's crossed edge, a surface polygon — gets
/// the same key.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct FaceKey {
    axis: u8,
    /// Min corner of the shared rectangle; `rmin[axis]` is its plane.
    rmin: [i32; 3],
    rsize: i32,
}

impl FaceKey {
    fn min_corner(self) -> IVec3 {
        IVec3::from(self.rmin)
    }

    /// The rectangle's center — where the surface crosses this adjacency.
    fn center(self) -> Vec3 {
        let mut p = self.min_corner().as_vec3();
        let (u, v) = tangents(self.axis as usize);
        for t in [u, v] {
            p[t] += self.rsize as f32 * 0.5;
        }
        p
    }
}

/// The face two face-adjacent leaves of differing state share, on the
/// plane through the octree vertex `q` normal to `axis`. `neg` is the
/// leaf on the negative side of that plane.
///
/// The shared face is where the two leaves overlap — both their faces
/// when the sizes match, the smaller leaf's face when they don't. The
/// surface crosses at its **center** ([`FaceKey::center`]) rather than
/// where the segment between the two leaf *centers* pierces the plane,
/// which is what keeps the surface independent of how *empty* space
/// happens to be partitioned: a solid voxel meets its neighbour at its
/// own face center whether that neighbour is one voxel or a merged
/// 32-wide leaf. So the same solid shape meshes the same way wherever it
/// sits in the octree, an isolated voxel comes out symmetric, and the
/// edge of the world behaves exactly like adjacent inactive voxels
/// rather than like the huge empty leaf that represents it.
///
/// That center is the one place richer voxel data would interpolate
/// instead of taking a midpoint.
fn shared_face<N: TreeNode>(neg: CellRef<N>, pos: CellRef<N>, axis: usize, q: IVec3) -> FaceKey {
    // Only a crossed edge reaches here, so the two slots hold different
    // leaves and must therefore abut the plane rather than span it.
    debug_assert_eq!(
        neg.min[axis] + neg.size,
        q[axis],
        "leaves do not abut at {q}"
    );
    debug_assert_eq!(pos.min[axis], q[axis], "leaves do not abut at {q}");

    let mut rmin = [0i32; 3];
    rmin[axis] = q[axis];
    let (u, v) = tangents(axis);
    let mut rsize = 0;
    for t in [u, v] {
        let lo = neg.min[t].max(pos.min[t]);
        let hi = (neg.min[t] + neg.size).min(pos.min[t] + pos.size);
        debug_assert!(lo < hi, "face-adjacent leaves must overlap on axis {t}");
        debug_assert!(rsize == 0 || rsize == hi - lo, "shared face is not square");
        rmin[t] = lo;
        rsize = hi - lo;
    }
    FaceKey {
        axis: axis as u8,
        rmin,
        rsize,
    }
}

fn unit(axis: usize) -> IVec3 {
    IVec3::AXES[axis]
}

/// A crossed minimal face (the shared face between two leaves of differing
/// state), recorded during traversal and turned into a polygon afterwards
/// — its boundary dual cells may not all exist yet while traversing.
struct FaceRec {
    key: FaceKey,
    neg_active: bool,
}

/// A dual cell's contribution: its corner-state mask and one mesh vertex
/// per patch, indexed like `case_info(mask).patches`.
struct DualCell {
    mask: u8,
    patch_verts: Vec<u32>,
}

#[derive(Default)]
struct Builder {
    mesh: PolyMesh,
    dual_cells: HashMap<IVec3, DualCell>,
    faces: Vec<FaceRec>,
    /// What to record beyond the mesh, and the recordings so far.
    what: Inspect,
    inspection: Inspection,
}

fn mesh_lattice<'a>(
    root_at: &dyn Fn(IVec3) -> CellRef<&'a Node>,
    coords: &[IVec3],
    side: i32,
    what: Inspect,
) -> (PolyMesh, Inspection) {
    let mut b = Builder {
        what,
        ..Builder::default()
    };

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
        if self.what.dual_cells {
            self.inspection.dual_cells.push(DualCellInfo {
                vertex: q,
                mask,
                corners: slots.map(|s| s.center()),
            });
        }
        let info = case_info(mask);
        let mut crossing: [Option<(Vec3, f32)>; 12] = [None; 12];
        for (e, &(a, b)) in EDGE_ENDPOINTS.iter().enumerate() {
            let shared_by = info.patches_of(e).len();
            if shared_by > 0 {
                let key = shared_face(slots[a], slots[b], edge_axis(e), q);
                // A crossing on a chord is on the boundary of every
                // patch along it, so it counts once in total, split
                // evenly between them.
                crossing[e] = Some((key.center(), 1.0 / shared_by as f32));
            }
        }

        let mut patch_verts = Vec::with_capacity(info.patches.len());
        for &patch in &info.patches {
            let mut sum = Vec3::ZERO;
            let mut total = 0.0;
            for (e, c) in crossing.iter().enumerate() {
                if patch & (1 << e) != 0 {
                    let (p, w) = c.expect("a patch edge is crossed");
                    sum += p * w;
                    total += w;
                }
            }
            patch_verts.push(self.mesh.push_vertex(sum / total));
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
        let crossed = neg.active() != pos.active();
        // Every call here is one minimal face-adjacency between two
        // leaves, crossed or not — i.e. exactly one dual-graph edge.
        if self.what.dual_graph {
            self.inspection.dual_graph.push(DualEdge {
                a: neg.center(),
                b: pos.center(),
                crossed,
            });
        }
        if !crossed {
            return;
        }
        let key = shared_face(neg, pos, n, rmin);
        debug_assert_eq!(key.min_corner(), rmin, "face rect min mismatch");
        debug_assert_eq!(key.rsize, rsize, "face rect isn't the smaller leaf's");
        debug_assert_eq!(rsize, neg.size.min(pos.size), "face rect size mismatch");
        self.faces.push(FaceRec {
            key,
            neg_active: neg.active(),
        });
    }

    /// The mesh vertices a crossed face uses at boundary point `q` of its
    /// shared rectangle: those of the dual cell's patches touching this
    /// adjacency, in order around it. The polygon arrives from the
    /// previous ring point along `d_in` and leaves toward the next along
    /// `d_out`, so the order runs from the patch owning the surface
    /// segment on the cell face toward the previous point to the one
    /// owning the segment toward the next. Corner points must have a dual
    /// cell; interior edge points ("hanging" octree vertices) contribute
    /// only where one exists.
    ///
    /// At a hanging point the two leaves span `q` along the side's axis
    /// and so fill two slots each: two cell edges stand for this one
    /// adjacency (and a zero-length segment joins them on the face
    /// between). The ring then walks the first edge's patches up to that
    /// face and the second's from it onward.
    fn face_vertices_at(
        &self,
        f: &FaceRec,
        q: IVec3,
        required: bool,
        d_in: IVec3,
        d_out: IVec3,
        out: &mut Vec<u32>,
    ) {
        let Some(cell) = self.dual_cells.get(&q) else {
            assert!(
                !required,
                "no dual cell at {q} for a crossed face {:?}+{}",
                f.key.rmin, f.key.rsize
            );
            return;
        };
        let info = case_info(cell.mask);
        let axis = f.key.axis as usize;
        let (u, v) = tangents(axis);
        // The previous point lies back along `d_in`, the next ahead along
        // `d_out`; the faces of the cell toward them.
        let face_toward = |d: IVec3, ahead: bool| {
            let t = (0..3).find(|&t| d[t] != 0).expect("ring step is axis-aligned");
            debug_assert!(t == u || t == v, "ring must step along a face tangent");
            face_index(t, usize::from((d[t] > 0) == ahead))
        };
        let f_prev = face_toward(d_in, false);
        let f_next = face_toward(d_out, true);
        // The cell edges standing for this adjacency: along the face
        // normal, at the tangent slots the rectangle covers — one per
        // tangent at a corner, both where `q` is interior to a side.
        let edge_on = |face: usize| {
            let (t, side) = (face / 2, face % 2);
            let other = if t == u { v } else { u };
            let o_neg = (side << t) | (self.slot_bit(f, q, other) << other);
            edge_between(o_neg, o_neg | (1 << axis))
        };
        let (e_in, e_out) = (edge_on(f_prev), edge_on(f_next));
        // `chain(e, from)`: the patches around crossing `e` starting from
        // the face `from`, which must be one of the edge's two faces.
        let chain = |e: usize, from: usize| {
            let [f_lo, f_hi] = edge_faces(e);
            debug_assert!(from == f_lo || from == f_hi, "face {from} does not hold edge {e}");
            let c = info.patches_of(e);
            assert!(!c.is_empty(), "dual cell at {q} has no patch on edge {e}");
            let forward = from == f_lo;
            (0..c.len()).map(move |i| c[if forward { i } else { c.len() - 1 - i }])
        };
        let push = |out: &mut Vec<u32>, p: u8| out.push(cell.patch_verts[p as usize]);
        if e_in == e_out {
            chain(e_in, f_prev).for_each(|p| push(out, p));
        } else {
            let f_mid = edge_faces(e_in)
                .into_iter()
                .find(|fm| edge_faces(e_out).contains(fm))
                .expect("coincident crossings share a face");
            let mut first: Vec<u8> = chain(e_in, f_prev).collect();
            let rest: Vec<u8> = chain(e_out, f_mid).collect();
            debug_assert_eq!(first.last(), rest.first(), "zero-length segment changes patch");
            first.pop();
            first.into_iter().chain(rest).for_each(|p| push(out, p));
        }
    }

    /// Which side of `q` along tangent `t` lies inside `f`'s rectangle —
    /// the slot bit naming a corner of the cell that both adjacent leaves
    /// cover. (Where `q` is interior to a side both do; the caller then
    /// asks per face instead.)
    fn slot_bit(&self, f: &FaceRec, q: IVec3, t: usize) -> usize {
        usize::from(q[t] != f.key.rmin[t] + f.key.rsize)
    }

    fn finish(mut self) -> (PolyMesh, Inspection) {
        let faces = std::mem::take(&mut self.faces);
        let mut ids = Vec::with_capacity(8);
        for f in &faces {
            let (u, v) = tangents(f.key.axis as usize);
            let rsize = f.key.rsize;
            // Boundary cycle: the rectangle's corners in counter-clockwise
            // order seen from +axis, with any hanging dual cells between
            // them. Each point is entered along the step that reaches it
            // and left along its own side's step; a split cell orders its
            // vertices by those.
            let corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
                .map(|(bu, bv)| f.key.min_corner() + bu * rsize * unit(u) + bv * rsize * unit(v));
            let steps: [IVec3; 4] = std::array::from_fn(|i| (corners[(i + 1) % 4] - corners[i]) / rsize);
            ids.clear();
            for i in 0..4 {
                for k in 0..rsize {
                    let d_in = if k == 0 { steps[(i + 3) % 4] } else { steps[i] };
                    let q = corners[i] + steps[i] * k;
                    self.face_vertices_at(f, q, k == 0, d_in, steps[i], &mut ids);
                }
            }
            // neg side active → surface faces +axis; the cycle is CCW from
            // +axis, so keep it; otherwise reverse.
            if !f.neg_active {
                ids.reverse();
            }
            self.mesh.push_face(ids.clone());
        }
        (self.mesh, self.inspection)
    }
}
