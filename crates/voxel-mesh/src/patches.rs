//! The Dual Marching Cubes patch table: for each of the 256 corner-state
//! cases, how the marching-cubes surface inside the cell decomposes.
//!
//! Nielson's DMC places one dual vertex per *patch* (connected piece of
//! the marching-cubes surface inside a cell), not one per cell — that is
//! the difference from Surface Nets and what keeps thin/diagonal features
//! from collapsing.
//!
//! The table is generated, not transcribed. Crossed edges follow directly
//! from corner states. On each face, the surface is a set of **segments**
//! joining that face's crossed edges pairwise ([`Segment`]); two crossed
//! edges are in the same patch iff a chain of segments joins them. A face
//! with 4 crossed edges (diagonally opposite active corners) is ambiguous;
//! the fixed rule here pairs the segments so the **empty corners are
//! separated**, which joins the active ones. Material is therefore
//! edge-connected — two voxels meeting along an edge mesh as one surface —
//! while empty space is cut apart there. Corner-only contacts are a
//! different matter: no face sees both voxels, so a body diagonal is still
//! two separate pieces.
//!
//! Whatever the rule, it must depend only on the face's own corner states,
//! since that is what makes the two cells sharing a face agree.
//!
//! ## Splitting the joint patch
//!
//! In 36 cases one patch takes all four crossings of an ambiguous face.
//! They are exactly the edge contacts: two diagonal actives on that face,
//! alone (12 cases) or with one of them extended one step along the face
//! normal (24). That patch wraps around the contact edge like a tube, and
//! a single vertex for it can only sit *on* the contact line — so the two
//! voxels' faces pinch to a point there, and the ambiguous face's two
//! segments collapse onto one mesh edge shared by four polygons (the
//! non-manifold pinch that condition (2) of Proposition 1 in Manifold
//! Dual Contouring, Schaefer/Ju/Warren, rules out).
//!
//! So the table cuts that patch in two along the plane through the
//! contact edge and the two active corners. Each half keeps the segment
//! on its side of the plane, and the cut — a [`Chord`], an edge of the
//! surface lying on no cell face — runs between the crossings where the
//! patch's boundary crosses the plane: the crossings on the two
//! normal-direction edges at the active corners. Where the boundary
//! crosses the plane through the interior of a segment instead (the
//! extended cases, where one of those edges is not crossed), the cut fans
//! out to both of that segment's crossings and the small piece between
//! the two chords becomes a third patch, which keeps the split symmetric.
//!
//! A crossing on a cut belongs to every patch along it; the mesher
//! weights such a crossing by `1/n` when placing the `n` vertices, and a
//! polygon dual to it visits all of them in order ([`CaseInfo::
//! edge_patches`]). Dually a chord is a mesh edge between two of this
//! cell's vertices, so the throat at a joint becomes a proper ring and the
//! joined voxels read as one bar rather than two blobs meeting at a point.

use std::sync::OnceLock;

use crate::cube::{EDGE_ENDPOINTS, edge_between, edge_faces, face_corners, face_index};

pub const NO_PATCH: u8 = u8::MAX;

/// The most patches one crossing can belong to: a crossing where two
/// chords meet.
pub const MAX_PATCHES_PER_EDGE: usize = 3;

/// One piece of the marching-cubes surface lying on a single cell face:
/// the segment joining two of that face's crossed edges. A face with two
/// crossings carries one segment; an ambiguous face (four crossings)
/// carries two.
///
/// Segments are what patches are glued along, and dually they are the
/// *edges* of the output surface — which is why the mesher needs them and
/// not just the patch grouping they produce.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    /// Which cell face the segment lies on, as [`face_index`].
    pub face: usize,
    /// The two crossed cube edges it joins.
    pub edges: [usize; 2],
    /// The patch whose boundary this segment is part of.
    pub patch: u8,
}

/// A cut through a patch's interior between two of its crossings — an
/// edge of the surface that lies on no cell face. Dually a mesh edge
/// between the vertices of the two patches it separates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Chord {
    pub edges: [usize; 2],
    pub patches: [u8; 2],
}

/// One case's surface decomposition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaseInfo {
    /// Each patch as a bitmask over the 12 edges. Every crossed edge is in
    /// at least one patch — in more than one only at a chord's end —
    /// and uncrossed edges are in none.
    pub patches: Vec<u16>,
    /// Per edge, the patches containing it, padded with [`NO_PATCH`]. The
    /// order is around the crossing: from the patch owning the edge's
    /// segment on the lower-indexed of its two faces ([`edge_faces`]) to
    /// the one owning the segment on the higher-indexed face, with a
    /// chord between consecutive entries. Prefer [`CaseInfo::patches_of`].
    pub edge_patches: [[u8; MAX_PATCHES_PER_EDGE]; 12],
    /// Every surface segment on every face of the cell.
    pub segments: Vec<Segment>,
    /// The cuts splitting a joint patch, if this is one of the 36 cases.
    pub chords: Vec<Chord>,
}

impl CaseInfo {
    /// The patches containing `edge`, in order around the crossing (see
    /// [`CaseInfo::edge_patches`]). Empty for an uncrossed edge.
    pub fn patches_of(&self, edge: usize) -> &[u8] {
        let list = &self.edge_patches[edge];
        let n = list.iter().position(|&p| p == NO_PATCH).unwrap_or(list.len());
        &list[..n]
    }

    /// The edge that `edge` is joined to by a segment on `face`, if any.
    pub fn segment_partner(&self, face: usize, edge: usize) -> Option<usize> {
        segment_partner(&self.segments, face, edge)
    }

    /// The patch owning the segment on `face` through `edge`, if any.
    pub fn segment_owner(&self, face: usize, edge: usize) -> Option<u8> {
        self.segments
            .iter()
            .find(|s| s.face == face && s.edges.contains(&edge))
            .map(|s| s.patch)
    }
}

fn segment_partner(segments: &[Segment], face: usize, edge: usize) -> Option<usize> {
    segments.iter().find_map(|s| match s.edges {
        _ if s.face != face => None,
        [a, b] if a == edge => Some(b),
        [a, b] if b == edge => Some(a),
        _ => None,
    })
}

/// Bitmask of the edges whose two corners have differing states.
pub fn crossed_edges(mask: u8) -> u16 {
    let mut crossed = 0u16;
    for (e, &(a, b)) in EDGE_ENDPOINTS.iter().enumerate() {
        if ((mask >> a) & 1) != ((mask >> b) & 1) {
            crossed |= 1 << e;
        }
    }
    crossed
}

/// The surface decomposition for a corner-state mask (bit `i` set = corner
/// `i` active).
pub fn case_info(mask: u8) -> &'static CaseInfo {
    static TABLE: OnceLock<Vec<CaseInfo>> = OnceLock::new();
    &TABLE.get_or_init(|| (0u16..256).map(|m| build_case(m as u8)).collect())[mask as usize]
}

fn build_case(mask: u8) -> CaseInfo {
    let crossed = crossed_edges(mask);

    // The surface on each face, as segments between that face's crossings.
    let mut segments: Vec<Segment> = Vec::new();
    for axis in 0..3 {
        for side in 0..2 {
            let face = face_index(axis, side);
            let corners = face_corners(axis, side);
            let cycle_edges: [usize; 4] =
                std::array::from_fn(|i| edge_between(corners[i], corners[(i + 1) % 4]));
            let face_crossed: Vec<usize> = cycle_edges
                .iter()
                .copied()
                .filter(|&e| crossed & (1 << e) != 0)
                .collect();
            match face_crossed.len() {
                0 => {}
                2 => segments.push(Segment {
                    face,
                    edges: [face_crossed[0], face_crossed[1]],
                    patch: NO_PATCH,
                }),
                4 => {
                    // Alternating states around the cycle: join the two
                    // edges flanking each *empty* corner, cutting those
                    // apart and so connecting the active ones.
                    for (i, &c) in corners.iter().enumerate() {
                        if (mask >> c) & 1 == 0 {
                            segments.push(Segment {
                                face,
                                edges: [cycle_edges[(i + 3) % 4], cycle_edges[i]],
                                patch: NO_PATCH,
                            });
                        }
                    }
                }
                n => unreachable!("a face cycle has 0, 2 or 4 crossings, got {n}"),
            }
        }
    }

    // Patches are the connected components of the crossed edges under
    // "joined by a segment".
    let mut uf = UnionFind::new();
    for s in &segments {
        uf.union(s.edges[0], s.edges[1]);
    }

    let mut patches: Vec<u16> = Vec::new();
    let mut root_to_patch = [NO_PATCH; 12];
    for e in 0..12 {
        if crossed & (1 << e) == 0 {
            continue;
        }
        let root = uf.find(e);
        if root_to_patch[root] == NO_PATCH {
            root_to_patch[root] = patches.len() as u8;
            patches.push(0);
        }
        patches[root_to_patch[root] as usize] |= 1 << e;
    }
    for s in &mut segments {
        s.patch = root_to_patch[uf.find(s.edges[0])];
    }

    // A joint: both segments of an ambiguous face in one patch. Split it.
    let mut chords = Vec::new();
    for face in 0..6 {
        let on_face: Vec<Segment> = segments.iter().copied().filter(|s| s.face == face).collect();
        if on_face.len() == 2 && on_face[0].patch == on_face[1].patch {
            split_joint(mask, face, on_face[0].patch, &mut patches, &mut segments, &mut chords);
        }
    }

    // Order each crossing's patches around it, chord by chord.
    let mut edge_patches = [[NO_PATCH; MAX_PATCHES_PER_EDGE]; 12];
    for (e, slots) in edge_patches.iter_mut().enumerate() {
        if crossed & (1 << e) == 0 {
            continue;
        }
        let containing: Vec<u8> = (0..patches.len() as u8)
            .filter(|&p| patches[p as usize] & (1 << e) != 0)
            .collect();
        let [f_lo, f_hi] = edge_faces(e);
        let owner = |f| {
            segments
                .iter()
                .find(|s| s.face == f && s.edges.contains(&e))
                .map(|s| s.patch)
                .expect("a crossed edge has a segment on each of its faces")
        };
        let mut chain = vec![owner(f_lo)];
        while chain.len() < containing.len() {
            let last = *chain.last().unwrap();
            let next = chords
                .iter()
                .filter(|c| c.edges.contains(&e) && c.patches.contains(&last))
                .map(|c| c.patches[0] ^ c.patches[1] ^ last)
                .find(|p| !chain.contains(p))
                .expect("patches at a crossing are chained by chords");
            chain.push(next);
        }
        assert_eq!(*chain.last().unwrap(), owner(f_hi), "case {mask} edge {e}: chain ends");
        assert!(
            chain.iter().all(|p| containing.contains(p)),
            "case {mask} edge {e}: chain leaves the edge's patches"
        );
        for (slot, p) in chain.into_iter().enumerate() {
            slots[slot] = p;
        }
    }

    CaseInfo {
        patches,
        edge_patches,
        segments,
        chords,
    }
}

/// Which side of the cutting plane a crossing lies on. The plane passes
/// through the two active corners of the merged face and runs along the
/// face normal, so it contains those corners' normal-direction edges.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    C,
    D,
    Plane,
}

/// Cuts patch `p`, which holds both segments of the ambiguous `face`, in
/// two along the plane through the face's active diagonal (see the
/// module docs). `C` is the side of the empty corner following the first
/// active one in cycle order, `D` the other.
fn split_joint(
    mask: u8,
    face: usize,
    p: u8,
    patches: &mut Vec<u16>,
    segments: &mut [Segment],
    chords: &mut Vec<Chord>,
) {
    let (axis, side) = (face / 2, face % 2);
    let corners = face_corners(axis, side);
    let i0 = usize::from((mask >> corners[0]) & 1 == 0);
    let (a, c, d) = (corners[i0], corners[i0 + 1], corners[(i0 + 3) % 4]);
    let n = 1 << axis;
    let side_of = |e: usize| {
        let (lo, hi) = EDGE_ENDPOINTS[e];
        let touches = |k: usize| lo == k || hi == k || lo == k ^ n || hi == k ^ n;
        if touches(c) {
            Side::C
        } else if touches(d) {
            Side::D
        } else {
            Side::Plane
        }
    };

    // Walk the patch's boundary cycle, starting across `face` from the
    // crossing on the a–c edge so it opens with the C-side segment.
    let start = edge_between(a, c);
    let mut cycle = vec![start];
    let (mut at, mut via) = (start, face);
    loop {
        let next = segment_partner(segments, via, at).expect("a crossing has a segment per face");
        if next == start {
            break;
        }
        cycle.push(next);
        let [f1, f2] = edge_faces(next);
        via = if f1 == via { f2 } else { f1 };
        at = next;
    }
    let sides: Vec<Side> = cycle.iter().map(|&e| side_of(e)).collect();

    // Where the cycle crosses the plane: at a crossing on it, or through
    // a segment whose ends lie on opposite sides.
    let mut singles: Vec<usize> = Vec::new();
    let mut pairs: Vec<[usize; 2]> = Vec::new(); // [C end, D end]
    for i in 0..cycle.len() {
        let j = (i + 1) % cycle.len();
        match (sides[i], sides[j]) {
            (Side::Plane, _) => singles.push(cycle[i]),
            (Side::C, Side::D) => pairs.push([cycle[i], cycle[j]]),
            (Side::D, Side::C) => pairs.push([cycle[j], cycle[i]]),
            _ => {}
        }
    }
    assert_eq!(
        singles.len() + pairs.len(),
        2,
        "case {mask}: joint boundary crosses the cutting plane {} times",
        singles.len() + pairs.len()
    );

    let bits = |pred: &dyn Fn(usize) -> bool| -> u16 {
        cycle
            .iter()
            .enumerate()
            .filter(|&(i, _)| pred(i))
            .fold(0, |m, (_, &e)| m | 1 << e)
    };
    let single_bits = bits(&|i| sides[i] == Side::Plane);
    let half_c = p;
    let half_d = patches.len() as u8;
    patches[half_c as usize] = bits(&|i| sides[i] == Side::C) | single_bits;
    patches.push(bits(&|i| sides[i] == Side::D) | single_bits);
    let middle = if pairs.is_empty() {
        None
    } else {
        let ends = pairs.iter().flatten().fold(single_bits, |m, &e| m | 1 << e);
        patches.push(ends);
        Some(patches.len() as u8 - 1)
    };

    for s in segments.iter_mut().filter(|s| s.patch == p) {
        let ends = [side_of(s.edges[0]), side_of(s.edges[1])];
        s.patch = if ends.contains(&Side::C) && ends.contains(&Side::D) {
            middle.expect("a side-crossing segment implies a middle patch")
        } else if ends.contains(&Side::C) {
            half_c
        } else {
            half_d
        };
    }

    match (singles.as_slice(), pairs.as_slice(), middle) {
        ([x, y], [], None) => chords.push(Chord {
            edges: [*x, *y],
            patches: [half_c, half_d],
        }),
        ([x], [[yc, yd]], Some(m)) => {
            chords.push(Chord {
                edges: [*x, *yc],
                patches: [half_c, m],
            });
            chords.push(Chord {
                edges: [*x, *yd],
                patches: [m, half_d],
            });
        }
        ([], [[xc, xd], [yc, yd]], Some(m)) => {
            chords.push(Chord {
                edges: [*xc, *yc],
                patches: [half_c, m],
            });
            chords.push(Chord {
                edges: [*xd, *yd],
                patches: [m, half_d],
            });
        }
        _ => unreachable!("case {mask}: {singles:?} {pairs:?}"),
    }
}

struct UnionFind {
    parent: [usize; 12],
}

impl UnionFind {
    fn new() -> Self {
        UnionFind {
            parent: std::array::from_fn(|i| i),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        self.parent[ra] = rb;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 3 edges incident to a corner, as a mask.
    fn corner_edge_mask(corner: usize) -> u16 {
        let mut m = 0;
        for axis in 0..3 {
            m |= 1 << edge_between(corner, corner ^ (1 << axis));
        }
        m
    }

    /// The 4 cycle edges of a face.
    fn face_edges(axis: usize, side: usize) -> [usize; 4] {
        let c = face_corners(axis, side);
        std::array::from_fn(|i| edge_between(c[i], c[(i + 1) % 4]))
    }

    #[test]
    fn every_case_covers_its_crossed_edges() {
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            let crossed = crossed_edges(mask);

            let mut union = 0u16;
            for &p in &info.patches {
                assert_ne!(p, 0, "case {mask}: empty patch");
                union |= p;
            }
            assert_eq!(union, crossed, "case {mask}: patch union != crossed edges");
            for e in 0..12 {
                let containing: Vec<u8> = (0..info.patches.len() as u8)
                    .filter(|&p| info.patches[p as usize] & (1 << e) != 0)
                    .collect();
                let mut listed = info.patches_of(e).to_vec();
                listed.sort_unstable();
                assert_eq!(listed, containing, "case {mask} edge {e}");
                if crossed & (1 << e) == 0 {
                    assert!(listed.is_empty(), "case {mask} edge {e}: uncrossed in a patch");
                } else {
                    assert!(!listed.is_empty(), "case {mask} edge {e}: crossed in no patch");
                }
            }
            // complementing the case flips no crossings
            assert_eq!(crossed, crossed_edges(!mask));
        }
    }

    #[test]
    fn segments_pair_up_the_crossings_of_their_own_face() {
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            let crossed = crossed_edges(mask);
            for axis in 0..3 {
                for side in 0..2 {
                    let f = face_index(axis, side);
                    let edges = face_edges(axis, side);
                    let n = edges.iter().filter(|&&e| crossed & (1 << e) != 0).count();
                    let segs: Vec<&Segment> =
                        info.segments.iter().filter(|s| s.face == f).collect();
                    assert_eq!(segs.len(), n / 2, "case {mask} face {f}: {n} crossings");
                    let mut covered = 0u16;
                    for s in segs {
                        let owner = info.patches[s.patch as usize];
                        for &e in &s.edges {
                            assert!(edges.contains(&e), "case {mask}: segment off its face");
                            assert_ne!(crossed & (1 << e), 0, "case {mask}: uncrossed in segment");
                            assert_eq!(covered & (1 << e), 0, "case {mask}: crossing used twice");
                            assert_ne!(owner & (1 << e), 0, "case {mask}: owner lacks the edge");
                            covered |= 1 << e;
                            let partner = s.edges[0] ^ s.edges[1] ^ e;
                            assert_eq!(info.segment_partner(f, e), Some(partner));
                            assert_eq!(info.segment_owner(f, e), Some(s.patch));
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn every_patch_is_a_disk_bounded_by_one_cycle() {
        // Manifold Dual Contouring, Proposition 1 condition (1):
        // chi(S_v) = 1 for the surface collapsed onto each vertex. Here
        // S_v is one patch. Its boundary runs along the face segments it
        // owns plus the chords on it; every one of its crossings must be
        // on exactly two of those, and they must make a single cycle. A
        // genus-0 piece bounded by one loop is a disk — which is what
        // makes collapsing the patch to one vertex topologically safe.
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            for (i, &patch) in info.patches.iter().enumerate() {
                let edges: Vec<usize> = (0..12).filter(|&e| patch & (1 << e) != 0).collect();
                assert!(edges.len() >= 3, "case {mask} patch {i}: fewer than 3 crossings");
                let mut degree = [0usize; 12];
                let mut uf = UnionFind::new();
                let boundary = info
                    .segments
                    .iter()
                    .filter(|s| s.patch == i as u8)
                    .map(|s| s.edges)
                    .chain(
                        info.chords
                            .iter()
                            .filter(|c| c.patches.contains(&(i as u8)))
                            .map(|c| c.edges),
                    );
                for [a, b] in boundary {
                    assert!(
                        patch & (1 << a) != 0 && patch & (1 << b) != 0,
                        "case {mask} patch {i}: boundary edge {a}-{b} leaves the patch"
                    );
                    degree[a] += 1;
                    degree[b] += 1;
                    uf.union(a, b);
                }
                for &e in &edges {
                    assert_eq!(degree[e], 2, "case {mask} patch {i}: edge {e} degree");
                }
                let root = uf.find(edges[0]);
                for &e in &edges {
                    assert_eq!(
                        uf.find(e),
                        root,
                        "case {mask} patch {i}: boundary is more than one cycle"
                    );
                }
            }
        }
    }

    #[test]
    fn no_patch_owns_both_segments_of_a_face() {
        // Manifold Dual Contouring, Proposition 1 condition (2), in the
        // form that matters for the dual: if one patch owned both
        // segments of a face, both would map to the same mesh edge and
        // four polygons would share it. Before the split the 36 joint
        // cases did exactly that; now every face's two segments go to
        // different patches, in every case.
        let mut split_cases = 0;
        let mut two_patch = 0;
        let mut three_patch = 0;
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            for f in 0..6 {
                let owners: Vec<u8> = info
                    .segments
                    .iter()
                    .filter(|s| s.face == f)
                    .map(|s| s.patch)
                    .collect();
                if owners.len() == 2 {
                    assert_ne!(owners[0], owners[1], "case {mask} face {f}: pinch");
                }
            }
            if info.chords.is_empty() {
                continue;
            }
            split_cases += 1;
            match info.chords.len() {
                1 => {
                    two_patch += 1;
                    // the bare pair: two halves of 4 crossings sharing 2
                    let [x, y] = info.chords[0].edges;
                    assert_eq!(info.patches.len(), 2, "case {mask}");
                    for &p in &info.patches {
                        assert_eq!(p.count_ones(), 4, "case {mask}");
                        assert!(p & (1 << x) != 0 && p & (1 << y) != 0, "case {mask}");
                    }
                    assert_eq!(info.patches_of(x).len(), 2);
                    assert_eq!(info.patches_of(y).len(), 2);
                }
                2 => {
                    three_patch += 1;
                    assert_eq!(info.patches.len(), 3, "case {mask}");
                    // one crossing is on both chords and so in all three
                    let shared = (0..12).filter(|&e| info.patches_of(e).len() == 3).count();
                    assert_eq!(shared, 1, "case {mask}");
                    let middle = info.chords[0].patches[1];
                    assert_eq!(info.chords[1].patches[0], middle, "case {mask}: chords chain");
                    assert_eq!(info.patches[middle as usize].count_ones(), 3, "case {mask}");
                }
                n => panic!("case {mask}: {n} chords"),
            }
        }
        assert_eq!(split_cases, 36, "joint cases");
        assert_eq!(two_patch, 12, "bare pairs");
        assert_eq!(three_patch, 24, "extended pairs");
    }

    #[test]
    fn patches_at_a_crossing_are_ordered_between_its_two_faces() {
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            for e in 0..12 {
                let chain = info.patches_of(e);
                if chain.is_empty() {
                    continue;
                }
                let [f_lo, f_hi] = edge_faces(e);
                assert_eq!(info.segment_owner(f_lo, e), Some(chain[0]), "case {mask} edge {e}");
                assert_eq!(
                    info.segment_owner(f_hi, e),
                    Some(*chain.last().unwrap()),
                    "case {mask} edge {e}"
                );
                for w in chain.windows(2) {
                    assert!(
                        info.chords.iter().any(|c| c.edges.contains(&e)
                            && c.patches.contains(&w[0])
                            && c.patches.contains(&w[1])),
                        "case {mask} edge {e}: {} and {} not joined by a chord",
                        w[0],
                        w[1]
                    );
                }
                if chain.len() == 1 {
                    assert!(
                        !info.chords.iter().any(|c| c.edges.contains(&e)),
                        "case {mask} edge {e}: chord end in one patch"
                    );
                }
            }
        }
    }

    #[test]
    fn known_cases_have_known_patches() {
        // empty / full: no surface
        assert!(case_info(0).patches.is_empty());
        assert!(case_info(255).patches.is_empty());

        // one active corner: one triangular patch on that corner's 3 edges
        for corner in 0..8 {
            let info = case_info(1 << corner);
            assert_eq!(
                info.patches,
                vec![corner_edge_mask(corner)],
                "corner {corner}"
            );
        }

        // two face-adjacent actives: one 4-edge tube
        let info = case_info(0b0000_0011); // corners 0 and 1
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 4);
        assert!(info.chords.is_empty(), "nothing ambiguous");

        // Two actives meeting along an edge: the ambiguous face is
        // resolved to join them, which would merge their caps into one
        // 6-edge tube — the pinch — so the tube is cut in two along the
        // plane through both corners. Each half holds that face's segment
        // on its side plus the two z-edge crossings at the corners, which
        // the halves share.
        let info = case_info(0b0000_1001); // corners 0 and 3
        assert_eq!(info.patches.len(), 2);
        let z0 = edge_between(0, 4);
        let z3 = edge_between(3, 7);
        let shared = 1 << z0 | 1 << z3;
        assert_eq!(info.patches[0] & info.patches[1], shared);
        assert_eq!(info.patches[0] | info.patches[1], crossed_edges(0b0000_1001));
        assert_eq!(
            info.chords,
            vec![Chord {
                edges: [z3, z0],
                patches: [0, 1]
            }],
            "one chord between the corner edges"
        );
        // corner 1 (x = 1, y = 0) is on the first half's side with the
        // segment flanking it; corner 2 on the other's
        assert!(info.patches[0] & corner_edge_mask(1) == corner_edge_mask(1) & crossed_edges(9));
        assert!(info.patches[1] & corner_edge_mask(2) == corner_edge_mask(2) & crossed_edges(9));

        // two body-diagonal actives: 2 separate corner patches, since no
        // face sees both corners for the rule to act on
        let info = case_info(0b1000_0001); // corners 0 and 7
        assert_eq!(info.patches, vec![corner_edge_mask(0), corner_edge_mask(7)]);
        assert!(info.chords.is_empty());

        // full bottom face: one flat 4-edge sheet
        let info = case_info(0b0000_1111);
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 4);

        // Checkerboard: still 4 caps, but now around the *empty* corners
        // rather than the active ones — the material is joined through
        // every ambiguous face and the empty space is what gets cut apart.
        // Each cap takes 2 of an ambiguous face's 4 crossings, so nothing
        // is merged and nothing needs splitting.
        let info = case_info(0b0110_1001); // active 0, 3, 5, 6
        assert_eq!(info.patches.len(), 4);
        for corner in [1usize, 2, 4, 7] {
            assert!(
                info.patches.contains(&corner_edge_mask(corner)),
                "missing patch for empty corner {corner}"
            );
        }
        assert!(info.chords.is_empty());

        // Complement of the edge-touching case: now it is the two *empty*
        // diagonal corners that get separated, so this splits into two
        // triangles where the original merged into a tube.
        let info = case_info(!0b0000_1001);
        assert_eq!(info.patches.len(), 2);
        assert!(info.patches.iter().all(|p| p.count_ones() == 3));
        assert!(info.chords.is_empty());

        // The pair with one voxel extended upward: the cut from the
        // z-edge at corner 3 fans to the two crossings around corner 4,
        // and the triangle between is a patch of its own.
        let info = case_info(0b0001_1001); // corners 0, 3, 4
        assert_eq!(info.patches.len(), 3);
        assert_eq!(info.chords.len(), 2);
        assert!(info.chords.iter().all(|c| c.edges.contains(&z3)));
        assert_eq!(info.patches_of(z3).len(), 3);
    }

    #[test]
    fn patch_counts_are_complement_symmetric_for_unambiguous_cases() {
        // For cases with no 4-crossed face the pairing rule never fires,
        // so complementing must keep the exact same patch structure.
        for mask in 0u16..256 {
            let mask = mask as u8;
            let ambiguous = (0..3).any(|axis| {
                (0..2).any(|side| {
                    let c = face_corners(axis, side);
                    let states: Vec<_> = c.iter().map(|&c| (mask >> c) & 1).collect();
                    states[0] == states[2] && states[1] == states[3] && states[0] != states[1]
                })
            });
            if !ambiguous {
                assert_eq!(
                    case_info(mask).patches,
                    case_info(!mask).patches,
                    "case {mask}"
                );
            }
        }
    }
}
