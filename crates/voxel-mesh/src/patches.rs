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
//! since that is what makes the two cells sharing a face agree on it.
//!
//! ## Patches, segments and manifoldness
//!
//! The mesher builds the full dual of this structure: patch → vertex,
//! segment → mesh edge, crossing → polygon (see [`crate::dmc`]). Manifold
//! Dual Contouring (Schaefer, Ju & Warren, TVCG) gives two conditions
//! under which collapsing a piece of surface onto one vertex stays 2-
//! manifold — its Euler characteristic must be 1, and it must meet the
//! four edges of each cell face in 0 or 2 points. Both are properties of
//! this table, and both are tested here:
//!
//! * Condition 1 holds for every patch of every case: the segments make
//!   each patch's crossings a single cycle, so the patch is a disk.
//! * Condition 2 fails in exactly 36 of the 256 cases — those where one
//!   patch swallows all four crossed edges of an ambiguous face, flagged
//!   by [`CaseInfo::face_merged`]. There the face's two segments land on
//!   the same pair of patch vertices, collapsing two mesh edges into one
//!   shared by four polygons: the pinch. [`crate::dmc`] pulls them apart
//!   by giving such a segment its own vertex.

use std::sync::OnceLock;

use crate::cube::{EDGE_ENDPOINTS, edge_between, face_corners, face_index};

pub const NO_PATCH: u8 = u8::MAX;

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
}

/// One case's surface decomposition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaseInfo {
    /// Each patch as a bitmask over the 12 edges. Every crossed edge is in
    /// exactly one patch; uncrossed edges are in none.
    pub patches: Vec<u16>,
    /// `edge_patch[e]` = index into `patches` of the patch containing edge
    /// `e`, or [`NO_PATCH`] if `e` is not crossed.
    pub edge_patch: [u8; 12],
    /// Every surface segment on every face of the cell.
    pub segments: Vec<Segment>,
    /// Per face ([`face_index`]), whether all four of its crossed edges
    /// belong to a single patch — the configuration that pinches the
    /// surface unless the face's two segments are separated. See the
    /// module docs.
    pub face_merged: [bool; 6],
}

impl CaseInfo {
    /// The edge that `edge` is joined to by a segment on `face`, if any.
    pub fn segment_partner(&self, face: usize, edge: usize) -> Option<usize> {
        self.segments.iter().find_map(|s| match s.edges {
            _ if s.face != face => None,
            [a, b] if a == edge => Some(b),
            [a, b] if b == edge => Some(a),
            _ => None,
        })
    }
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
    let mut edge_patch = [NO_PATCH; 12];
    for (e, ep) in edge_patch.iter_mut().enumerate() {
        if crossed & (1 << e) == 0 {
            continue;
        }
        let root = uf.find(e);
        if root_to_patch[root] == NO_PATCH {
            root_to_patch[root] = patches.len() as u8;
            patches.push(0);
        }
        let p = root_to_patch[root];
        patches[p as usize] |= 1 << e;
        *ep = p;
    }

    // A face is "merged" when both of its segments ended up in the same
    // patch — only possible for the ambiguous, four-crossing faces.
    let mut face_merged = [false; 6];
    for (f, merged) in face_merged.iter_mut().enumerate() {
        let on_face: Vec<&Segment> = segments.iter().filter(|s| s.face == f).collect();
        *merged = on_face.len() == 2
            && on_face
                .iter()
                .all(|s| edge_patch[s.edges[0]] == edge_patch[on_face[0].edges[0]]);
    }

    CaseInfo {
        patches,
        edge_patch,
        segments,
        face_merged,
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
    fn every_case_partitions_its_crossed_edges() {
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            let crossed = crossed_edges(mask);

            let mut union = 0u16;
            for (i, &p) in info.patches.iter().enumerate() {
                assert_ne!(p, 0, "case {mask}: empty patch");
                assert_eq!(union & p, 0, "case {mask}: patches overlap");
                union |= p;
                for e in 0..12 {
                    if p & (1 << e) != 0 {
                        assert_eq!(info.edge_patch[e], i as u8, "case {mask} edge {e}");
                    }
                }
            }
            assert_eq!(union, crossed, "case {mask}: patch union != crossed edges");
            for e in 0..12 {
                if crossed & (1 << e) == 0 {
                    assert_eq!(info.edge_patch[e], NO_PATCH, "case {mask} edge {e}");
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
                        for &e in &s.edges {
                            assert!(edges.contains(&e), "case {mask}: segment off its face");
                            assert_ne!(crossed & (1 << e), 0, "case {mask}: uncrossed in segment");
                            assert_eq!(covered & (1 << e), 0, "case {mask}: crossing used twice");
                            covered |= 1 << e;
                            let partner = s.edges[0] ^ s.edges[1] ^ e;
                            assert_eq!(info.segment_partner(f, e), Some(partner));
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
        // S_v is one patch. Its boundary runs along the face segments,
        // and every crossing lies on exactly two faces, so the segments
        // make the patch's crossings a disjoint union of cycles; for a
        // genus-0 piece chi = 2 - loops. A single cycle therefore means
        // chi = 1 — a disk — which is what makes collapsing the patch to
        // one vertex topologically safe.
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            for (i, &patch) in info.patches.iter().enumerate() {
                let edges: Vec<usize> = (0..12).filter(|&e| patch & (1 << e) != 0).collect();
                let mut degree = [0usize; 12];
                let mut uf = UnionFind::new();
                for s in &info.segments {
                    if patch & (1 << s.edges[0]) == 0 {
                        continue;
                    }
                    assert_ne!(
                        patch & (1 << s.edges[1]),
                        0,
                        "case {mask}: segment {s:?} straddles two patches"
                    );
                    degree[s.edges[0]] += 1;
                    degree[s.edges[1]] += 1;
                    uf.union(s.edges[0], s.edges[1]);
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
    fn only_merged_ambiguous_faces_break_the_manifold_criterion() {
        // Manifold Dual Contouring, Proposition 1 condition (2): a
        // vertex's surface must meet the four edges of each cell face in
        // 0 or 2 points. 4 means the face's two segments both run
        // between the same pair of patch vertices, so two mesh edges
        // collapse into one shared by four polygons — the pinch. Those
        // cases are exactly `face_merged`, and `dmc` gives such a segment
        // its own vertex to pull them apart again.
        let mut masks_affected = 0;
        let mut merged_faces = 0;
        for mask in 0u16..256 {
            let mask = mask as u8;
            let info = case_info(mask);
            let mut here = 0;
            for axis in 0..3 {
                for side in 0..2 {
                    let f = face_index(axis, side);
                    let edges = face_edges(axis, side);
                    let mut four = false;
                    for p in 0..info.patches.len() as u8 {
                        let n = edges.iter().filter(|&&e| info.edge_patch[e] == p).count();
                        assert!(
                            n == 0 || n == 2 || n == 4,
                            "case {mask} face {f}: {n} crossings in patch {p}"
                        );
                        four |= n == 4;
                    }
                    assert_eq!(four, info.face_merged[f], "case {mask} face {f} flag");
                    here += usize::from(four);
                }
            }
            if here > 0 {
                assert_eq!(here, 1, "case {mask}: {here} merged faces, expected 1");
                masks_affected += 1;
            }
            merged_faces += here;
        }
        assert_eq!(masks_affected, 36, "cases needing a segment vertex");
        assert_eq!(merged_faces, 36, "one merged face each");
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
        assert!(!info.face_merged.iter().any(|&m| m), "nothing ambiguous");
        // Two actives meeting along an edge: the ambiguous faces are
        // resolved to join them, so their caps merge into one 6-edge tube
        // rather than two triangles — and that merge is exactly the case
        // needing a segment vertex, on the face the two corners share.
        let info = case_info(0b0000_1001); // corners 0 and 3
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 6);
        assert_eq!(
            info.face_merged,
            [false, false, false, false, true, false],
            "corners 0 and 3 are diagonal on the z = 0 face"
        );

        // two body-diagonal actives: 2 separate corner patches, since no
        // face sees both corners for the rule to act on
        let info = case_info(0b1000_0001); // corners 0 and 7
        assert_eq!(info.patches, vec![corner_edge_mask(0), corner_edge_mask(7)]);
        assert!(!info.face_merged.iter().any(|&m| m));

        // full bottom face: one flat 4-edge sheet
        let info = case_info(0b0000_1111);
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 4);

        // Checkerboard: still 4 caps, but now around the *empty* corners
        // rather than the active ones — the material is joined through
        // every ambiguous face and the empty space is what gets cut apart.
        // Each cap takes 2 of an ambiguous face's 4 crossings, so nothing
        // is merged and no segment vertex is needed.
        let info = case_info(0b0110_1001); // active 0, 3, 5, 6
        assert_eq!(info.patches.len(), 4);
        for corner in [1usize, 2, 4, 7] {
            assert!(
                info.patches.contains(&corner_edge_mask(corner)),
                "missing patch for empty corner {corner}"
            );
        }
        assert!(!info.face_merged.iter().any(|&m| m));

        // Complement of the edge-touching case: now it is the two *empty*
        // diagonal corners that get separated, so this splits into two
        // triangles where the original merged into a tube.
        let info = case_info(!0b0000_1001);
        assert_eq!(info.patches.len(), 2);
        assert!(info.patches.iter().all(|p| p.count_ones() == 3));
        assert!(!info.face_merged.iter().any(|&m| m));
    }

    #[test]
    fn patch_counts_are_complement_symmetric_for_unambiguous_cases() {
        // For cases with no 4-crossed face the pairing rule never fires,
        // so complementing must keep the exact same patch structure.
        for mask in 0u16..256 {
            let mask = mask as u8;
            let ambiguous = (0..3).any(|axis| {
                (0..2).any(|side| {
                    face_corners(axis, side)
                        .iter()
                        .filter(|&&c| (mask >> c) & 1 == 1)
                        .count()
                        == 2
                        && {
                            let c = face_corners(axis, side);
                            let states: Vec<_> = c.iter().map(|&c| (mask >> c) & 1).collect();
                            states[0] == states[2]
                                && states[1] == states[3]
                                && states[0] != states[1]
                        }
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
