//! The Dual Marching Cubes patch table: for each of the 256 corner-state
//! cases, which crossed edges belong to which surface patch.
//!
//! Nielson's DMC places one dual vertex per *patch* (connected piece of
//! the marching-cubes surface inside a cell), not one per cell — that is
//! the difference from Surface Nets and what keeps thin/diagonal features
//! from collapsing.
//!
//! The table is generated, not transcribed: crossed edges follow directly
//! from corner states, and two crossed edges are in the same patch iff
//! they are joined by a surface segment on some face. A face with 4
//! crossed edges (diagonally opposite active corners) is ambiguous; the
//! fixed rule here pairs the segments so the **active corners are
//! separated**. That makes active material face-connected (6-connectivity)
//! — two voxels touching only along a diagonal get separate surface
//! sheets — and, because the rule depends only on the face's own corner
//! states, the two cells sharing any face always agree on it.

use std::sync::OnceLock;

use crate::cube::{EDGE_ENDPOINTS, edge_between, face_corners};

pub const NO_PATCH: u8 = u8::MAX;

/// One case's patch decomposition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaseInfo {
    /// Each patch as a bitmask over the 12 edges. Every crossed edge is in
    /// exactly one patch; uncrossed edges are in none.
    pub patches: Vec<u16>,
    /// `edge_patch[e]` = index into `patches` of the patch containing edge
    /// `e`, or [`NO_PATCH`] if `e` is not crossed.
    pub edge_patch: [u8; 12],
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

/// The patch decomposition for a corner-state mask (bit `i` set = corner
/// `i` active).
pub fn case_info(mask: u8) -> &'static CaseInfo {
    static TABLE: OnceLock<Vec<CaseInfo>> = OnceLock::new();
    &TABLE.get_or_init(|| (0u16..256).map(|m| build_case(m as u8)).collect())[mask as usize]
}

fn build_case(mask: u8) -> CaseInfo {
    let crossed = crossed_edges(mask);
    let mut uf = UnionFind::new();

    for axis in 0..3 {
        for side in 0..2 {
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
                2 => uf.union(face_crossed[0], face_crossed[1]),
                4 => {
                    // Alternating states around the cycle: join the two
                    // edges flanking each *active* corner, cutting the
                    // active corners apart.
                    for (i, &c) in corners.iter().enumerate() {
                        if (mask >> c) & 1 == 1 {
                            uf.union(cycle_edges[(i + 3) % 4], cycle_edges[i]);
                        }
                    }
                }
                n => unreachable!("a face cycle has 0, 2 or 4 crossings, got {n}"),
            }
        }
    }

    let mut patches: Vec<u16> = Vec::new();
    let mut root_to_patch = [NO_PATCH; 12];
    let mut edge_patch = [NO_PATCH; 12];
    for e in 0..12 {
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
        edge_patch[e] = p;
    }
    CaseInfo { patches, edge_patch }
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
    fn known_cases_have_known_patches() {
        // empty / full: no surface
        assert!(case_info(0).patches.is_empty());
        assert!(case_info(255).patches.is_empty());

        // one active corner: one triangular patch on that corner's 3 edges
        for corner in 0..8 {
            let info = case_info(1 << corner);
            assert_eq!(info.patches, vec![corner_edge_mask(corner)], "corner {corner}");
        }

        // two face-adjacent actives: one 4-edge tube
        let info = case_info(0b0000_0011); // corners 0 and 1
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 4);

        // two face-diagonal actives (share a face, not an edge): 2 patches
        let info = case_info(0b0000_1001); // corners 0 and 3
        assert_eq!(info.patches.len(), 2);
        assert!(info.patches.iter().all(|p| p.count_ones() == 3));

        // two body-diagonal actives: 2 separate corner patches
        let info = case_info(0b1000_0001); // corners 0 and 7
        assert_eq!(
            info.patches,
            vec![corner_edge_mask(0), corner_edge_mask(7)]
        );

        // full bottom face: one flat 4-edge sheet
        let info = case_info(0b0000_1111);
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 4);

        // checkerboard (4 mutually diagonal actives): 4 separate patches
        let info = case_info(0b0110_1001); // corners 0, 3, 5, 6
        assert_eq!(info.patches.len(), 4);
        for corner in [0usize, 3, 5, 6] {
            assert!(
                info.patches.contains(&corner_edge_mask(corner)),
                "missing patch for corner {corner}"
            );
        }

        // complement of the face-diagonal case: the two empty diagonal
        // corners *connect* (empty space is diagonal-connected under the
        // separate-the-actives rule), so their caps merge into one 6-edge
        // tube rather than two triangles
        let info = case_info(!0b0000_1001);
        assert_eq!(info.patches.len(), 1);
        assert_eq!(info.patches[0].count_ones(), 6);
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
                            states[0] == states[2] && states[1] == states[3] && states[0] != states[1]
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
