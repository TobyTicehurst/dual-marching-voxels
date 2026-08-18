//! Topology of the unit cell cube: corner, edge and face numbering shared
//! by the patch table and the meshing traversal.
//!
//! Corner indices reuse the octant bit encoding from `voxel_core::octree`:
//! bit 0 = +x, bit 1 = +y, bit 2 = +z. Everything here is derived from that
//! one convention.

use glam::IVec3;
pub use voxel_core::octree::octant_offset as corner_offset;

/// The 12 cube edges as `(corner, corner)` pairs, grouped by axis:
/// edges `0..4` run along x, `4..8` along y, `8..12` along z. Within an
/// axis group, edges are ordered by the binary value of the remaining two
/// bits of their lower corner.
pub const EDGE_ENDPOINTS: [(usize, usize); 12] = [
    // x axis: partner differs in bit 0
    (0b000, 0b001),
    (0b010, 0b011),
    (0b100, 0b101),
    (0b110, 0b111),
    // y axis: partner differs in bit 1
    (0b000, 0b010),
    (0b001, 0b011),
    (0b100, 0b110),
    (0b101, 0b111),
    // z axis: partner differs in bit 2
    (0b000, 0b100),
    (0b001, 0b101),
    (0b010, 0b110),
    (0b011, 0b111),
];

/// Axis (0 = x, 1 = y, 2 = z) an edge runs along.
pub const fn edge_axis(edge: usize) -> usize {
    edge / 4
}

/// The edge index connecting two corners that differ in exactly one bit.
pub fn edge_between(a: usize, b: usize) -> usize {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    let axis = (lo ^ hi).trailing_zeros() as usize;
    debug_assert_eq!(
        lo ^ hi,
        1 << axis,
        "corners {a} and {b} are not edge-adjacent"
    );
    // The two bits of `lo` other than `axis`, packed low-to-high, pick the
    // edge within the axis group — matching EDGE_ENDPOINTS' ordering.
    let u = (axis + 1) % 3;
    let v = (axis + 2) % 3;
    let (u, v) = if u < v { (u, v) } else { (v, u) };
    axis * 4 + ((lo >> u) & 1) + 2 * ((lo >> v) & 1)
}

/// The 4 corners of the face `bit(axis) == side`, in cycle order (each
/// consecutive pair, wrapping, is edge-adjacent).
pub fn face_corners(axis: usize, side: usize) -> [usize; 4] {
    let u = (axis + 1) % 3;
    let v = (axis + 2) % 3;
    let base = side << axis;
    [
        base,
        base | (1 << u),
        base | (1 << u) | (1 << v),
        base | (1 << v),
    ]
}

/// Offset of corner `i` scaled by `size` — where the corner sits for a
/// cell of that size.
pub fn corner_pos(min: IVec3, size: i32, corner: usize) -> IVec3 {
    min + corner_offset(corner) * size
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_between_inverts_edge_endpoints() {
        for (e, &(a, b)) in EDGE_ENDPOINTS.iter().enumerate() {
            assert_eq!(edge_between(a, b), e);
            assert_eq!(edge_between(b, a), e);
            assert_eq!(edge_axis(e), (a ^ b).trailing_zeros() as usize);
        }
    }

    #[test]
    fn face_cycles_are_edge_adjacent_and_cover_all_edges() {
        let mut edge_seen = [0u32; 12];
        for axis in 0..3 {
            for side in 0..2 {
                let c = face_corners(axis, side);
                for i in 0..4 {
                    let (a, b) = (c[i], c[(i + 1) % 4]);
                    assert_eq!((a ^ b).count_ones(), 1, "face cycle not edge-adjacent");
                    edge_seen[edge_between(a, b)] += 1;
                }
            }
        }
        // every cube edge lies on exactly 2 faces
        assert!(edge_seen.iter().all(|&n| n == 2), "{edge_seen:?}");
    }
}
