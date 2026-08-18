//! Mesh topology checks used by tests (and available to debug tooling):
//! welding, edge manifoldness, Euler characteristic, signed volume.

use std::collections::HashMap;

use crate::quad_mesh::QuadMesh;

/// Merges vertices at bit-identical positions and remaps quads. Exact
/// comparison is intentional: a dual cell recomputed anywhere (a chunk's
/// interior pass, a seam pass, a single-domain reference mesh) sees the
/// same corner leaves in the same order, so shared vertices come out
/// bit-identical — anything less than exact equality would paper over a
/// real mismatch.
pub fn weld(mesh: &QuadMesh) -> QuadMesh {
    let mut map: HashMap<[u32; 3], u32> = HashMap::new();
    let mut out = QuadMesh::new();
    let mut remap = Vec::with_capacity(mesh.positions.len());
    for &p in &mesh.positions {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let idx = *map.entry(key).or_insert_with(|| out.push_vertex(p));
        remap.push(idx);
    }
    for q in &mesh.quads {
        out.push_quad(q.map(|i| remap[i as usize]));
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EdgeStats {
    /// Edges used by exactly one face — holes/cracks in a closed surface.
    pub boundary: usize,
    /// Edges used by exactly two faces.
    pub manifold: usize,
    /// Edges used by three or more faces.
    pub non_manifold: usize,
}

/// Counts undirected quad edges by how many faces use them. Run on a
/// *welded* mesh; self-edges from collapsed quad corners are ignored.
pub fn edge_stats(mesh: &QuadMesh) -> EdgeStats {
    let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
    for q in &mesh.quads {
        for i in 0..4 {
            let (a, b) = (q[i], q[(i + 1) % 4]);
            if a == b {
                continue;
            }
            *counts.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let mut stats = EdgeStats::default();
    for &n in counts.values() {
        match n {
            1 => stats.boundary += 1,
            2 => stats.manifold += 1,
            _ => stats.non_manifold += 1,
        }
    }
    stats
}

/// V − E + F of a welded mesh (2 for a sphere, 0 for a torus). Faces with
/// collapsed corners count as the smaller polygon they actually are.
pub fn euler_characteristic(mesh: &QuadMesh) -> i64 {
    let mut edges: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    let mut used: std::collections::HashSet<u32> = std::collections::HashSet::new();
    let mut faces = 0i64;
    for q in &mesh.quads {
        let mut degenerate_to_nothing = true;
        for i in 0..4 {
            let (a, b) = (q[i], q[(i + 1) % 4]);
            used.insert(a);
            if a != b {
                degenerate_to_nothing = false;
                edges.insert((a.min(b), a.max(b)));
            }
        }
        if !degenerate_to_nothing {
            faces += 1;
        }
    }
    used.len() as i64 - edges.len() as i64 + faces
}

/// Signed volume enclosed by the mesh: positive when faces are wound
/// counter-clockwise seen from outside (the crate's convention).
pub fn signed_volume(mesh: &QuadMesh) -> f64 {
    let mut six_v = 0.0f64;
    for &[a, b, c, d] in &mesh.quads {
        for [i, j, k] in [[a, b, c], [a, c, d]] {
            let (p, q, r) = (
                mesh.positions[i as usize].as_dvec3(),
                mesh.positions[j as usize].as_dvec3(),
                mesh.positions[k as usize].as_dvec3(),
            );
            six_v += p.dot(q.cross(r));
        }
    }
    six_v / 6.0
}

/// Convenience for tests: welded edge stats say the surface is closed and
/// every edge is shared by exactly two faces.
pub fn is_watertight(mesh: &QuadMesh) -> bool {
    let stats = edge_stats(&weld(mesh));
    stats.boundary == 0 && stats.non_manifold == 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    /// A unit cube built as 6 CCW-outward quads.
    fn cube() -> QuadMesh {
        let mut qm = QuadMesh::new();
        let corners: Vec<u32> = (0..8)
            .map(|i| {
                qm.push_vertex(Vec3::new(
                    (i & 1) as f32,
                    ((i >> 1) & 1) as f32,
                    ((i >> 2) & 1) as f32,
                ))
            })
            .collect();
        let faces = [
            [0, 2, 3, 1], // -z
            [4, 5, 7, 6], // +z
            [0, 4, 6, 2], // -x
            [1, 3, 7, 5], // +x
            [0, 1, 5, 4], // -y
            [2, 6, 7, 3], // +y
        ];
        for f in faces {
            qm.push_quad(f.map(|i: usize| corners[i]));
        }
        qm
    }

    #[test]
    fn cube_is_watertight_with_positive_volume_and_euler_2() {
        let qm = cube();
        assert!(is_watertight(&qm));
        let welded = weld(&qm);
        assert_eq!(euler_characteristic(&welded), 2);
        assert!((signed_volume(&welded) - 1.0).abs() < 1e-9);
        let stats = edge_stats(&welded);
        assert_eq!(stats.manifold, 12);
    }

    #[test]
    fn weld_merges_bit_identical_positions() {
        let mut qm = QuadMesh::new();
        let a = qm.push_vertex(Vec3::new(1.5, 2.5, 3.5));
        let b = qm.push_vertex(Vec3::new(1.5, 2.5, 3.5));
        let c = qm.push_vertex(Vec3::new(0.0, 0.0, 0.0));
        let d = qm.push_vertex(Vec3::new(0.0, 1.0, 0.0));
        qm.push_quad([a, b, c, d]);
        let welded = weld(&qm);
        assert_eq!(welded.positions.len(), 3);
        assert_eq!(welded.quads[0][0], welded.quads[0][1]);
    }

    #[test]
    fn missing_face_shows_as_boundary_edges() {
        let mut qm = cube();
        qm.quads.pop();
        assert!(!is_watertight(&qm));
        assert_eq!(edge_stats(&weld(&qm)).boundary, 4);
    }
}
