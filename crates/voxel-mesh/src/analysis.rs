//! Mesh topology checks used by tests (and available to debug tooling):
//! welding, edge manifoldness, Euler characteristic, signed volume.

use std::collections::{HashMap, HashSet};

use crate::poly_mesh::PolyMesh;

/// Merges vertices at bit-identical positions and remaps faces. Exact
/// comparison is intentional: a dual cell recomputed anywhere (a chunk's
/// interior pass, a seam pass, a single-domain reference mesh) sees the
/// same corner leaves in the same order, so shared vertices come out
/// bit-identical — anything less than exact equality would paper over a
/// real mismatch.
pub fn weld(mesh: &PolyMesh) -> PolyMesh {
    let mut map: HashMap<[u32; 3], u32> = HashMap::new();
    let mut out = PolyMesh::new();
    let mut remap = Vec::with_capacity(mesh.positions.len());
    for &p in &mesh.positions {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let idx = *map.entry(key).or_insert_with(|| out.push_vertex(p));
        remap.push(idx);
    }
    for f in &mesh.faces {
        out.push_face(f.iter().map(|&i| remap[i as usize]).collect());
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

/// Counts undirected face edges by how many faces use them. Run on a
/// *welded* mesh; self-edges from collapsed face corners are ignored.
pub fn edge_stats(mesh: &PolyMesh) -> EdgeStats {
    let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
    for f in &mesh.faces {
        for (a, b) in PolyMesh::face_edges(f) {
            if a != b {
                *counts.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
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
pub fn euler_characteristic(mesh: &PolyMesh) -> i64 {
    let mut edges: HashSet<(u32, u32)> = HashSet::new();
    let mut used: HashSet<u32> = HashSet::new();
    let mut faces = 0i64;
    for f in &mesh.faces {
        let mut has_edge = false;
        for (a, b) in PolyMesh::face_edges(f) {
            used.insert(a);
            if a != b {
                has_edge = true;
                edges.insert((a.min(b), a.max(b)));
            }
        }
        if has_edge {
            faces += 1;
        }
    }
    used.len() as i64 - edges.len() as i64 + faces
}

/// Signed volume enclosed by the mesh: positive when faces are wound
/// counter-clockwise seen from outside (the crate's convention). Faces are
/// fan-triangulated.
pub fn signed_volume(mesh: &PolyMesh) -> f64 {
    let mut six_v = 0.0f64;
    for f in &mesh.faces {
        let p0 = mesh.positions[f[0] as usize].as_dvec3();
        for w in f[1..].windows(2) {
            let (q, r) = (
                mesh.positions[w[0] as usize].as_dvec3(),
                mesh.positions[w[1] as usize].as_dvec3(),
            );
            six_v += p0.dot(q.cross(r));
        }
    }
    six_v / 6.0
}

/// Convenience for tests: welded edge stats say the surface is closed and
/// every edge is shared by exactly two faces.
pub fn is_watertight(mesh: &PolyMesh) -> bool {
    let stats = edge_stats(&weld(mesh));
    stats.boundary == 0 && stats.non_manifold == 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    /// A unit cube built as 6 CCW-outward quads.
    fn cube() -> PolyMesh {
        let mut mesh = PolyMesh::new();
        let corners: Vec<u32> = (0..8)
            .map(|i| {
                mesh.push_vertex(Vec3::new(
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
            mesh.push_face(f.iter().map(|&i: &usize| corners[i]).collect());
        }
        mesh
    }

    #[test]
    fn cube_is_watertight_with_positive_volume_and_euler_2() {
        let mesh = cube();
        assert!(is_watertight(&mesh));
        let welded = weld(&mesh);
        assert_eq!(euler_characteristic(&welded), 2);
        assert!((signed_volume(&welded) - 1.0).abs() < 1e-9);
        assert_eq!(edge_stats(&welded).manifold, 12);
    }

    #[test]
    fn weld_merges_bit_identical_positions() {
        let mut mesh = PolyMesh::new();
        let a = mesh.push_vertex(Vec3::new(1.5, 2.5, 3.5));
        let b = mesh.push_vertex(Vec3::new(1.5, 2.5, 3.5));
        let c = mesh.push_vertex(Vec3::new(0.0, 0.0, 0.0));
        let d = mesh.push_vertex(Vec3::new(0.0, 1.0, 0.0));
        mesh.push_face(vec![a, b, c, d]);
        let welded = weld(&mesh);
        assert_eq!(welded.positions.len(), 3);
        assert_eq!(welded.faces[0][0], welded.faces[0][1]);
    }

    #[test]
    fn missing_face_shows_as_boundary_edges() {
        let mut mesh = cube();
        mesh.faces.pop();
        assert!(!is_watertight(&mesh));
        assert_eq!(edge_stats(&weld(&mesh)).boundary, 4);
    }
}
