//! Engine-agnostic render buffers: flat-shaded triangles.

use glam::Vec3;

use crate::poly_mesh::PolyMesh;

/// Flat position/normal/index buffers in the layout rendering engines
/// expect. Flat shading: every triangle has its own 3 vertices carrying
/// the triangle's face normal — normals are derived from geometry here,
/// never during meshing.
#[derive(Debug, Clone, Default)]
pub struct MeshData {
    pub positions: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub indices: Vec<u32>,
}

impl MeshData {
    /// Fan-triangulates each polygon into flat-shaded triangles. Triangles
    /// with (near-)zero area — e.g. from dual vertices that coincide — are
    /// dropped; they contribute nothing visible and have no meaningful
    /// normal.
    pub fn flat_from_polys(poly_mesh: &PolyMesh) -> Self {
        let mut mesh = MeshData::default();
        for f in &poly_mesh.faces {
            let p0 = poly_mesh.positions[f[0] as usize];
            for w in f[1..].windows(2) {
                let p = [
                    p0,
                    poly_mesh.positions[w[0] as usize],
                    poly_mesh.positions[w[1] as usize],
                ];
                let normal = (p[1] - p[0]).cross(p[2] - p[0]);
                if normal.length_squared() <= 1e-12 {
                    continue;
                }
                let normal = normal.normalize();
                let base = mesh.positions.len() as u32;
                mesh.positions.extend(p);
                mesh.normals.extend([normal; 3]);
                mesh.indices.extend([base, base + 1, base + 2]);
            }
        }
        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expansion_is_flat_and_skips_degenerates() {
        let mut poly = PolyMesh::new();
        let v = [
            poly.push_vertex(Vec3::new(0.0, 0.0, 0.0)),
            poly.push_vertex(Vec3::new(1.0, 0.0, 0.0)),
            poly.push_vertex(Vec3::new(1.0, 1.0, 0.0)),
            poly.push_vertex(Vec3::new(0.5, 1.5, 0.0)),
            poly.push_vertex(Vec3::new(0.0, 1.0, 0.0)),
        ];
        poly.push_face(v.to_vec()); // one CCW pentagon in the xy plane
        let d = poly.push_vertex(Vec3::splat(5.0));
        poly.push_face(vec![d, d, d]); // fully degenerate

        let mesh = MeshData::flat_from_polys(&poly);
        assert_eq!(mesh.indices.len(), 9, "pentagon → three fan triangles");
        assert_eq!(mesh.positions.len(), 9, "flat shading duplicates vertices");
        // CCW polygon in the xy plane → +z face normals everywhere
        assert!(mesh.normals.iter().all(|n| n.abs_diff_eq(Vec3::Z, 1e-6)));
    }
}
