//! Engine-agnostic render buffers: flat-shaded triangles.

use glam::Vec3;

use crate::quad_mesh::QuadMesh;

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
    /// Expands quads to flat-shaded triangles (two per quad). Triangles
    /// with (near-)zero area — e.g. from dual vertices that coincide — are
    /// dropped; they contribute nothing visible and have no meaningful
    /// normal.
    pub fn flat_from_quads(quad_mesh: &QuadMesh) -> Self {
        let mut mesh = MeshData::default();
        for &[a, b, c, d] in &quad_mesh.quads {
            for tri in [[a, b, c], [a, c, d]] {
                let p = tri.map(|i| quad_mesh.positions[i as usize]);
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
        let mut qm = QuadMesh::new();
        let v = [
            qm.push_vertex(Vec3::new(0.0, 0.0, 0.0)),
            qm.push_vertex(Vec3::new(1.0, 0.0, 0.0)),
            qm.push_vertex(Vec3::new(1.0, 1.0, 0.0)),
            qm.push_vertex(Vec3::new(0.0, 1.0, 0.0)),
        ];
        qm.push_quad([v[0], v[1], v[2], v[3]]);
        // degenerate quad: all four corners at the same position
        let d = qm.push_vertex(Vec3::splat(5.0));
        qm.push_quad([d, d, d, d]);

        let mesh = MeshData::flat_from_quads(&qm);
        assert_eq!(mesh.indices.len(), 6, "one real quad → two triangles");
        assert_eq!(mesh.positions.len(), 6, "flat shading duplicates vertices");
        // CCW quad in the xy plane → +z face normals everywhere
        assert!(mesh.normals.iter().all(|n| n.abs_diff_eq(Vec3::Z, 1e-6)));
    }
}
