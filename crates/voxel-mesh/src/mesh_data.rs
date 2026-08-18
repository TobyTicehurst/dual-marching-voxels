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
    /// Triangulates each polygon into flat-shaded triangles. Triangles
    /// with (near-)zero area — e.g. from dual vertices that coincide —
    /// are dropped; they contribute nothing visible and have no
    /// meaningful normal.
    ///
    /// Dual faces are **not** reliably convex or planar: a scale
    /// transition turns a face into an n-gon (the hanging dual cells
    /// along the coarse face's boundary), and those come out mildly
    /// non-convex often enough to matter. A fan from one fixed vertex is
    /// only valid for convex polygons — on the rest it emits a triangle
    /// facing the wrong way, which renders as a backfacing sliver spike
    /// poking out of the surface. So:
    ///
    /// - **quads** keep two triangles, split along whichever diagonal
    ///   leaves both facing the same way as the polygon (exactly one
    ///   diagonal works when a quad is non-convex);
    /// - **n-gons** fan around the polygon's centroid, one triangle per
    ///   edge, which stays fold-free through mild non-convexity and
    ///   avoids picking an arbitrary crease.
    pub fn flat_from_polys(poly_mesh: &PolyMesh) -> Self {
        let mut mesh = MeshData::default();
        let mut corners: Vec<Vec3> = Vec::new();
        for f in &poly_mesh.faces {
            corners.clear();
            corners.extend(f.iter().map(|&i| poly_mesh.positions[i as usize]));
            let normal = newell_normal(&corners);

            match corners.len() {
                0..=2 => {}
                3 => mesh.push_triangle([corners[0], corners[1], corners[2]]),
                4 => {
                    let [a, b, c, d] = [corners[0], corners[1], corners[2], corners[3]];
                    if faces_same_way(a, b, c, normal) && faces_same_way(a, c, d, normal) {
                        mesh.push_triangle([a, b, c]);
                        mesh.push_triangle([a, c, d]);
                    } else {
                        mesh.push_triangle([b, c, d]);
                        mesh.push_triangle([b, d, a]);
                    }
                }
                n => {
                    let center = corners.iter().copied().sum::<Vec3>() / n as f32;
                    for i in 0..n {
                        mesh.push_triangle([center, corners[i], corners[(i + 1) % n]]);
                    }
                }
            }
        }
        mesh
    }

    fn push_triangle(&mut self, p: [Vec3; 3]) {
        let normal = (p[1] - p[0]).cross(p[2] - p[0]);
        if normal.length_squared() <= 1e-12 {
            return;
        }
        let base = self.positions.len() as u32;
        self.positions.extend(p);
        self.normals.extend([normal.normalize(); 3]);
        self.indices.extend([base, base + 1, base + 2]);
    }
}

/// Area-weighted (Newell) normal of a polygon: unlike the cross product
/// of the first three corners it stays well conditioned when those are
/// nearly collinear, which is common on transition n-gons.
fn newell_normal(corners: &[Vec3]) -> Vec3 {
    let mut n = Vec3::ZERO;
    for i in 0..corners.len() {
        n += corners[i].cross(corners[(i + 1) % corners.len()]);
    }
    n
}

/// Whether triangle `a b c` winds the same way as the polygon it belongs
/// to. Degenerate triangles count as agreeing — they are dropped later
/// and must not veto an otherwise good split.
fn faces_same_way(a: Vec3, b: Vec3, c: Vec3, polygon_normal: Vec3) -> bool {
    (b - a).cross(c - a).dot(polygon_normal) >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Triangle normals as emitted, one per triangle.
    fn triangle_normals(mesh: &MeshData) -> Vec<Vec3> {
        mesh.normals.chunks(3).map(|c| c[0]).collect()
    }

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
        assert_eq!(mesh.indices.len(), 15, "pentagon → one triangle per edge");
        assert_eq!(mesh.positions.len(), 15, "flat shading duplicates vertices");
        // CCW polygon in the xy plane → +z face normals everywhere
        assert!(mesh.normals.iter().all(|n| n.abs_diff_eq(Vec3::Z, 1e-6)));
    }

    #[test]
    fn quads_stay_two_triangles() {
        let mut poly = PolyMesh::new();
        let v: Vec<u32> = [
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ]
        .iter()
        .map(|&p| poly.push_vertex(p))
        .collect();
        poly.push_face(v);

        let mesh = MeshData::flat_from_polys(&poly);
        assert_eq!(mesh.indices.len(), 6);
        assert!(mesh.normals.iter().all(|n| n.abs_diff_eq(Vec3::Z, 1e-6)));
    }

    /// The bug this triangulation exists to avoid: a fan from corner 0 of
    /// a non-convex polygon emits a triangle facing backwards, which
    /// renders as a sliver spike sticking out of the surface.
    #[test]
    fn non_convex_polygons_never_emit_a_backfacing_triangle() {
        // A dart: corner 1 is reflex, so the fan triangle (0, 1, 2) would
        // wind clockwise while the polygon as a whole winds CCW.
        let dart = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 1.4, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(2.0, 4.0, 0.0),
        ];
        // Same shape with an extra corner, so it takes the n-gon path.
        let dart_pentagon = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 1.4, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(3.0, 3.0, 0.0),
            Vec3::new(1.0, 3.0, 0.0),
        ];

        for corners in [dart.to_vec(), dart_pentagon.to_vec()] {
            let mut poly = PolyMesh::new();
            let face: Vec<u32> = corners.iter().map(|&p| poly.push_vertex(p)).collect();
            poly.push_face(face);
            let expected = newell_normal(&corners).normalize();

            let mesh = MeshData::flat_from_polys(&poly);
            assert!(!mesh.indices.is_empty());
            for n in triangle_normals(&mesh) {
                assert!(
                    n.dot(expected) > 0.0,
                    "{}-gon emitted a triangle facing {n:?}, polygon faces {expected:?}",
                    corners.len()
                );
            }
        }
    }

    #[test]
    fn non_planar_quad_picks_a_diagonal_that_does_not_fold() {
        // Corner 3 lifted out of the plane and pulled inward, so the
        // 0-2 diagonal folds but the 1-3 diagonal does not.
        let corners = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(4.0, 4.0, 0.0),
            Vec3::new(1.6, 1.5, 0.8),
        ];
        let mut poly = PolyMesh::new();
        let face: Vec<u32> = corners.iter().map(|&p| poly.push_vertex(p)).collect();
        poly.push_face(face);
        let expected = newell_normal(&corners).normalize();

        let mesh = MeshData::flat_from_polys(&poly);
        assert_eq!(mesh.indices.len(), 6);
        for n in triangle_normals(&mesh) {
            assert!(n.dot(expected) > 0.0, "quad folded: triangle faces {n:?}");
        }
    }
}
