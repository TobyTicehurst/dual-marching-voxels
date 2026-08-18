//! The mesher's logical output: shared/indexed dual vertices and polygon
//! faces. Topology tests and analysis run on this; rendering expands it to
//! flat triangles ([`crate::mesh_data::MeshData`]) separately.

use glam::Vec3;

/// A quad-dominant polygon mesh. Faces dual to a crossed adjacency between
/// equal-scale voxels are quads; at scale transitions a face gains a vertex
/// for every hanging dual cell on its boundary, which is exactly what keeps
/// the mesh watertight there. Positions are in world voxel space (1.0 = one
/// finest-scale voxel). Faces are wound counter-clockwise seen from outside
/// the surface.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PolyMesh {
    pub positions: Vec<Vec3>,
    pub faces: Vec<Vec<u32>>,
}

impl PolyMesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_vertex(&mut self, p: Vec3) -> u32 {
        let i = u32::try_from(self.positions.len()).expect("PolyMesh overflowed u32 indices");
        self.positions.push(p);
        i
    }

    pub fn push_face(&mut self, face: Vec<u32>) {
        debug_assert!(face.len() >= 3, "faces need at least 3 vertices");
        self.faces.push(face);
    }

    /// Appends another mesh (e.g. a neighbouring chunk's), remapping its
    /// indices. Duplicate vertices are kept; weld afterwards if needed.
    pub fn append(&mut self, other: &PolyMesh) {
        let base = u32::try_from(self.positions.len()).expect("PolyMesh overflowed u32 indices");
        self.positions.extend_from_slice(&other.positions);
        self.faces
            .extend(other.faces.iter().map(|f| f.iter().map(|i| i + base).collect()));
    }

    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    /// The edges of face `f` as index pairs, in cycle order.
    pub fn face_edges(face: &[u32]) -> impl Iterator<Item = (u32, u32)> + '_ {
        (0..face.len()).map(|i| (face[i], face[(i + 1) % face.len()]))
    }
}
