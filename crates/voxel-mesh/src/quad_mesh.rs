//! The mesher's logical output: shared/indexed dual vertices and quads.
//! Topology tests and analysis run on this; rendering expands it to flat
//! triangles ([`crate::mesh_data::MeshData`]) separately.

use glam::Vec3;

/// Positions are in world voxel space (1.0 = one finest-scale voxel).
/// Quads are wound counter-clockwise seen from outside the surface.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct QuadMesh {
    pub positions: Vec<Vec3>,
    pub quads: Vec<[u32; 4]>,
}

impl QuadMesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_vertex(&mut self, p: Vec3) -> u32 {
        let i = u32::try_from(self.positions.len()).expect("QuadMesh overflowed u32 indices");
        self.positions.push(p);
        i
    }

    pub fn push_quad(&mut self, quad: [u32; 4]) {
        self.quads.push(quad);
    }

    /// Appends another mesh (e.g. a neighbouring chunk's), remapping its
    /// indices. Duplicate vertices are kept; weld afterwards if needed.
    pub fn append(&mut self, other: &QuadMesh) {
        let base = u32::try_from(self.positions.len()).expect("QuadMesh overflowed u32 indices");
        self.positions.extend_from_slice(&other.positions);
        self.quads
            .extend(other.quads.iter().map(|q| q.map(|i| i + base)));
    }

    pub fn is_empty(&self) -> bool {
        self.quads.is_empty()
    }
}
