//! The interface future meshing algorithms (Surface Nets, custom) plug
//! into, so callers — including the renderer — never name a concrete
//! algorithm.

use voxel_core::world::World;

use crate::poly_mesh::PolyMesh;

pub trait Mesher {
    /// Meshes the whole world (chunks and seams) into one polygon mesh in
    /// world voxel coordinates.
    fn mesh_world(&self, world: &World) -> PolyMesh;
}

/// Nielson-style Dual Marching Cubes without normals — see [`crate::dmc`].
pub struct DualMarchingCubes;

impl Mesher for DualMarchingCubes {
    fn mesh_world(&self, world: &World) -> PolyMesh {
        crate::dmc::mesh_world(world)
    }
}
