//! Meshing: turns voxel octrees into surface geometry. Engine-agnostic —
//! rendering lives elsewhere.

pub mod analysis;
pub mod cube;
pub mod mesh_data;
pub mod patches;
pub mod quad_mesh;

pub use mesh_data::MeshData;
pub use quad_mesh::QuadMesh;
