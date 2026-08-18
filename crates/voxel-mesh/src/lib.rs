//! Meshing: turns voxel octrees into surface geometry. Engine-agnostic —
//! rendering lives elsewhere.

pub mod analysis;
pub mod cube;
pub mod dmc;
pub mod mesh_data;
pub mod mesher;
pub mod patches;
pub mod poly_mesh;

pub use mesh_data::MeshData;
pub use mesher::{DualMarchingCubes, Mesher};
pub use poly_mesh::PolyMesh;
