//! Voxel data model: multi-scale binary voxels stored in octree chunks.
//!
//! Nothing in this crate knows about meshing or rendering. Meshers consume
//! voxel data through [`tree_view::TreeNode`], so storage can be swapped
//! without touching them.

pub mod chunk;
pub mod octree;
pub mod shapes;
pub mod tree_view;
pub mod voxel;
pub mod world;

pub use chunk::{CHUNK_DEPTH, CHUNK_SIZE, Chunk};
pub use octree::{Node, Octree};
pub use tree_view::{EMPTY_NODE, TreeNode};
pub use voxel::Voxel;
pub use world::World;
