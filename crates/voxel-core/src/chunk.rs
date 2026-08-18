//! A fixed-size octree of voxels — the unit of storage and (in voxel-mesh)
//! the unit of meshing.

use glam::IVec3;

use crate::octree::Octree;
use crate::voxel::Voxel;

pub const CHUNK_DEPTH: u32 = 5;
/// Chunks are 32³ finest-scale voxels.
pub const CHUNK_SIZE: u32 = 1 << CHUNK_DEPTH;

/// One 32³ chunk. Coordinates on this type are chunk-local, `[0, 32)³`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chunk {
    tree: Octree,
}

impl Chunk {
    pub fn empty() -> Self {
        Chunk {
            tree: Octree::uniform(CHUNK_DEPTH, Voxel::INACTIVE),
        }
    }

    /// Builds by sampling `f` at every finest-scale local position; uniform
    /// power-of-two blocks merge into single coarser voxels.
    pub fn from_fn(f: &mut impl FnMut(IVec3) -> Voxel) -> Self {
        Chunk {
            tree: Octree::from_fn(CHUNK_DEPTH, f),
        }
    }

    pub fn tree(&self) -> &Octree {
        &self.tree
    }

    pub fn get(&self, local: IVec3) -> Voxel {
        self.tree.get(local)
    }

    pub fn set(&mut self, local: IVec3, voxel: Voxel) {
        self.tree.set(local, voxel);
    }

    /// Sets one voxel of power-of-two `size` at aligned local min `min`.
    pub fn set_region(&mut self, min: IVec3, size: u32, voxel: Voxel) {
        self.tree.set_region(min, size, voxel);
    }

    pub fn is_empty(&self) -> bool {
        self.tree.as_uniform() == Some(Voxel::INACTIVE)
    }
}

impl Default for Chunk {
    fn default() -> Self {
        Self::empty()
    }
}
