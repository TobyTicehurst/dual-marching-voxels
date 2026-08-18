//! The voxel value itself.

/// A single voxel's data. Deliberately a struct rather than a bare `bool`:
/// richer per-voxel data (materials, densities) can be added later without
/// changing every signature that passes voxels around.
///
/// A voxel does not know its own size — scale lives in the octree structure
/// (a leaf's depth), not in the value stored at the leaf.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Voxel {
    pub active: bool,
}

impl Voxel {
    pub const ACTIVE: Voxel = Voxel { active: true };
    pub const INACTIVE: Voxel = Voxel { active: false };

    pub const fn new(active: bool) -> Self {
        Voxel { active }
    }

    pub const fn is_active(self) -> bool {
        self.active
    }
}
