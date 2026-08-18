//! An unbounded voxel world: a sparse map from chunk coordinate to chunk.

use std::collections::HashMap;

use glam::IVec3;

use crate::chunk::{CHUNK_SIZE, Chunk};
use crate::voxel::Voxel;

/// Component-wise Euclidean division/remainder, so negative world
/// coordinates map to the correct chunk instead of rounding toward zero.
fn div_euclid(p: IVec3, d: i32) -> IVec3 {
    IVec3::new(p.x.div_euclid(d), p.y.div_euclid(d), p.z.div_euclid(d))
}

fn rem_euclid(p: IVec3, d: i32) -> IVec3 {
    IVec3::new(p.x.rem_euclid(d), p.y.rem_euclid(d), p.z.rem_euclid(d))
}

#[derive(Debug, Default)]
pub struct World {
    chunks: HashMap<IVec3, Chunk>,
}

impl World {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn chunk_coord_of(world_pos: IVec3) -> IVec3 {
        div_euclid(world_pos, CHUNK_SIZE as i32)
    }

    pub fn local_of(world_pos: IVec3) -> IVec3 {
        rem_euclid(world_pos, CHUNK_SIZE as i32)
    }

    /// The world position of a chunk's local `(0, 0, 0)`.
    pub fn chunk_origin(chunk_coord: IVec3) -> IVec3 {
        chunk_coord * CHUNK_SIZE as i32
    }

    pub fn chunk(&self, chunk_coord: IVec3) -> Option<&Chunk> {
        self.chunks.get(&chunk_coord)
    }

    /// The chunk at `chunk_coord`, created empty if absent.
    pub fn chunk_mut(&mut self, chunk_coord: IVec3) -> &mut Chunk {
        self.chunks.entry(chunk_coord).or_default()
    }

    pub fn insert_chunk(&mut self, chunk_coord: IVec3, chunk: Chunk) {
        self.chunks.insert(chunk_coord, chunk);
    }

    /// Reads the finest-scale voxel at a world position. Positions in
    /// chunks that don't exist read as inactive.
    pub fn get_voxel(&self, world_pos: IVec3) -> Voxel {
        self.chunk(Self::chunk_coord_of(world_pos))
            .map_or(Voxel::INACTIVE, |c| c.get(Self::local_of(world_pos)))
    }

    pub fn set_voxel(&mut self, world_pos: IVec3, voxel: Voxel) {
        self.chunk_mut(Self::chunk_coord_of(world_pos))
            .set(Self::local_of(world_pos), voxel);
    }

    /// All existing chunk coordinates, sorted for deterministic iteration.
    pub fn chunk_coords(&self) -> Vec<IVec3> {
        let mut coords: Vec<IVec3> = self.chunks.keys().copied().collect();
        coords.sort_by_key(|c| (c.z, c.y, c.x));
        coords
    }

    /// Builds (or replaces) one chunk by sampling `f` at **world**
    /// positions.
    pub fn build_chunk(&mut self, chunk_coord: IVec3, f: &impl Fn(IVec3) -> Voxel) {
        let origin = Self::chunk_origin(chunk_coord);
        self.insert_chunk(chunk_coord, Chunk::from_fn(&mut |local| f(origin + local)));
    }

    /// Builds every chunk in the inclusive chunk-coordinate box
    /// `[min, max]` by sampling `f` at world positions.
    pub fn build_chunks(&mut self, min: IVec3, max: IVec3, f: &impl Fn(IVec3) -> Voxel) {
        for z in min.z..=max.z {
            for y in min.y..=max.y {
                for x in min.x..=max.x {
                    self.build_chunk(IVec3::new(x, y, z), f);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordinate_math_handles_negative_positions() {
        assert_eq!(
            World::chunk_coord_of(IVec3::new(0, 31, 32)),
            IVec3::new(0, 0, 1)
        );
        assert_eq!(
            World::chunk_coord_of(IVec3::new(-1, -32, -33)),
            IVec3::new(-1, -1, -2)
        );
        assert_eq!(
            World::local_of(IVec3::new(-1, -32, -33)),
            IVec3::new(31, 0, 31)
        );
        // origin + local must round-trip to the original position
        let p = IVec3::new(-100, 7, 45);
        assert_eq!(
            World::chunk_origin(World::chunk_coord_of(p)) + World::local_of(p),
            p
        );
    }

    #[test]
    fn set_then_get_round_trips_across_chunk_boundaries() {
        let mut world = World::new();
        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(31, 31, 31),
            IVec3::new(32, 0, 0),
            IVec3::new(-1, 5, 70),
        ];
        for p in positions {
            world.set_voxel(p, Voxel::ACTIVE);
        }
        for p in positions {
            assert_eq!(world.get_voxel(p), Voxel::ACTIVE, "at {p}");
        }
        assert_eq!(world.get_voxel(IVec3::new(1, 0, 0)), Voxel::INACTIVE);
        // untouched chunks read inactive and are not created by reads
        assert_eq!(world.get_voxel(IVec3::new(1000, 0, 0)), Voxel::INACTIVE);
        assert!(world.chunk(IVec3::new(31, 0, 0)).is_none());
    }

    #[test]
    fn build_chunk_samples_world_coordinates() {
        let mut world = World::new();
        // active iff world x == 33 — lands in chunk (1, 0, 0) at local x 1
        world.build_chunk(IVec3::new(1, 0, 0), &|p| Voxel::new(p.x == 33));
        assert_eq!(world.get_voxel(IVec3::new(33, 4, 9)), Voxel::ACTIVE);
        assert_eq!(world.get_voxel(IVec3::new(34, 4, 9)), Voxel::INACTIVE);
    }
}
