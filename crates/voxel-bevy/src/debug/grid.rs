//! Chunk bounds and (optionally) the full finest-scale voxel grid.

use bevy::prelude::*;
use voxel_core::chunk::CHUNK_SIZE;
use voxel_core::world::World as VoxelWorld;

use crate::chunk_render::VoxelWorldRes;
use crate::config;

pub fn draw(mut gizmos: Gizmos, world: Res<VoxelWorldRes>) {
    let side = CHUNK_SIZE as f32;
    for c in world.0.chunk_coords() {
        let origin = VoxelWorld::chunk_origin(c).as_vec3();
        if config::SHOW_CHUNK_BOUNDS {
            gizmos.cube(
                Transform::from_translation(origin + Vec3::splat(side / 2.0))
                    .with_scale(Vec3::splat(side)),
                config::CHUNK_BOUNDS_COLOR,
            );
        }
        if config::SHOW_VOXEL_GRID {
            let n = CHUNK_SIZE as i32;
            for a in 0..=n {
                for b in 0..=n {
                    let (a, b) = (a as f32, b as f32);
                    gizmos.line(
                        origin + Vec3::new(a, b, 0.0),
                        origin + Vec3::new(a, b, side),
                        config::VOXEL_GRID_COLOR,
                    );
                    gizmos.line(
                        origin + Vec3::new(a, 0.0, b),
                        origin + Vec3::new(a, side, b),
                        config::VOXEL_GRID_COLOR,
                    );
                    gizmos.line(
                        origin + Vec3::new(0.0, a, b),
                        origin + Vec3::new(side, a, b),
                        config::VOXEL_GRID_COLOR,
                    );
                }
            }
        }
    }
}
