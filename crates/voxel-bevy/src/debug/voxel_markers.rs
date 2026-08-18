//! A sphere at every active voxel's center, scaled to the voxel's size.

use bevy::prelude::*;
use voxel_core::world::World as VoxelWorld;

use crate::chunk_render::VoxelWorldRes;
use crate::config;

pub fn draw(mut gizmos: Gizmos, world: Res<VoxelWorldRes>) {
    for c in world.0.chunk_coords() {
        let origin = VoxelWorld::chunk_origin(c).as_vec3();
        let Some(chunk) = world.0.chunk(c) else {
            continue;
        };
        chunk.tree().for_each_leaf(&mut |min, size, voxel| {
            if voxel.is_active() {
                let size = size as f32;
                gizmos.sphere(
                    origin + min.as_vec3() + Vec3::splat(size / 2.0),
                    size * config::VOXEL_MARKER_RADIUS,
                    config::VOXEL_MARKER_COLOR,
                );
            }
        });
    }
}
