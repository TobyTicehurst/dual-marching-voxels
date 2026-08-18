//! A wireframe box around every active voxel at its own scale — the
//! multi-scale octree structure made visible.

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
                gizmos.cube(
                    Transform::from_translation(origin + min.as_vec3() + Vec3::splat(size / 2.0))
                        .with_scale(Vec3::splat(size)),
                    config::ACTIVE_VOXEL_BOX_COLOR,
                );
            }
        });
    }
}
