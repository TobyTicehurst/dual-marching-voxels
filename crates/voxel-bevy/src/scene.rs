//! The demo world. Pure voxel-core — nothing here knows about Bevy, so it
//! doubles as an example of authoring multi-scale content.

use glam::{IVec3, Vec3};
use voxel_core::shapes;
use voxel_core::voxel::Voxel;
use voxel_core::world::World;

/// 4×2×4 chunks (128×64×128 voxels): rolling terrain (fine surface,
/// automatically coarse underground) plus floating showcase objects —
/// a fine sphere, a torus, a deliberately 4×-quantized sphere, and a
/// half-fine/half-coarse sphere to show scale transitions up close.
/// Everything crosses chunk borders somewhere.
pub fn build_demo_world() -> World {
    let terrain = shapes::sine_terrain(22.0, 5.0, 11.0);
    let sphere = shapes::sphere(Vec3::new(30.0, 42.0, 34.0), 9.0);
    let torus = shapes::torus(Vec3::new(88.0, 44.0, 88.0), 13.0, 5.0);
    let blocky_sphere = shapes::quantized(shapes::sphere(Vec3::new(96.0, 44.0, 30.0), 10.0), 4);
    let dual_fine = shapes::sphere(Vec3::new(32.0, 46.0, 96.0), 10.0);
    let dual_coarse = shapes::quantized(shapes::sphere(Vec3::new(32.0, 46.0, 96.0), 10.0), 4);

    let f = move |p: IVec3| {
        let dual = if p.x < 32 {
            dual_coarse(p)
        } else {
            dual_fine(p)
        };
        Voxel::new(
            terrain(p).is_active()
                || sphere(p).is_active()
                || torus(p).is_active()
                || blocky_sphere(p).is_active()
                || dual.is_active(),
        )
    };

    let mut world = World::new();
    world.build_chunks(IVec3::new(0, 0, 0), IVec3::new(3, 1, 3), &f);
    world
}

/// Where the fly camera starts: looking across the whole scene.
pub fn camera_start() -> (Vec3, Vec3) {
    (Vec3::new(150.0, 80.0, 150.0), Vec3::new(64.0, 28.0, 64.0))
}
