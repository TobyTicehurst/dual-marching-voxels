//! Demo scenes. Pure voxel-core — nothing here knows about Bevy, so this
//! doubles as an example of authoring multi-scale content.
//!
//! Which scene the app builds is [`crate::config::SCENE`], a compile-time
//! choice like every other knob in `config`.

use glam::{IVec3, Vec3};
use voxel_core::shapes;
use voxel_core::voxel::Voxel;
use voxel_core::world::World;

// Whichever scene `config::SCENE` does not name is never constructed —
// that is what a compile-time selector means, not an unused variant.
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Scene {
    /// Small test bed for trying one thing at a time — currently one
    /// large voxel beside two half-scale ones. Edit [`test_bed`] freely.
    TestBed,
    /// The full landscape demo: terrain plus showcase objects across
    /// 4×2×4 chunks.
    Landscape,
}

/// Where the fly camera starts and how fast it moves, per scene — a small
/// scene needs a close viewpoint and a slow camera.
pub struct CameraStart {
    pub eye: Vec3,
    pub target: Vec3,
    pub speed: f32,
}

pub fn build_world(scene: Scene) -> World {
    match scene {
        Scene::TestBed => test_bed(),
        Scene::Landscape => landscape(),
    }
}

pub fn camera_start(scene: Scene) -> CameraStart {
    match scene {
        // framed on the voxel cluster below, which spans x 12..18,
        // y 12..16, z 12..16
        Scene::TestBed => CameraStart {
            eye: Vec3::new(23.0, 19.0, 23.0),
            target: Vec3::new(15.0, 14.0, 14.0),
            speed: 6.0,
        },
        Scene::Landscape => CameraStart {
            eye: Vec3::new(150.0, 80.0, 150.0),
            target: Vec3::new(64.0, 28.0, 64.0),
            speed: 40.0,
        },
    }
}

/// **The scratch scene** — small, hand-placed voxels for testing one
/// behaviour at a time. Everything sits in a single chunk near its middle
/// so nothing touches a chunk seam.
///
/// Right now: one 4³ voxel with two 2³ voxels on its +x face. The pair
/// covers half that face (the `z 12..14` strip), so the transition shows
/// both halves of the interesting case at once — where the coarse face is
/// met by finer voxels, and where it stays exposed. The dual cells along
/// the coarse voxel's face boundary are the "hanging" ones that turn the
/// dual face into an n-gon rather than a quad, which is what keeps the
/// mesh crack-free. `debug::dual_cells` draws them.
fn test_bed() -> World {
    let mut world = World::new();
    let chunk = world.chunk_mut(IVec3::ZERO);

    // The large voxel: 4³, occupying x/y/z 12..16.
    chunk.set_region(IVec3::new(12, 12, 12), 4, Voxel::ACTIVE);

    // Two half-scale (2³) voxels stacked in y on its +x face.
    chunk.set_region(IVec3::new(16, 12, 12), 2, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 14, 12), 2, Voxel::ACTIVE);

    world
}

/// 4×2×4 chunks (128×64×128 voxels): rolling terrain (fine surface,
/// automatically coarse underground) plus floating showcase objects —
/// a fine sphere, a torus, a deliberately 4×-quantized sphere, and a
/// half-fine/half-coarse sphere to show scale transitions up close.
/// Everything crosses chunk borders somewhere.
fn landscape() -> World {
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
