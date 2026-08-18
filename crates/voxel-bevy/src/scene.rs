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
    /// A skeletal tree: thin branches of finest-scale voxels running in
    /// diagonal directions.
    Tree,
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
        Scene::Tree => tree(),
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
        Scene::Tree => CameraStart {
            eye: Vec3::new(100.0, 44.0, 100.0),
            target: Vec3::new(48.0, 26.0, 48.0),
            speed: 20.0,
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

    // The large voxel: 4³, occupying x/y/z 12..16.
    chunk.set_region(IVec3::new(12, 16, 12), 4, Voxel::ACTIVE);

    // The large voxel: 4³, occupying x/y/z 12..16.
    chunk.set_region(IVec3::new(16, 16, 12), 4, Voxel::ACTIVE);

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

/// A skeletal tree, authored one finest-scale voxel at a time.
///
/// Branches are walked as 1-voxel-thick curved paths in freely chosen
/// directions rather than along the axes, so nearly every surface is
/// stair-stepped and the mesher meets diagonal configurations constantly
/// — the ambiguous cases, where material has to stay 6-connected while
/// empty space connects diagonally. Nothing here calls `set_region`, and
/// the paths are thin enough that no aligned 2³ block ever fills, so the
/// octree keeps every voxel at size 1 (asserted in the tests below).
fn tree() -> World {
    let mut world = World::new();
    let mut rng = Rng(0x51ED_BEEF);
    grow(
        &mut world,
        Vec3::new(48.5, 4.0, 48.5),
        Vec3::new(0.35, 1.0, 0.22).normalize(),
        15.0,
        6,
        &mut rng,
    );
    world
}

/// Walks one branch, then splits into two or three children.
fn grow(world: &mut World, from: Vec3, dir: Vec3, length: f32, depth: u32, rng: &mut Rng) {
    // Under half a voxel: fine enough that a branch follows its curve
    // closely rather than cutting across it.
    const STEP: f32 = 0.45;

    // Never step diagonally: material is 6-connected in the mesher, so two
    // voxels touching only along an edge or at a corner mesh as separate
    // pieces, and a branch built that way falls apart into floating stubs.
    let mut pos = from;
    let mut dir = dir;
    let mut last = pos.floor().as_ivec3();
    world.set_voxel(last, Voxel::ACTIVE);
    for _ in 0..(length / STEP).ceil() as i32 {
        // gentle curve, plus a slight pull upward so branches do not droop
        dir = (dir + perpendicular(dir, rng.angle()) * 0.05 + Vec3::Y * 0.015).normalize();
        pos += dir * STEP;
        last = connect(world, last, pos.floor().as_ivec3());
    }

    if depth == 0 {
        return;
    }
    let children = if rng.unit() < 0.35 { 3 } else { 2 };
    for _ in 0..children {
        // Bend well away from the parent: a child that ran along a voxel
        // axis would defeat the point of the scene.
        let bend = rng.range(0.65, 1.15);
        let axis = perpendicular(dir, rng.angle());
        let child = (dir * bend.cos() + axis * bend.sin()).normalize();
        grow(
            world,
            pos,
            child,
            length * rng.range(0.6, 0.78),
            depth - 1,
            rng,
        );
    }
}

/// Sets every voxel from `from` to `to` moving one axis at a time, and
/// returns `to`. Consecutive samples of a branch can differ on two axes
/// at once when the path crosses two voxel planes within one step; going
/// straight there would leave the two voxels touching only along an edge.
fn connect(world: &mut World, from: IVec3, to: IVec3) -> IVec3 {
    let mut p = from;
    for axis in 0..3 {
        while p[axis] != to[axis] {
            p[axis] += (to[axis] - p[axis]).signum();
            world.set_voxel(p, Voxel::ACTIVE);
        }
    }
    p
}

/// A unit vector perpendicular to `dir`, picked by rolling `roll` radians
/// around it.
fn perpendicular(dir: Vec3, roll: f32) -> Vec3 {
    let seed = if dir.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = dir.cross(seed).normalize();
    let v = dir.cross(u).normalize();
    u * roll.cos() + v * roll.sin()
}

/// Deterministic xorshift PRNG: the demo must grow the same tree every
/// run, and voxel-core deliberately carries no random dependency.
struct Rng(u32);

impl Rng {
    fn unit(&mut self) -> f32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        (self.0 >> 8) as f32 / (1u32 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }

    fn angle(&mut self) -> f32 {
        self.range(0.0, std::f32::consts::TAU)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn active_voxels(world: &World) -> Vec<(IVec3, u32)> {
        let mut leaves = Vec::new();
        for c in world.chunk_coords() {
            let origin = World::chunk_origin(c);
            world
                .chunk(c)
                .unwrap()
                .tree()
                .for_each_leaf(&mut |min, size, voxel| {
                    if voxel.is_active() {
                        leaves.push((origin + min, size));
                    }
                });
        }
        leaves
    }

    #[test]
    fn the_tree_is_built_entirely_from_finest_scale_voxels() {
        let voxels = active_voxels(&build_world(Scene::Tree));
        assert!(
            voxels.len() > 1000,
            "expected a substantial tree, got {} voxels",
            voxels.len()
        );
        assert!(
            voxels.iter().all(|&(_, size)| size == 1),
            "every voxel must be at the finest scale, but {} are coarser",
            voxels.iter().filter(|&&(_, size)| size > 1).count()
        );
    }

    #[test]
    fn the_tree_is_full_of_diagonal_adjacencies() {
        let voxels = active_voxels(&build_world(Scene::Tree));
        let solid: std::collections::HashSet<IVec3> = voxels.iter().map(|&(p, _)| p).collect();

        let mut diagonal = 0;
        for &p in &solid {
            for dx in -1i32..=1 {
                for dy in -1i32..=1 {
                    for dz in -1i32..=1 {
                        // 2 or 3 steps away: leaves touching only along an
                        // edge or at a corner, which are the configurations
                        // the patch table has to disambiguate
                        if dx.abs() + dy.abs() + dz.abs() >= 2
                            && solid.contains(&(p + IVec3::new(dx, dy, dz)))
                        {
                            diagonal += 1;
                        }
                    }
                }
            }
        }
        assert!(
            diagonal > solid.len(),
            "expected diagonal neighbours throughout, got {diagonal} across {} voxels",
            solid.len()
        );
    }

    /// A branch that stepped diagonally would mesh as separate floating
    /// pieces, since the mesher keeps material 6-connected. The tree has
    /// to be a single face-connected component.
    #[test]
    fn the_tree_is_one_connected_piece() {
        let voxels = active_voxels(&build_world(Scene::Tree));
        let solid: std::collections::HashSet<IVec3> = voxels.iter().map(|&(p, _)| p).collect();

        let start = *solid.iter().next().expect("the tree is not empty");
        let mut seen = std::collections::HashSet::from([start]);
        let mut queue = vec![start];
        while let Some(p) = queue.pop() {
            for axis in 0..3 {
                for step in [-1, 1] {
                    let mut neighbour = p;
                    neighbour[axis] += step;
                    if solid.contains(&neighbour) && seen.insert(neighbour) {
                        queue.push(neighbour);
                    }
                }
            }
        }
        assert_eq!(
            seen.len(),
            solid.len(),
            "the tree broke into pieces: {} of {} voxels are cut off from the trunk",
            solid.len() - seen.len(),
            solid.len()
        );
    }
}
