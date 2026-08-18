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
    /// diagonal directions, kept connected by filling every diagonal step.
    Tree,
    /// The same tree with those steps left unfilled, so branches break
    /// into pieces touching along edges — which the mesher now joins, so
    /// this shows the join, and the one corner contact it cannot see.
    DiagonalTree,
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
        Scene::Tree => tree(true),
        Scene::DiagonalTree => tree(false),
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
        Scene::Tree | Scene::DiagonalTree => CameraStart {
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
/// — the ambiguous cases, where the patch table decides whether material
/// or empty space is the one that stays connected. Nothing here calls `set_region`, and
/// the paths are thin enough that no aligned 2³ block ever fills, so the
/// octree keeps every voxel at size 1 (asserted in the tests below).
///
/// `fill_diagonal_steps` decides what happens when the path crosses two
/// voxel planes within one step, leaving two voxels that touch only along
/// an edge. Filling the corner in makes the tree face-connected, which no
/// meshing rule can argue with; leaving it produces
/// [`Scene::DiagonalTree`], whose voxels are joined only along those
/// edges — so it shows what the patch table does with edge contacts, and
/// is the scene to watch while changing that. Both scenes grow the *same*
/// tree, so they differ only in those joints.
fn tree(fill_diagonal_steps: bool) -> World {
    let mut world = World::new();
    let mut rng = Rng(0x51ED_BEEF);
    grow(
        &mut world,
        Vec3::new(48.5, 4.0, 48.5),
        Vec3::new(0.35, 1.0, 0.22).normalize(),
        15.0,
        6,
        &mut rng,
        fill_diagonal_steps,
    );
    world
}

/// Walks one branch, then splits into two or three children.
fn grow(
    world: &mut World,
    from: Vec3,
    dir: Vec3,
    length: f32,
    depth: u32,
    rng: &mut Rng,
    fill_diagonal_steps: bool,
) {
    // Under half a voxel: fine enough that a branch follows its curve
    // closely rather than cutting across it.
    const STEP: f32 = 0.45;

    let mut pos = from;
    let mut dir = dir;
    let mut last = pos.floor().as_ivec3();
    world.set_voxel(last, Voxel::ACTIVE);
    for _ in 0..(length / STEP).ceil() as i32 {
        // gentle curve, plus a slight pull upward so branches do not droop
        dir = (dir + perpendicular(dir, rng.angle()) * 0.05 + Vec3::Y * 0.015).normalize();
        pos += dir * STEP;
        let next = pos.floor().as_ivec3();
        last = if fill_diagonal_steps {
            connect(world, last, next)
        } else {
            world.set_voxel(next, Voxel::ACTIVE);
            next
        };
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
            fill_diagonal_steps,
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

    fn solid_voxels(scene: Scene) -> std::collections::HashSet<IVec3> {
        active_voxels(&build_world(scene))
            .iter()
            .map(|&(p, _)| p)
            .collect()
    }

    /// Number of connected components, where two voxels count as joined
    /// when they differ on at most `axes` axes: 1 is face contact only, 2
    /// also counts edge contacts, 3 also counts corner contacts. The
    /// mesher currently joins material through face *and* edge contacts,
    /// so 2 is the notion that matches what gets meshed.
    fn connected_pieces(solid: &std::collections::HashSet<IVec3>, axes: i32) -> usize {
        let mut unvisited = solid.clone();
        let mut pieces = 0;
        while let Some(&start) = unvisited.iter().next() {
            pieces += 1;
            let mut queue = vec![start];
            unvisited.remove(&start);
            while let Some(p) = queue.pop() {
                for dx in -1i32..=1 {
                    for dy in -1i32..=1 {
                        for dz in -1i32..=1 {
                            let d = IVec3::new(dx, dy, dz);
                            let steps = dx.abs() + dy.abs() + dz.abs();
                            if (1..=axes).contains(&steps) && unvisited.remove(&(p + d)) {
                                queue.push(p + d);
                            }
                        }
                    }
                }
            }
        }
        pieces
    }

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

    /// The connected variant fills every diagonal step, so it stands on
    /// its own regardless of how the mesher treats edge contacts: it must
    /// be a single face-connected component.
    #[test]
    fn the_tree_is_one_connected_piece() {
        let solid = solid_voxels(Scene::Tree);
        assert_eq!(
            connected_pieces(&solid, 1),
            1,
            "the tree broke apart: filling the diagonal steps should leave \
             exactly one face-connected piece"
        );
    }

    /// `DiagonalTree` is the same tree with its diagonal steps left
    /// unfilled, so its voxels come apart into pieces that touch only
    /// along edges. The mesher joins material through edge contacts, so
    /// the *rendered* tree is whole again — what this scene now shows is
    /// the join itself, and it is the scene to watch if that rule changes
    /// again (a corner-only contact would still leave a visible gap).
    #[test]
    fn the_diagonal_tree_is_broken_in_voxels_but_joined_by_the_mesher() {
        let diagonal = solid_voxels(Scene::DiagonalTree);
        let connected = solid_voxels(Scene::Tree);

        assert!(
            active_voxels(&build_world(Scene::DiagonalTree))
                .iter()
                .all(|&(_, size)| size == 1),
            "the diagonal tree must be finest-scale too"
        );
        assert!(
            connected_pieces(&diagonal, 1) > 1,
            "the diagonal tree is supposed to come apart, but it is one piece"
        );
        // The mesher joins material through edge contacts as well as face
        // ones, so nearly every one of those breaks is closed in the
        // rendered mesh — but one joint in this tree is a corner-only
        // contact, which no face rule can see, and it stays a visible gap.
        assert_eq!(
            connected_pieces(&diagonal, 2),
            2,
            "expected exactly one corner-only joint left unjoined"
        );
        assert_eq!(
            connected_pieces(&diagonal, 3),
            1,
            "and that last joint should be a corner contact, not a real gap"
        );

        // Same tree, minus the corner voxels that filling adds: the two
        // scenes must differ only at those joints.
        assert!(
            diagonal.is_subset(&connected),
            "the two trees should share every voxel the diagonal one has"
        );
        assert!(diagonal.len() < connected.len());
    }

    /// The scenes double as the mesher's stress test: the two trees are
    /// almost nothing but diagonal contacts, and the landscape mixes
    /// scales everywhere. All of them must come out closed and
    /// 2-manifold — every edge contact is a place the surface used to
    /// pinch before segment vertices split those junctions open.
    #[test]
    fn every_scene_meshes_to_a_closed_manifold_surface() {
        use voxel_mesh::analysis::{edge_stats, weld};
        use voxel_mesh::dmc::mesh_world;

        for scene in [
            Scene::TestBed,
            Scene::Tree,
            Scene::DiagonalTree,
            Scene::Landscape,
        ] {
            let mesh = mesh_world(&build_world(scene));
            let stats = edge_stats(&weld(&mesh));
            assert_eq!(stats.boundary, 0, "{scene:?}: cracks: {stats:?}");
            assert_eq!(stats.non_manifold, 0, "{scene:?}: pinched edges: {stats:?}");
        }
    }
}
