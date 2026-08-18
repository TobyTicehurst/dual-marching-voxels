//! Analytic shape samplers for demos and tests. Each returns a
//! `Fn(IVec3) -> Voxel` over **world** voxel coordinates, ready for
//! [`crate::world::World::build_chunks`] / [`crate::octree::Octree::from_fn`].
//! No noise/PRNG dependencies — everything is deterministic and analytic.

use glam::{IVec3, Vec3};

use crate::voxel::Voxel;

/// Where a finest-scale voxel's center sits in continuous space. All shape
/// membership is decided at voxel centers.
pub fn voxel_center(p: IVec3) -> Vec3 {
    p.as_vec3() + 0.5
}

pub fn sphere(center: Vec3, radius: f32) -> impl Fn(IVec3) -> Voxel {
    move |p| Voxel::new(voxel_center(p).distance_squared(center) <= radius * radius)
}

/// Torus around `center` with its axis along +y; `major` is the ring
/// radius, `minor` the tube radius.
pub fn torus(center: Vec3, major: f32, minor: f32) -> impl Fn(IVec3) -> Voxel {
    move |p| {
        let d = voxel_center(p) - center;
        let ring = Vec3::new(d.x, 0.0, d.z);
        let q_x = ring.length() - major;
        Voxel::new(q_x * q_x + d.y * d.y <= minor * minor)
    }
}

/// Rolling terrain: active below a sum-of-sines heightfield. Uniform
/// underground regions merge into large voxels on their own when built
/// into an octree.
pub fn sine_terrain(base_height: f32, amplitude: f32, wavelength: f32) -> impl Fn(IVec3) -> Voxel {
    move |p| {
        let c = voxel_center(p);
        let h = base_height
            + amplitude * (c.x / wavelength).sin() * (c.z / wavelength).cos()
            + 0.35
                * amplitude
                * (c.x / (wavelength * 0.31)).cos()
                * (c.z / (wavelength * 0.43)).sin();
        Voxel::new(c.y <= h)
    }
}

/// Coarsens `f` to `block`-sized voxels (a power of two): every aligned
/// `block`³ group samples `f` once at its representative center voxel, so
/// the group is uniform and the octree stores it as one genuinely bigger
/// voxel. This is how demo scenes author *deliberately* coarse regions.
pub fn quantized(f: impl Fn(IVec3) -> Voxel, block: i32) -> impl Fn(IVec3) -> Voxel {
    debug_assert!(block >= 1 && (block as u32).is_power_of_two());
    move |p| {
        let min = IVec3::new(
            p.x.div_euclid(block) * block,
            p.y.div_euclid(block) * block,
            p.z.div_euclid(block) * block,
        );
        f(min + IVec3::splat(block / 2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chunk::Chunk;
    use crate::world::World;

    #[test]
    fn sphere_volume_is_roughly_right() {
        let f = sphere(Vec3::splat(16.0), 10.0);
        let mut world = World::new();
        world.build_chunks(IVec3::ZERO, IVec3::ZERO, &f);
        let mut active = 0u32;
        for x in 0..32 {
            for y in 0..32 {
                for z in 0..32 {
                    if world.get_voxel(IVec3::new(x, y, z)).is_active() {
                        active += 1;
                    }
                }
            }
        }
        let expected = 4.0 / 3.0 * std::f32::consts::PI * 1000.0;
        let ratio = active as f32 / expected;
        assert!(
            (0.9..1.1).contains(&ratio),
            "sphere volume off: {active} vs {expected}"
        );
    }

    #[test]
    fn quantized_sphere_produces_coarse_leaves() {
        let fine = Chunk::from_fn(&mut sphere(Vec3::splat(16.0), 12.0));
        let coarse = Chunk::from_fn(&mut quantized(sphere(Vec3::splat(16.0), 12.0), 4));
        assert!(
            coarse.tree().leaf_count() < fine.tree().leaf_count() / 4,
            "quantizing should shrink the tree a lot: {} vs {}",
            coarse.tree().leaf_count(),
            fine.tree().leaf_count()
        );
        // and the coarse tree must actually contain size-4 voxels
        let mut has_coarse_active = false;
        coarse.tree().for_each_leaf(&mut |_, size, v| {
            if size >= 4 && v.is_active() {
                has_coarse_active = true;
            }
        });
        assert!(has_coarse_active);
    }

    #[test]
    fn terrain_underground_merges_coarse() {
        let chunk = Chunk::from_fn(&mut sine_terrain(24.0, 4.0, 7.0));
        let mut max_active_size = 0;
        chunk.tree().for_each_leaf(&mut |_, size, v| {
            if v.is_active() {
                max_active_size = max_active_size.max(size);
            }
        });
        assert!(
            max_active_size >= 8,
            "deep underground should merge into ≥8-sized voxels, got {max_active_size}"
        );
    }
}
