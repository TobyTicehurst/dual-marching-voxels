//! End-to-end meshing tests: uniform resolution (milestone 3), mixed
//! scales (milestone 4) and chunk seams (milestone 5).

use glam::{IVec3, Vec3};
use voxel_core::octree::Octree;
use voxel_core::shapes;
use voxel_core::voxel::Voxel;
use voxel_core::world::World;
use voxel_mesh::analysis::{edge_stats, euler_characteristic, signed_volume, weld};
use voxel_mesh::dmc::{Inspect, inspect_world, mesh_octree, mesh_world};
use voxel_mesh::poly_mesh::PolyMesh;

fn world_from_fn(min: IVec3, max: IVec3, f: impl Fn(IVec3) -> Voxel) -> World {
    let mut world = World::new();
    world.build_chunks(min, max, &f);
    world
}

fn assert_closed(mesh: &PolyMesh, expected_euler: i64, what: &str) {
    assert!(!mesh.is_empty(), "{what}: empty mesh");
    let welded = weld(mesh);
    let stats = edge_stats(&welded);
    assert_eq!(
        stats.boundary, 0,
        "{what}: boundary edges (cracks): {stats:?}"
    );
    assert_eq!(
        stats.non_manifold, 0,
        "{what}: non-manifold edges: {stats:?}"
    );
    assert_eq!(
        euler_characteristic(&welded),
        expected_euler,
        "{what}: wrong Euler characteristic"
    );
}

// ---------------------------------------------------------------- uniform

#[test]
fn lone_voxel_is_a_small_closed_cube() {
    let mut world = World::new();
    world.set_voxel(IVec3::new(5, 6, 7), Voxel::ACTIVE);
    let mesh = mesh_world(&world);

    assert_eq!(mesh.faces.len(), 6, "one voxel → one face per side");
    assert_eq!(weld(&mesh).positions.len(), 8);
    assert_closed(&mesh, 2, "lone voxel");
    assert!(signed_volume(&mesh) > 0.0, "winding must be outward");
}

#[test]
fn sphere_is_watertight_with_correct_volume_and_winding() {
    let world = world_from_fn(
        IVec3::ZERO,
        IVec3::ZERO,
        shapes::sphere(Vec3::splat(16.0), 10.0),
    );
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "sphere");

    let vol = signed_volume(&mesh);
    let expected = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
    let ratio = vol / expected;
    assert!(
        (0.85..1.1).contains(&ratio),
        "sphere volume {vol} vs expected {expected}"
    );
}

#[test]
fn torus_has_euler_characteristic_zero() {
    let world = world_from_fn(
        IVec3::ZERO,
        IVec3::ZERO,
        shapes::torus(Vec3::splat(16.0), 9.0, 4.0),
    );
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 0, "torus");
    assert!(signed_volume(&mesh) > 0.0);
}

#[test]
fn slab_closes_against_the_world_boundary() {
    // Fills the full chunk footprint up to y = 12: every chunk face it
    // touches is against absent (empty) chunks, which must cap it.
    let world = world_from_fn(IVec3::ZERO, IVec3::ZERO, |p: IVec3| Voxel::new(p.y <= 12));
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "slab");
    // 32 × 32 × 13 = 13312 voxels of material. Dual smoothing rounds the
    // rim and pulls the walls in noticeably, but the bulk must survive.
    let vol = signed_volume(&mesh);
    assert!((8500.0..13800.0).contains(&vol), "slab volume {vol}");
}

#[test]
fn face_diagonal_voxels_stay_two_separate_closed_surfaces() {
    // Two voxels touching only along a diagonal: Surface Nets would pinch
    // them together at a shared vertex; DMC's per-patch vertices must keep
    // two disjoint closed cubes (Euler 2 + 2 = 4, no non-manifold edges).
    let mut world = World::new();
    world.set_voxel(IVec3::new(4, 4, 4), Voxel::ACTIVE);
    world.set_voxel(IVec3::new(5, 5, 4), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_eq!(mesh.faces.len(), 12);
    assert_closed(&mesh, 4, "face-diagonal pair");
}

#[test]
fn body_diagonal_voxels_stay_separate_too() {
    let mut world = World::new();
    world.set_voxel(IVec3::new(4, 4, 4), Voxel::ACTIVE);
    world.set_voxel(IVec3::new(5, 5, 5), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 4, "body-diagonal pair");
}

#[test]
fn checkerboard_cluster_keeps_all_four_components_apart() {
    // Four mutually diagonal voxels of a 2×2×2 block — the hardest
    // ambiguous configuration (checkerboard case) at every shared cell.
    let mut world = World::new();
    for o in [0b000usize, 0b011, 0b101, 0b110] {
        let p = IVec3::new((o & 1) as i32, ((o >> 1) & 1) as i32, ((o >> 2) & 1) as i32);
        world.set_voxel(IVec3::splat(8) + p, Voxel::ACTIVE);
    }
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 8, "checkerboard");
}

// ------------------------------------------------------------ mixed scale

#[test]
fn one_coarse_voxel_is_a_closed_cube() {
    let mut world = World::new();
    world
        .chunk_mut(IVec3::ZERO)
        .set_region(IVec3::splat(8), 4, Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_eq!(
        mesh.faces.len(),
        6,
        "one coarse voxel still meshes as one cube"
    );
    assert_closed(&mesh, 2, "4³ voxel");
    // Centroid placement rounds an isolated cube hard (that's inherent to
    // the Surface-Nets-style simplification), but meshing must be exactly
    // scale-similar: the same configuration at 4× the scale — a 4³ voxel
    // in a side-32 tree vs a 1³ voxel in a side-8 tree — has 4³ = 64×
    // the volume.
    let mut coarse_tree = Octree::uniform(5, Voxel::INACTIVE);
    coarse_tree.set_region(IVec3::splat(8), 4, Voxel::ACTIVE);
    let mut fine_tree = Octree::uniform(3, Voxel::INACTIVE);
    fine_tree.set_region(IVec3::splat(2), 1, Voxel::ACTIVE);
    let ratio = signed_volume(&mesh_octree(&coarse_tree, IVec3::ZERO))
        / signed_volume(&mesh_octree(&fine_tree, IVec3::ZERO));
    assert!(
        (ratio - 64.0).abs() < 0.1,
        "4× scaled configuration should have exactly 64× the volume, got {ratio}"
    );
}

#[test]
fn coarse_voxel_next_to_fine_voxel_meshes_crack_free() {
    let mut world = World::new();
    let chunk = world.chunk_mut(IVec3::ZERO);
    chunk.set_region(IVec3::new(8, 8, 8), 4, Voxel::ACTIVE);
    // one fine voxel centred on the +x face of the coarse one
    chunk.set(IVec3::new(12, 9, 9), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "coarse+fine pair");
    assert!(signed_volume(&mesh) > 0.0);
}

/// The voxel-bevy `TestBed` scene: one 4³ voxel with two 2³ voxels on
/// its +x face, covering half of it. The half-covered coarse face is the
/// case that needs hanging dual cells in its polygon, so this pins the
/// demo scene's geometry as well as the behaviour.
#[test]
fn coarse_voxel_with_two_half_scale_neighbours_is_watertight() {
    let mut world = World::new();
    let chunk = world.chunk_mut(IVec3::ZERO);
    chunk.set_region(IVec3::new(12, 12, 12), 4, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 12, 12), 2, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 14, 12), 2, Voxel::ACTIVE);

    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "4³ voxel + two 2³ neighbours");
    assert!(signed_volume(&mesh) > 0.0);
    // The transition must actually exercise the n-gon path: a plain
    // all-quad dual would leave T-junction cracks here.
    assert!(
        mesh.faces.iter().any(|f| f.len() > 4),
        "expected n-gon faces at the scale transition"
    );
}

#[test]
fn coarse_slab_under_fine_terrain_transition_is_crack_free() {
    // Bottom half at 8× scale, top half fine — a big T-junction wall.
    let mut world = World::new();
    world.build_chunk(IVec3::ZERO, &|p: IVec3| {
        if p.y < 16 {
            Voxel::ACTIVE
        } else {
            Voxel::new(p.y <= 16 + ((p.x / 4 + p.z / 4) % 2) * 2)
        }
    });
    // sanity: the build actually produced coarse *and* fine active leaves
    let mut sizes = std::collections::HashSet::new();
    world
        .chunk(IVec3::ZERO)
        .unwrap()
        .tree()
        .for_each_leaf(&mut |_, size, v| {
            if v.is_active() {
                sizes.insert(size);
            }
        });
    assert!(
        sizes.len() > 1,
        "expected mixed active leaf sizes, got {sizes:?}"
    );

    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "coarse slab + fine steps");
}

#[test]
fn mixed_resolution_sphere_is_watertight() {
    // -x hemisphere sampled at 4× blocks, +x hemisphere at full detail.
    let fine = shapes::sphere(Vec3::splat(16.0), 11.0);
    let coarse = shapes::quantized(shapes::sphere(Vec3::splat(16.0), 11.0), 4);
    let world = world_from_fn(IVec3::ZERO, IVec3::ZERO, move |p: IVec3| {
        if p.x < 16 { coarse(p) } else { fine(p) }
    });
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "mixed-resolution sphere");
    assert!(signed_volume(&mesh) > 0.0);
}

// ------------------------------------------------------------ inspection

#[test]
fn dual_graph_is_the_full_adjacency_graph_and_its_crossings_are_the_faces() {
    let world = world_from_fn(
        IVec3::ZERO,
        IVec3::ZERO,
        shapes::sphere(Vec3::splat(16.0), 10.0),
    );
    let mesh = mesh_world(&world);
    let inspection = inspect_world(&world, Inspect::ALL);

    let crossed = inspection.dual_graph.iter().filter(|e| e.crossed).count();
    assert_eq!(
        crossed,
        mesh.faces.len(),
        "each crossed adjacency must produce exactly one surface polygon"
    );
    assert!(
        crossed < inspection.dual_graph.len(),
        "the graph must carry uncrossed adjacencies too, not just surface ones"
    );
    assert!(
        inspection.dual_graph.iter().all(|e| e.a != e.b),
        "an adjacency joins two distinct leaves"
    );
    // Every dual cell the mesher kept is genuinely a mixed one.
    assert!(!inspection.dual_cells.is_empty());
    assert!(
        inspection
            .dual_cells
            .iter()
            .all(|c| c.mask != 0 && c.mask != 0xFF)
    );
}

#[test]
fn inspection_is_opt_in() {
    let mut world = World::new();
    world.set_voxel(IVec3::new(4, 4, 4), Voxel::ACTIVE);
    let nothing = inspect_world(&world, Inspect::NONE);
    assert!(nothing.dual_graph.is_empty() && nothing.dual_cells.is_empty());

    // A lone voxel has exactly 6 crossed adjacencies — one per face.
    let graph = inspect_world(
        &world,
        Inspect {
            dual_graph: true,
            ..Inspect::NONE
        },
    );
    assert_eq!(graph.dual_graph.iter().filter(|e| e.crossed).count(), 6);
    assert!(graph.dual_cells.is_empty());
}

// ------------------------------------------------------------ chunk seams

/// Canonical form for comparing meshes independent of vertex/face order:
/// every face as its vertex positions, rotated so the lexicographically
/// smallest comes first (cyclic order preserved), then sorted.
fn canonical_faces(mesh: &PolyMesh) -> Vec<Vec<[u32; 3]>> {
    let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    let mut faces: Vec<Vec<[u32; 3]>> = mesh
        .faces
        .iter()
        .map(|f| {
            let ps: Vec<[u32; 3]> = f.iter().map(|&i| key(mesh.positions[i as usize])).collect();
            let start = (0..ps.len()).min_by_key(|&s| ps[s]).unwrap();
            (0..ps.len()).map(|i| ps[(start + i) % ps.len()]).collect()
        })
        .collect();
    faces.sort();
    faces
}

#[test]
fn chunked_sphere_equals_single_domain_sphere() {
    // A sphere centred on the corner where 8 chunks meet, meshed as a
    // chunked world (per-chunk + seam traversals) and as one depth-6
    // octree. The welded geometry must be *identical*.
    let f = shapes::sphere(Vec3::splat(32.0), 17.0);
    let world = world_from_fn(IVec3::ZERO, IVec3::ONE, &f);
    let chunked = mesh_world(&world);

    let single = Octree::from_fn(6, &mut |p| f(p));
    let reference = mesh_octree(&single, IVec3::ZERO);

    assert_closed(&chunked, 2, "chunked sphere");
    assert_eq!(
        canonical_faces(&chunked),
        canonical_faces(&reference),
        "chunked and single-domain meshes differ"
    );
}

#[test]
fn chunked_mixed_scale_shape_equals_single_domain() {
    // Mixed-resolution shape straddling all 8 chunks: coarse 8× blocks on
    // one side of the seam corner, fine on the other.
    let fine = shapes::sphere(Vec3::splat(32.0), 19.0);
    let coarse = shapes::quantized(shapes::sphere(Vec3::splat(32.0), 19.0), 8);
    let f = move |p: IVec3| if p.y < 32 { coarse(p) } else { fine(p) };

    let world = world_from_fn(IVec3::ZERO, IVec3::ONE, &f);
    let chunked = mesh_world(&world);

    let single = Octree::from_fn(6, &mut |p| f(p));
    let reference = mesh_octree(&single, IVec3::ZERO);

    assert_closed(&chunked, 2, "chunked mixed-scale sphere");
    assert_eq!(canonical_faces(&chunked), canonical_faces(&reference));
}

#[test]
fn terrain_across_many_chunks_is_watertight() {
    let world = world_from_fn(
        IVec3::new(0, 0, 0),
        IVec3::new(1, 0, 1),
        shapes::sine_terrain(20.0, 5.0, 9.0),
    );
    let mesh = mesh_world(&world);
    // Terrain spans 2×1×2 chunks and is capped at every world boundary →
    // one closed blob.
    assert_closed(&mesh, 2, "terrain 2×1×2 chunks");
}
