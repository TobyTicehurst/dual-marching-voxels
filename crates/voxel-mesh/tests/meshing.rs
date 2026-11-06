//! End-to-end meshing tests: uniform resolution (milestone 3), mixed
//! scales (milestone 4) and chunk seams (milestone 5).

use glam::{IVec3, Vec3};
use voxel_core::octree::Octree;
use voxel_core::shapes;
use voxel_core::voxel::Voxel;
use voxel_core::world::World;
use voxel_mesh::analysis::{edge_stats, euler_characteristic, signed_volume, weld};
use voxel_mesh::dmc::{Inspect, inspect_world, mesh_octree, mesh_world};
use voxel_mesh::mesh_data::MeshData;
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
/// Two voxels meeting along an edge mesh as **one manifold** surface: the
/// ambiguous faces between them are resolved to join the material, and
/// the segment vertices give that join a throat instead of a pinch.
///
/// Without them the two dual cells straddling the contact both place
/// their vertex on the contact line itself and the two surface segments
/// between them collapse onto a single mesh edge, which the two voxels'
/// four polygons then all share. Splitting the segments turns that edge
/// into a four-sided opening, so each of those four polygons picks up one
/// extra corner and every edge is back to exactly two faces.
#[test]
fn edge_touching_voxels_mesh_as_one_surface_with_a_manifold_throat() {
    let mut world = World::new();
    world.set_voxel(IVec3::new(4, 4, 4), Voxel::ACTIVE);
    world.set_voxel(IVec3::new(5, 5, 4), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_eq!(mesh.faces.len(), 12);
    let widened = mesh.faces.iter().filter(|f| f.len() == 5).count();
    assert_eq!(widened, 4, "the four polygons around the contact");

    // Two blobs joined by one throat is still a sphere: even Euler
    // characteristic, and no edge shared by more than two faces. Before
    // the split this was 3 with one non-manifold edge — an odd
    // characteristic being impossible for a closed manifold.
    assert_closed(&mesh, 2, "edge-touching pair");

    // The throat is a real opening, not a collapsed one: the two segment
    // vertices sit either side of the contact line (x = y = 5).
    let throat: Vec<Vec3> = mesh
        .positions
        .iter()
        .copied()
        .filter(|p| p.z == 4.5 && p.x != 5.0)
        .collect();
    assert_eq!(throat.len(), 2, "two segment vertices: {throat:?}");
    assert!(
        (throat[0] - throat[1]).length() > 0.5,
        "throat is pinched shut: {throat:?}"
    );
}

/// A run of edge contacts is still one manifold tube — the throats do
/// not interfere with each other even when consecutive voxels share the
/// dual cells around their contacts.
#[test]
fn a_diagonal_staircase_is_one_manifold_tube() {
    let mut world = World::new();
    for i in 0..6 {
        world.set_voxel(IVec3::new(4 + i, 4 + i, 4), Voxel::ACTIVE);
    }
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "diagonal staircase");
    assert!(signed_volume(&mesh) > 0.0);
}

/// The same contact across a scale transition. The two leaves are
/// different sizes, so the segment's two crossings are the centers of
/// differently-sized faces — the throat is lopsided but must still be a
/// throat.
#[test]
fn edge_touching_voxels_of_different_scales_stay_manifold() {
    let mut world = World::new();
    let chunk = world.chunk_mut(IVec3::ZERO);
    chunk.set_region(IVec3::new(8, 8, 8), 4, Voxel::ACTIVE);
    // one fine voxel touching the coarse voxel's +x/+y edge only
    chunk.set(IVec3::new(12, 12, 9), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "coarse/fine edge contact");
    assert!(signed_volume(&mesh) > 0.0);
}

/// Corner-only contacts are untouched by the face rule: no single face
/// sees both voxels, so there is no ambiguous face to resolve and the two
/// stay separate closed cubes. Connecting these would need a rule about
/// the cell interior instead.
#[test]
fn body_diagonal_voxels_stay_separate() {
    let mut world = World::new();
    world.set_voxel(IVec3::new(4, 4, 4), Voxel::ACTIVE);
    world.set_voxel(IVec3::new(5, 5, 5), Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 4, "body-diagonal pair");
}

#[test]
fn checkerboard_cluster_meshes_as_a_single_blob() {
    // Four mutually edge-touching voxels of a 2×2×2 block — the hardest
    // ambiguous configuration, every shared cell alternating. They all
    // meet around the block's center, so joining them yields one closed
    // surface with no handles (and no pinch: the junction is a volume,
    // not a single edge).
    let mut world = World::new();
    for o in [0b000usize, 0b011, 0b101, 0b110] {
        let p = IVec3::new((o & 1) as i32, ((o >> 1) & 1) as i32, ((o >> 2) & 1) as i32);
        world.set_voxel(IVec3::splat(8) + p, Voxel::ACTIVE);
    }
    let mesh = mesh_world(&world);
    assert_closed(&mesh, 2, "checkerboard");
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

/// One 4³ voxel with two 2³ voxels on its +x face, covering half of it.
/// The half-covered coarse face is the case that needs hanging dual cells
/// in its polygon.
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

// ------------------------------------------------------ vertex placement

/// An isolated voxel is symmetric, so its mesh must be too — and it must
/// not matter where the voxel sits. Every crossing is the center of the
/// voxel's own face (its empty neighbours are never smaller than it), so
/// each of the 8 dual vertices is the centroid of three of those face
/// centers: a cube of half-extent `size / 6` about the voxel center.
///
/// This fails if crossings are placed on the segment between leaf
/// centers, because then the surface follows the size of whatever empty
/// leaf happens to be opposite — which varies from face to face, moves
/// when the voxel is re-aligned in the octree, and goes wild at the world
/// edge where the neighbour is a whole absent chunk.
#[test]
fn a_lone_voxel_meshes_to_the_same_symmetric_cube_wherever_it_sits() {
    for (min, size) in [
        (IVec3::splat(16), 1u32),  // middle of a chunk
        (IVec3::new(5, 9, 23), 1), // arbitrary, off every node boundary
        (IVec3::ZERO, 1),          // corner: three sides face absent chunks
        (IVec3::splat(12), 4),     // aligned to octree node boundaries
        (IVec3::splat(16), 4),     // one node over — the mirrored case
        (IVec3::splat(28), 4),     // against the far corner of the chunk
    ] {
        let mut world = World::new();
        world
            .chunk_mut(IVec3::ZERO)
            .set_region(min, size, Voxel::ACTIVE);
        let mesh = weld(&mesh_world(&world));
        let what = format!("{size}³ voxel at {min}");

        assert_closed(&mesh, 2, &what);
        assert_eq!(mesh.positions.len(), 8, "{what}: expected 8 dual vertices");

        let center = min.as_vec3() + Vec3::splat(size as f32 / 2.0);
        let half = Vec3::splat(size as f32 / 6.0);
        for p in &mesh.positions {
            assert!(
                (*p - center).abs().abs_diff_eq(half, 1e-4),
                "{what}: vertex {p:?} is not at the corner of a cube of \
                 half-extent {half:?} about {center:?}"
            );
        }
    }
}

/// A shape must mesh the same way wherever it is put, even though moving
/// it changes the sizes of the merged empty leaves around it completely.
#[test]
fn a_shape_meshes_congruently_wherever_it_is_placed() {
    // An L of mixed scales, so this covers transitions as well as the
    // lone-voxel case above.
    let build = |origin: IVec3| {
        let mut world = World::new();
        let chunk = world.chunk_mut(IVec3::ZERO);
        chunk.set_region(origin, 4, Voxel::ACTIVE);
        chunk.set_region(origin + IVec3::new(4, 0, 0), 2, Voxel::ACTIVE);
        chunk.set_region(origin + IVec3::new(0, 4, 0), 2, Voxel::ACTIVE);
        world
    };
    let shape_at = |origin: IVec3| {
        let mesh = weld(&mesh_world(&build(origin)));
        // Positions relative to the shape, rounded before comparing:
        // subtracting different origins from the same geometry can differ
        // in the last bit, which says nothing about congruence.
        let mut relative: Vec<[i64; 3]> = mesh
            .positions
            .iter()
            .map(|p| {
                let d = (*p - origin.as_vec3()) * 10_000.0;
                [d.x.round() as i64, d.y.round() as i64, d.z.round() as i64]
            })
            .collect();
        relative.sort();
        relative
    };

    // (4,4,4) is deep inside one octree node; (12,12,12) and (16,16,16)
    // sit on different node boundaries; (24,24,24) puts the shape against
    // the chunk's far corner, where the neighbours are absent chunks.
    let reference = shape_at(IVec3::splat(4));
    assert!(!reference.is_empty());
    for origin in [IVec3::splat(12), IVec3::splat(16), IVec3::splat(24)] {
        assert_eq!(
            shape_at(origin),
            reference,
            "the same shape meshed differently at {origin}"
        );
    }
}

// ------------------------------------------------- render triangulation

/// Area-weighted normal of a polygon, robust to nearly collinear corners.
fn newell(corners: &[Vec3]) -> Vec3 {
    let mut n = Vec3::ZERO;
    for i in 0..corners.len() {
        n += corners[i].cross(corners[(i + 1) % corners.len()]);
    }
    n
}

/// No triangle may face away from the polygon it came from. Scale
/// transitions produce n-gons that are mildly non-convex, and fanning
/// those from a fixed corner emits a backfacing sliver that renders as a
/// spike sticking out of the surface.
fn assert_no_folded_triangles(mesh: &PolyMesh, what: &str) {
    let mut checked = 0;
    for (i, face) in mesh.faces.iter().enumerate() {
        let corners: Vec<Vec3> = face.iter().map(|&v| mesh.positions[v as usize]).collect();
        let normal = newell(&corners);
        if normal.length_squared() <= 1e-12 {
            continue;
        }
        let normal = normal.normalize();

        // Triangulate this one face through the real API.
        let mut single = PolyMesh::new();
        let single_face: Vec<u32> = corners.iter().map(|&p| single.push_vertex(p)).collect();
        single.push_face(single_face);
        let data = MeshData::flat_from_polys(&single);

        for triangle in data.normals.chunks(3) {
            assert!(
                triangle[0].dot(normal) > 0.0,
                "{what}: face {i} ({} corners) produced a triangle facing {:?}, \
                 but the face faces {normal:?}",
                corners.len(),
                triangle[0]
            );
        }
        checked += 1;
    }
    assert!(checked > 0, "{what}: nothing was checked");
}

#[test]
fn scale_transitions_triangulate_without_folding() {
    // The voxel-bevy TestBed scene: a coarse voxel, two half-scale
    // neighbours on its face, and coarse voxels stacked above — the
    // configuration whose transition n-gons exposed the fold.
    let mut world = World::new();
    let chunk = world.chunk_mut(IVec3::ZERO);
    chunk.set_region(IVec3::new(12, 12, 12), 4, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 12, 12), 2, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 14, 12), 2, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(12, 16, 12), 4, Voxel::ACTIVE);
    chunk.set_region(IVec3::new(16, 16, 12), 4, Voxel::ACTIVE);
    let mesh = mesh_world(&world);
    assert!(
        mesh.faces.iter().any(|f| f.len() > 4),
        "this configuration must exercise transition n-gons"
    );
    assert_no_folded_triangles(&mesh, "stacked scale transition");
}

#[test]
fn mixed_resolution_shapes_triangulate_without_folding() {
    let fine = shapes::sphere(Vec3::splat(16.0), 11.0);
    let coarse = shapes::quantized(shapes::sphere(Vec3::splat(16.0), 11.0), 4);
    let world = world_from_fn(IVec3::ZERO, IVec3::ZERO, move |p: IVec3| {
        if p.x < 16 { coarse(p) } else { fine(p) }
    });
    assert_no_folded_triangles(&mesh_world(&world), "mixed-resolution sphere");
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
