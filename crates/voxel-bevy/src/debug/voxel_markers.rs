//! A small solid marker at every active voxel's center, sized to that
//! voxel — the nodes of the dual graph.
//!
//! Every marker is merged into one mesh at startup, so the whole view is
//! a single entity and a single draw call with no per-frame work. That
//! also means the marker shape's triangle count is the only thing that
//! costs anything as scenes grow: see [`MarkerShape`].

use bevy::mesh::{Indices, VertexAttributeValues};
use bevy::prelude::*;
use voxel_core::world::World as VoxelWorld;

use crate::chunk_render::VoxelWorldRes;
use crate::config;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[allow(dead_code)] // one variant is always the unselected compile-time choice
pub enum MarkerShape {
    /// Once-subdivided icosahedron: 80 triangles per marker.
    Sphere,
    /// 4 triangles per marker — 20× lighter, for scenes with many voxels.
    Tetrahedron,
}

pub fn spawn(
    mut commands: Commands,
    world: Res<VoxelWorldRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let base = base_mesh(config::VOXEL_MARKER_SHAPE);
    let (base_positions, base_normals, base_indices) = mesh_parts(&base);

    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    for coord in world.0.chunk_coords() {
        let origin = VoxelWorld::chunk_origin(coord).as_vec3();
        let Some(chunk) = world.0.chunk(coord) else {
            continue;
        };
        chunk.tree().for_each_leaf(&mut |min, size, voxel| {
            if !voxel.is_active() {
                return;
            }
            let size = size as f32;
            let center = origin + min.as_vec3() + Vec3::splat(size / 2.0);
            let radius = size * config::VOXEL_MARKER_RADIUS;

            let base_index = positions.len() as u32;
            positions.extend(
                base_positions
                    .iter()
                    .map(|p| (Vec3::from(*p) * radius + center).to_array()),
            );
            // Markers are only translated and uniformly scaled, so the
            // base shape's normals stay correct as they are.
            normals.extend_from_slice(base_normals);
            indices.extend(base_indices.iter().map(|i| i + base_index));
        });
    }

    if indices.is_empty() {
        return;
    }
    info!(
        "voxel markers: {} triangles ({:?})",
        indices.len() / 3,
        config::VOXEL_MARKER_SHAPE
    );

    let merged = Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(Indices::U32(indices));

    commands.spawn((
        Mesh3d(meshes.add(merged)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: config::VOXEL_MARKER_COLOR,
            perceptual_roughness: 0.8,
            ..default()
        })),
    ));
}

/// The marker shape, normalised so its furthest vertex sits at distance 1
/// from the center — making the two shapes the same visual size for a
/// given radius.
fn base_mesh(shape: MarkerShape) -> Mesh {
    let mut mesh = match shape {
        MarkerShape::Sphere => Sphere::new(1.0)
            .mesh()
            .ico(1)
            .expect("1 subdivision is well within the icosphere limit"),
        MarkerShape::Tetrahedron => Tetrahedron::default().mesh().build(),
    };
    let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    else {
        panic!("marker base mesh has no float3 positions");
    };
    let scale = positions
        .iter()
        .map(|p| Vec3::from(*p).length())
        .fold(0.0f32, f32::max)
        .recip();
    for p in positions.iter_mut() {
        *p = (Vec3::from(*p) * scale).to_array();
    }
    mesh
}

fn mesh_parts(mesh: &Mesh) -> (&[[f32; 3]], &[[f32; 3]], Vec<u32>) {
    let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    else {
        panic!("marker base mesh has no float3 positions");
    };
    let Some(VertexAttributeValues::Float32x3(normals)) = mesh.attribute(Mesh::ATTRIBUTE_NORMAL)
    else {
        panic!("marker base mesh has no float3 normals");
    };
    let indices = match mesh.indices().expect("marker base mesh is indexed") {
        Indices::U16(v) => v.iter().map(|&i| i as u32).collect(),
        Indices::U32(v) => v.clone(),
    };
    (positions, normals, indices)
}
