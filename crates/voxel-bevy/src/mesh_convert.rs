//! `voxel_mesh::MeshData` → `bevy::Mesh`. The only place mesh buffers
//! change hands between the engine-agnostic crates and Bevy.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};

use voxel_mesh::MeshData;

pub fn to_bevy_mesh(data: &MeshData) -> Mesh {
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        data.positions
            .iter()
            .map(|p| p.to_array())
            .collect::<Vec<_>>(),
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_NORMAL,
        data.normals
            .iter()
            .map(|n| n.to_array())
            .collect::<Vec<_>>(),
    )
    .with_inserted_indices(Indices::U32(data.indices.clone()))
}
