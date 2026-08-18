//! Builds the demo world, meshes it, and spawns the render entities.

use bevy::pbr::wireframe::WireframeConfig;
use bevy::prelude::*;

use voxel_mesh::{DualMarchingCubes, MeshData, Mesher};

use crate::{config, mesh_convert, scene};

/// The voxel data behind the rendered scene, readable by debug renderers.
#[derive(Resource)]
pub struct VoxelWorldRes(pub voxel_core::World);

pub fn plugin(app: &mut App) {
    app.add_systems(Startup, setup)
        .add_systems(Update, toggle_wireframe);
}

pub(crate) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut wireframe: ResMut<WireframeConfig>,
) {
    let world = scene::build_demo_world();

    if config::SHOW_MESHED_SURFACE {
        let poly = DualMarchingCubes.mesh_world(&world);
        let data = MeshData::flat_from_polys(&poly);
        info!(
            "meshed {} chunks: {} polygons, {} triangles",
            world.chunk_coords().len(),
            poly.faces.len(),
            data.indices.len() / 3
        );
        commands.spawn((
            Mesh3d(meshes.add(mesh_convert::to_bevy_mesh(&data))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.55, 0.65, 0.5),
                perceptual_roughness: 0.95,
                ..default()
            })),
            Transform::IDENTITY,
        ));
    }
    commands.insert_resource(VoxelWorldRes(world));

    wireframe.global = config::START_WIREFRAME;

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, 0.5, 0.0)),
    ));

    info!("controls: mouse to look, WASD + QE to fly, scroll = speed, F = wireframe, Esc = quit");
}

fn toggle_wireframe(keys: Res<ButtonInput<KeyCode>>, mut wireframe: ResMut<WireframeConfig>) {
    if keys.just_pressed(KeyCode::KeyF) {
        wireframe.global = !wireframe.global;
    }
}
