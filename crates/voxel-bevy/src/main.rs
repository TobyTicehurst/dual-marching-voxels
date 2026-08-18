//! The demo binary: multi-scale DMC-meshed voxel world rendered with Bevy.
//! This crate is the only place Bevy is allowed to appear.

mod chunk_render;
mod config;
mod debug;
mod fly_camera;
mod mesh_convert;
mod scene;

use bevy::pbr::wireframe::WireframePlugin;
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WireframePlugin::default())
        .add_plugins((chunk_render::plugin, fly_camera::plugin, debug::plugin))
        .run();
}
