//! Debug renderers, each gated by a compile-time flag in
//! [`crate::config`]. All of them draw gizmos straight from the voxel
//! world every frame — deliberately simple, deliberately unoptimised.

pub mod dual_cells;
pub mod grid;
pub mod voxel_boxes;
pub mod voxel_markers;

use bevy::prelude::*;

use crate::{chunk_render, config};

pub fn plugin(app: &mut App) {
    if config::SHOW_CHUNK_BOUNDS || config::SHOW_VOXEL_GRID {
        app.add_systems(Update, grid::draw);
    }
    if config::SHOW_ACTIVE_VOXEL_BOXES {
        app.add_systems(Update, voxel_boxes::draw);
    }
    if config::SHOW_VOXEL_MARKERS {
        app.add_systems(Update, voxel_markers::draw);
    }
    if config::SHOW_ACTIVE_CELL_BOXES {
        app.add_systems(Startup, dual_cells::collect.after(chunk_render::setup))
            .add_systems(Update, dual_cells::draw);
    }
}
