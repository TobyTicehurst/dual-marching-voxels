//! Debug renderers, each gated by a compile-time flag in
//! [`crate::config`]. All of them draw gizmos straight from the voxel
//! world every frame — deliberately simple, deliberately unoptimised.

pub mod active_cells;
pub mod grid;
pub mod voxel_markers;

use bevy::prelude::*;

use crate::config;

pub fn plugin(app: &mut App) {
    if config::SHOW_CHUNK_BOUNDS || config::SHOW_VOXEL_GRID {
        app.add_systems(Update, grid::draw);
    }
    if config::SHOW_ACTIVE_VOXEL_BOXES {
        app.add_systems(Update, active_cells::draw);
    }
    if config::SHOW_VOXEL_MARKERS {
        app.add_systems(Update, voxel_markers::draw);
    }
}
