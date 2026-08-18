//! Debug renderers, each gated by a compile-time flag in
//! [`crate::config`]. Deliberately simple, deliberately unoptimised —
//! though the marker view merges its geometry, since it scales with the
//! voxel count rather than with what is on screen.

pub mod dual_cells;
pub mod dual_graph;
pub mod grid;
pub mod inspect;
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
        app.add_systems(Startup, voxel_markers::spawn.after(chunk_render::setup));
    }
    // Both dual views read one shared inspection run.
    if config::SHOW_ACTIVE_CELL_BOXES || config::SHOW_DUAL_GRAPH {
        app.add_systems(Startup, inspect::collect.after(chunk_render::setup));
    }
    if config::SHOW_ACTIVE_CELL_BOXES {
        app.add_systems(Update, dual_cells::draw);
    }
    if config::SHOW_DUAL_GRAPH {
        app.add_systems(Update, dual_graph::draw);
    }
}
