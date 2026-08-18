//! Runs the mesher once in inspection mode at startup and keeps what it
//! saw, so the dual-cell and dual-graph views share a single traversal.

use bevy::prelude::*;
use voxel_mesh::dmc::{Inspect, Inspection, inspect_world};

use crate::chunk_render::VoxelWorldRes;
use crate::config;

#[derive(Resource)]
pub struct Inspected(pub Inspection);

/// What the enabled debug views need. Nothing is collected for a view
/// that is switched off.
pub const WANTED: Inspect = Inspect {
    dual_cells: config::SHOW_ACTIVE_CELL_BOXES,
    dual_graph: config::SHOW_DUAL_GRAPH,
};

pub fn collect(mut commands: Commands, world: Res<VoxelWorldRes>) {
    let inspection = inspect_world(&world.0, WANTED);
    info!(
        "inspection: {} active meshing cells, {} dual-graph edges",
        inspection.dual_cells.len(),
        inspection.dual_graph.len()
    );
    commands.insert_resource(Inspected(inspection));
}
