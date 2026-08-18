//! A box for every **active meshing cell** (dual cell): the cells the
//! 256-case patch table runs on, spanning between voxel centers — so
//! voxel centers sit on each box's corners, or on its edges/faces where a
//! coarser voxel fills several corner slots.

use bevy::prelude::*;

use crate::config;
use crate::debug::inspect::Inspected;

pub fn draw(mut gizmos: Gizmos, inspected: Res<Inspected>) {
    for cell in &inspected.0.dual_cells {
        let (min, max) = cell
            .corners
            .iter()
            .fold((cell.corners[0], cell.corners[0]), |(lo, hi), &c| {
                (lo.min(c), hi.max(c))
            });
        let extent = max - min;
        // A cell bordering a large empty leaf reaches all the way to that
        // leaf's center, so it can be far bigger than the voxels it
        // surfaces — legitimate, but it swamps the view. Skip those.
        if extent.max_element() > config::ACTIVE_CELL_BOX_MAX_EXTENT {
            continue;
        }
        gizmos.cube(
            Transform::from_translation((min + max) / 2.0).with_scale(extent),
            config::ACTIVE_CELL_BOX_COLOR,
        );
    }
}
