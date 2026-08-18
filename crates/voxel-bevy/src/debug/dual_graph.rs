//! The octree's **dual graph**: a line between the centers of every pair
//! of face-adjacent leaves.
//!
//! This is the structure the whole mesher reads off. Its nodes are the
//! voxel centers (the marker view draws those); each **crossed** edge —
//! the two leaves differ in state — becomes exactly one surface polygon,
//! and those are drawn highlighted. Where scales differ, an edge runs
//! diagonally, because a coarse leaf's center is offset from its finer
//! neighbour's.

use bevy::prelude::*;

use crate::config;
use crate::debug::inspect::Inspected;

pub fn draw(mut gizmos: Gizmos, inspected: Res<Inspected>) {
    for edge in &inspected.0.dual_graph {
        if !edge.crossed && !config::DUAL_GRAPH_SHOW_UNCROSSED {
            continue;
        }
        // Adjacencies against a large empty leaf reach that leaf's center,
        // far outside the region of interest — same filter as the cells.
        if edge.a.distance(edge.b) > config::DUAL_GRAPH_MAX_LENGTH {
            continue;
        }
        let color = if edge.crossed {
            config::DUAL_GRAPH_CROSSED_COLOR
        } else {
            config::DUAL_GRAPH_COLOR
        };
        gizmos.line(edge.a, edge.b, color);
    }
}
