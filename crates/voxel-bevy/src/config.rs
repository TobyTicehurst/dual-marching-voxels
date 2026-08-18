//! Compile-time debug/render configuration. Recompiling to change these is
//! the intended workflow; none of the debug renderers are optimised.

use bevy::color::Color;

use crate::scene::Scene;

/// Which scene to build — see [`crate::scene`]. `TestBed` is the small
/// hand-placed scratch scene for testing one behaviour at a time;
/// `Landscape` is the full demo.
pub const SCENE: Scene = Scene::TestBed;

/// Outline every chunk's 32³ bounds.
pub const SHOW_CHUNK_BOUNDS: bool = true;
pub const CHUNK_BOUNDS_COLOR: Color = Color::srgb(0.25, 0.25, 0.35);

/// The full finest-scale voxel grid of every chunk. Very heavy (≈ 3300
/// gizmo lines per chunk) — turn on for small scenes only.
pub const SHOW_VOXEL_GRID: bool = false;
pub const VOXEL_GRID_COLOR: Color = Color::srgba(0.3, 0.3, 0.3, 0.35);

/// A wireframe box around every *active* voxel, drawn at the voxel's own
/// scale — the clearest view of the multi-scale octree structure.
pub const SHOW_ACTIVE_VOXEL_BOXES: bool = true;
pub const ACTIVE_VOXEL_BOX_COLOR: Color = Color::srgb(0.1, 0.9, 0.9);

/// A box for every active *meshing* cell (dual cell) — the cells the
/// 256-case table runs on, spanning between voxel centers. Voxel centers
/// sit on each box's corners, or on its edges/faces where a coarser voxel
/// fills several corner slots.
pub const SHOW_ACTIVE_CELL_BOXES: bool = true;
pub const ACTIVE_CELL_BOX_COLOR: Color = Color::srgb(0.95, 0.2, 0.95);
/// Cells wider than this on any axis are not drawn: one bordering a large
/// empty leaf stretches to that leaf's center and would swamp the view.
/// Raise it to see those too (`f32::INFINITY` draws every cell).
pub const ACTIVE_CELL_BOX_MAX_EXTENT: f32 = 4.0;

/// A small sphere at every active voxel's center, scaled to the voxel.
pub const SHOW_VOXEL_MARKERS: bool = true;
pub const VOXEL_MARKER_COLOR: Color = Color::srgb(0.95, 0.6, 0.1);
/// Marker radius as a fraction of the voxel size (0.5 = touching faces).
pub const VOXEL_MARKER_RADIUS: f32 = 0.35;

/// Whether the meshed surface entity is spawned at all.
pub const SHOW_MESHED_SURFACE: bool = true;
/// Start with wireframe rendering on (toggle at runtime with F).
pub const START_WIREFRAME: bool = false;
