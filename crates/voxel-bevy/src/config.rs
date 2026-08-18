//! Compile-time debug/render configuration. Recompiling to change these is
//! the intended workflow; none of the debug renderers are optimised.

use bevy::color::Color;

use crate::debug::voxel_markers::MarkerShape;
use crate::scene::Scene;

/// Which scene to build — see [`crate::scene`]. `TestBed` is the small
/// hand-placed scratch scene for testing one behaviour at a time;
/// `Landscape` is the full demo.
pub const SCENE: Scene = Scene::Landscape;

/// Outline every chunk's 32³ bounds.
pub const SHOW_CHUNK_BOUNDS: bool = true;
pub const CHUNK_BOUNDS_COLOR: Color = Color::srgb(0.25, 0.25, 0.35);

/// The full finest-scale voxel grid of every chunk. Very heavy (≈ 3300
/// gizmo lines per chunk) — turn on for small scenes only.
pub const SHOW_VOXEL_GRID: bool = false;
pub const VOXEL_GRID_COLOR: Color = Color::srgba(0.3, 0.3, 0.3, 0.35);

/// A wireframe box around every *active* voxel, drawn at the voxel's own
/// scale — the clearest view of the multi-scale octree structure.
pub const SHOW_ACTIVE_VOXEL_BOXES: bool = false;
pub const ACTIVE_VOXEL_BOX_COLOR: Color = Color::srgb(0.1, 0.9, 0.9);

/// A box for every active *meshing* cell (dual cell) — the cells the
/// 256-case table runs on, spanning between voxel centers. Voxel centers
/// sit on each box's corners, or on its edges/faces where a coarser voxel
/// fills several corner slots. Off by default only because it overlaps
/// the dual-graph view below; the two together are hard to read.
pub const SHOW_ACTIVE_CELL_BOXES: bool = false;
pub const ACTIVE_CELL_BOX_COLOR: Color = Color::srgb(0.95, 0.2, 0.95);
/// Cells wider than this on any axis are not drawn: one bordering a large
/// empty leaf stretches to that leaf's center and would swamp the view.
/// Raise it to see those too (`f32::INFINITY` draws every cell).
pub const ACTIVE_CELL_BOX_MAX_EXTENT: f32 = 4.0;

/// A small solid marker at every active voxel's center, sized to that
/// voxel — the nodes of the dual graph below.
pub const SHOW_VOXEL_MARKERS: bool = false;
pub const VOXEL_MARKER_COLOR: Color = Color::srgb(0.95, 0.6, 0.1);
/// Marker radius as a fraction of the voxel size (0.5 = touching faces).
pub const VOXEL_MARKER_RADIUS: f32 = 0.2;
/// Markers are merged into one mesh, so this sets the view's whole
/// triangle count: 80 per marker for `Sphere`, 4 for `Tetrahedron`.
/// Measured in the `Landscape` scene, both render at the same frame rate
/// (1.6M vs 79k triangles, no difference) — pick on looks, not cost.
pub const VOXEL_MARKER_SHAPE: MarkerShape = MarkerShape::Sphere;

/// The octree's dual graph: an edge between the centers of every pair of
/// face-adjacent leaves. Crossed edges — where the two leaves differ in
/// state, so the mesher emits a surface polygon — are highlighted.
pub const SHOW_DUAL_GRAPH: bool = false;
pub const DUAL_GRAPH_CROSSED_COLOR: Color = Color::srgb(0.2, 1.0, 0.35);
pub const DUAL_GRAPH_COLOR: Color = Color::srgba(0.5, 0.55, 0.65, 0.5);
/// Draw the adjacencies that produce no surface, too. Off keeps the view
/// to just the edges that become polygons.
pub const DUAL_GRAPH_SHOW_UNCROSSED: bool = false;
/// Edges longer than this are not drawn: an adjacency against a large
/// empty leaf reaches that leaf's center, far outside the interesting
/// region. (`f32::INFINITY` draws every edge.)
pub const DUAL_GRAPH_MAX_LENGTH: f32 = 6.0;

/// Whether the meshed surface entity is spawned at all.
pub const SHOW_MESHED_SURFACE: bool = true;
/// Opacity of the meshed surface. Below 1.0 it turns translucent, which
/// is how the markers and dual graph *inside* the volume stay visible;
/// set it to 1.0 for an honest look at the shaded result.
pub const SURFACE_ALPHA: f32 = 1.0;
/// Start with wireframe rendering on (toggle at runtime with F). Note
/// this overlays the fill rather than replacing it — use `SURFACE_ALPHA`
/// to see inside.
pub const START_WIREFRAME: bool = false;
