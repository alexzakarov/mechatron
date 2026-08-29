// ============================================================================
// Primitives — mesh-only primitive generation for the ModelEditor.
//
// Declares mesh generators that produce MeshData for Blender-style primitives
// so the editor stays decoupled from CADKernel for purely procedural geometry.
// All primitives are unit-sized and centered on the origin in the XZ plane,
// matching the convention used by commit_interactive_add()'s half_extents
// scaling step.
//
// Definitions live in Primitives.cpp so the same translation unit backs both
// the editor (via mechatron_ui) and the unit tests (via mechatron_tests).
// ============================================================================

#pragma once

#include "cad/CADKernel.hpp"

namespace mechatron {

// Standard analytic torus: revolve a minor circle of radius `minor_radius`
// around the Y axis at distance `major_radius`. Lies in the XZ plane.
MeshData make_torus_mesh(int major_segments, int minor_segments,
                         float major_radius, float minor_radius);

// Suzanne (Blender's test monkey head). Reduced triangulated silhouette.
MeshData make_monkey_mesh();

// Cubic Bezier curve tessellated into a thin tube along Y=0.
MeshData make_bezier_curve_mesh(int samples, int tube_segments);

// Cubic B-spline ("NURBS-style") curve tessellated as a tube.
MeshData make_nurbs_curve_mesh(int samples, int tube_segments);

// Flat plane: a single quad in the XZ plane.
MeshData make_plane_mesh();

// Filled circle (disk) in the XZ plane.
MeshData make_circle_mesh(int segments);

// Subdivided grid in the XZ plane. Two triangles per cell.
MeshData make_grid_mesh(int divisions);

} // namespace mechatron
