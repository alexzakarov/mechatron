#pragma once

#include "CADKernel.hpp"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <deque>
#include <functional>
#include <cmath>
#include <array>

namespace mechatron {

// ============================================================================
// Selection mode
// ============================================================================
enum class SelectMode {
    Vertex,
    Edge,
    Face
};

enum class SelectionOp {
    Replace,
    Add,
    Subtract
};

// ============================================================================
// Edge (index pair with hash)
// ============================================================================
struct EdgeKey {
    uint32_t v0, v1;
    EdgeKey() : v0(0), v1(0) {}
    EdgeKey(uint32_t a, uint32_t b) : v0(std::min(a, b)), v1(std::max(a, b)) {}
    bool operator==(const EdgeKey& o) const { return v0 == o.v0 && v1 == o.v1; }
    struct Hash {
        size_t operator()(const EdgeKey& k) const {
            return (size_t)k.v0 * 2654435761u ^ (size_t)k.v1;
        }
    };
};

// ============================================================================
// EditableMesh - Full Blender-like mesh data model
//
// Features:
//   - Vertex / Edge / Face selection modes
//   - Box select, circle select
//   - Undo/redo (command pattern)
//   - Extrude (vertex, edge, face, region)
//   - Inset faces
//   - Bevel edges/vertices
//   - Loop cut
//   - Delete (vertices, edges, faces)
//   - Merge vertices
//   - Subdivide edges
//   - Flip normals
//   - Recalculate normals
// ============================================================================
class EditableMesh {
public:
    // --- I/O ---
    bool from_meshdata(const MeshData& md);
    MeshData to_meshdata() const;

    // --- Counts ---
    size_t vertex_count() const { return m_vertices.size(); }
    size_t triangle_count() const { return m_tris.size(); }
    size_t edge_count() const;

    // --- Accessors ---
    const std::vector<Vec3>& vertices() const { return m_vertices; }
    const std::vector<Triangle>& triangles() const { return m_tris; }
    std::vector<Vec3>& vertices_mut() { return m_vertices; }

    // --- Selection ---
    void set_select_mode(SelectMode mode) { m_select_mode = mode; }
    SelectMode select_mode() const { return m_select_mode; }

    void clear_selection();
    void select_all();
    void invert_selection();

    // Vertex selection
    void select_vertex(uint32_t idx, bool add);
    void deselect_vertex(uint32_t idx);
    bool is_vertex_selected(uint32_t idx) const;
    const std::unordered_set<uint32_t>& selected_vertices() const { return m_sel_verts; }

    // Edge selection
    void select_edge(const EdgeKey& ek, bool add);
    void deselect_edge(const EdgeKey& ek);
    bool is_edge_selected(const EdgeKey& ek) const;
    const std::unordered_set<EdgeKey, EdgeKey::Hash>& selected_edges() const { return m_sel_edges; }

    // Face selection
    void select_face(uint32_t tri_idx, bool add);
    void deselect_face(uint32_t tri_idx);
    bool is_face_selected(uint32_t tri_idx) const;
    const std::unordered_set<uint32_t>& selected_faces() const { return m_sel_faces; }

    // Box select (screen-space predicate: returns true if point is inside rect)
    void box_select(float x0, float y0, float x1, float y1,
                    const std::function<bool(const Vec3&, float&, float&)>& project_fn);
    void box_select(float x0, float y0, float x1, float y1, SelectionOp op,
                    const std::function<bool(const Vec3&, float&, float&)>& project_fn);

    // Circle select
    void circle_select(float cx, float cy, float radius, bool select,
                       const std::function<bool(const Vec3&, float&, float&)>& project_fn);
    void circle_select(float cx, float cy, float radius, SelectionOp op,
                       const std::function<bool(const Vec3&, float&, float&)>& project_fn);
    void polygon_select(const std::vector<std::array<float, 2>>& polygon, SelectionOp op,
                        const std::function<bool(const Vec3&, float&, float&)>& project_fn);

    // --- Queries ---
    // Get all edges (built from triangles on demand)
    std::vector<EdgeKey> get_edges() const;
    // Get edges adjacent to a vertex
    std::vector<EdgeKey> get_vertex_edges(uint32_t v) const;
    // Get faces adjacent to a vertex
    std::vector<uint32_t> get_vertex_faces(uint32_t v) const;
    // Get faces sharing an edge
    std::vector<uint32_t> get_edge_faces(const EdgeKey& ek) const;
    // Get vertices of a face
    void get_face_vertices(uint32_t tri_idx, uint32_t& a, uint32_t& b, uint32_t& c) const;

    // --- Basic edit ops ---
    bool set_vertex_pos(uint32_t idx, const Vec3& p);
    void translate_selected(const Vec3& delta);
    void rotate_selected(const Vec3& pivot, const Vec3& axis, float angle_deg);
    void scale_selected(const Vec3& pivot, const Vec3& factor);
    bool add_vertex(const Vec3& p, uint32_t* out_idx = nullptr);
    bool add_triangle(uint32_t a, uint32_t b, uint32_t c);
    void delete_selected_vertices();
    void delete_selected_edges();
    void delete_selected_faces();
    void delete_selected(); // dispatches based on select mode
    void delete_loose_vertices();

    // --- Mesh editing operations ---
    // Extrude selected faces (returns new face indices)
    std::vector<uint32_t> extrude_faces(const std::vector<uint32_t>& face_indices, float distance);
    // Extrude individual faces
    std::vector<uint32_t> extrude_faces_individual(const std::vector<uint32_t>& face_indices, float distance);
    // Extrude selected edges
    std::vector<uint32_t> extrude_edges(const std::vector<EdgeKey>& edges, float distance);
    // Extrude selected (dispatches by mode)
    void extrude_selected(float distance);

    // Inset faces (shrink selected faces inward)
    std::vector<uint32_t> inset_faces(const std::vector<uint32_t>& face_indices, float thickness, float depth);

    // Bevel edges
    void bevel_edges(const std::vector<EdgeKey>& edges, float amount, int segments);

    // Loop cut (insert a ring of edges around the mesh)
    int loop_cut(const EdgeKey& start_edge, float position);

    // Subdivide edges
    void subdivide_edges(const std::vector<EdgeKey>& edges, int cuts);

    // Merge selected vertices (to center, or to first selected)
    uint32_t merge_selected_vertices_to_center();
    uint32_t merge_selected_vertices_to_first();

    // Flip face normals of selected faces
    void flip_selected_normals();
    // Recalculate normals for all faces (make consistent winding)
    void recalculate_normals();

    // Duplicate selected
    void duplicate_selected();

    // --- Undo/Redo ---
    void push_undo(const std::string& name = "");
    bool undo();
    bool redo();
    bool can_undo() const;
    bool can_redo() const;
    std::string undo_name() const;
    std::string redo_name() const;
    void clear_undo_stack();

    // --- Utility ---
    // Rebuild edge map from triangles
    void rebuild_edge_map();
    Vec3 face_normal(uint32_t tri_idx) const;
    Vec3 face_center(uint32_t tri_idx) const;
    Vec3 edge_center(const EdgeKey& ek) const;
    // Get selected elements as a unified list (by mode)
    void get_selection_center(Vec3& out) const;

private:
    std::vector<Vec3> m_vertices;
    std::vector<Triangle> m_tris;

    // Edge topology (rebuilt from triangles)
    std::unordered_map<EdgeKey, std::vector<uint32_t>, EdgeKey::Hash> m_edge_to_faces;

    // Selection
    SelectMode m_select_mode = SelectMode::Vertex;
    std::unordered_set<uint32_t> m_sel_verts;
    std::unordered_set<EdgeKey, EdgeKey::Hash> m_sel_edges;
    std::unordered_set<uint32_t> m_sel_faces;

    // Undo/Redo
    struct Snapshot {
        std::vector<Vec3> vertices;
        std::vector<Triangle> tris;
        std::string name;
    };
    std::deque<Snapshot> m_undo_stack;
    std::deque<Snapshot> m_redo_stack;
    static constexpr size_t MAX_UNDO = 64;

    void take_snapshot(Snapshot& s);
    void restore_snapshot(const Snapshot& s);

    // Internal helpers
    void remove_face(uint32_t tri_idx);
    void remove_degenerate_triangles();
    void compact_unused_vertices();
};

} // namespace mechatron
