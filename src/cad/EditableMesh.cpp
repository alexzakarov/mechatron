#include "EditableMesh.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <set>

namespace mechatron {

// ============================================================================
// I/O
// ============================================================================

bool EditableMesh::from_meshdata(const MeshData& md) {
    if (md.vertices.empty() || md.triangles.empty()) return false;
    m_vertices = md.vertices;
    m_tris.clear();
    m_tris.reserve(md.triangles.size());
    for (const Triangle& t : md.triangles) {
        if (t.v0 >= m_vertices.size() || t.v1 >= m_vertices.size() || t.v2 >= m_vertices.size()) continue;
        if (t.v0 == t.v1 || t.v1 == t.v2 || t.v2 == t.v0) continue;
        m_tris.push_back(t);
    }
    if (m_tris.empty()) {
        m_vertices.clear();
        return false;
    }
    m_sel_verts.clear();
    m_sel_edges.clear();
    m_sel_faces.clear();
    rebuild_edge_map();
    return true;
}

MeshData EditableMesh::to_meshdata() const {
    MeshData md;
    md.vertices = m_vertices;
    md.triangles = m_tris;
    md.calculate_normals();
    return md;
}

// ============================================================================
// Edge topology
// ============================================================================

size_t EditableMesh::edge_count() const {
    // Count unique edges
    std::unordered_set<EdgeKey, EdgeKey::Hash> edges;
    for (const auto& t : m_tris) {
        edges.insert(EdgeKey(t.v0, t.v1));
        edges.insert(EdgeKey(t.v1, t.v2));
        edges.insert(EdgeKey(t.v2, t.v0));
    }
    return edges.size();
}

void EditableMesh::rebuild_edge_map() {
    m_edge_to_faces.clear();
    for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
        const auto& t = m_tris[i];
        m_edge_to_faces[EdgeKey(t.v0, t.v1)].push_back(i);
        m_edge_to_faces[EdgeKey(t.v1, t.v2)].push_back(i);
        m_edge_to_faces[EdgeKey(t.v2, t.v0)].push_back(i);
    }
}

std::vector<EdgeKey> EditableMesh::get_edges() const {
    std::vector<EdgeKey> edges;
    edges.reserve(m_edge_to_faces.size());
    for (const auto& [ek, _] : m_edge_to_faces) {
        edges.push_back(ek);
    }
    return edges;
}

std::vector<EdgeKey> EditableMesh::get_vertex_edges(uint32_t v) const {
    std::vector<EdgeKey> result;
    for (const auto& [ek, faces] : m_edge_to_faces) {
        if (ek.v0 == v || ek.v1 == v) result.push_back(ek);
    }
    return result;
}

std::vector<uint32_t> EditableMesh::get_vertex_faces(uint32_t v) const {
    std::unordered_set<uint32_t> face_set;
    for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
        const auto& t = m_tris[i];
        if (t.v0 == v || t.v1 == v || t.v2 == v) face_set.insert(i);
    }
    return std::vector<uint32_t>(face_set.begin(), face_set.end());
}

std::vector<uint32_t> EditableMesh::get_edge_faces(const EdgeKey& ek) const {
    auto it = m_edge_to_faces.find(ek);
    if (it != m_edge_to_faces.end()) return it->second;
    return {};
}

void EditableMesh::get_face_vertices(uint32_t tri_idx, uint32_t& a, uint32_t& b, uint32_t& c) const {
    if (tri_idx >= m_tris.size()) { a = b = c = 0; return; }
    a = m_tris[tri_idx].v0;
    b = m_tris[tri_idx].v1;
    c = m_tris[tri_idx].v2;
}

// ============================================================================
// Selection
// ============================================================================

void EditableMesh::clear_selection() {
    m_sel_verts.clear();
    m_sel_edges.clear();
    m_sel_faces.clear();
}

void EditableMesh::select_all() {
    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i)
                m_sel_verts.insert(i);
            break;
        case SelectMode::Edge:
            for (const auto& [ek, _] : m_edge_to_faces)
                m_sel_edges.insert(ek);
            break;
        case SelectMode::Face:
            for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i)
                m_sel_faces.insert(i);
            break;
    }
}

void EditableMesh::invert_selection() {
    switch (m_select_mode) {
        case SelectMode::Vertex: {
            std::unordered_set<uint32_t> new_sel;
            for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i) {
                if (!m_sel_verts.contains(i)) new_sel.insert(i);
            }
            m_sel_verts = std::move(new_sel);
            break;
        }
        case SelectMode::Edge: {
            std::unordered_set<EdgeKey, EdgeKey::Hash> new_sel;
            for (const auto& [ek, _] : m_edge_to_faces) {
                if (!m_sel_edges.contains(ek)) new_sel.insert(ek);
            }
            m_sel_edges = std::move(new_sel);
            break;
        }
        case SelectMode::Face: {
            std::unordered_set<uint32_t> new_sel;
            for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
                if (!m_sel_faces.contains(i)) new_sel.insert(i);
            }
            m_sel_faces = std::move(new_sel);
            break;
        }
    }
}

void EditableMesh::select_vertex(uint32_t idx, bool add) {
    if (idx >= m_vertices.size()) return;
    if (!add) m_sel_verts.clear();
    if (m_sel_verts.contains(idx)) m_sel_verts.erase(idx);
    else m_sel_verts.insert(idx);
}

void EditableMesh::deselect_vertex(uint32_t idx) {
    m_sel_verts.erase(idx);
}

bool EditableMesh::is_vertex_selected(uint32_t idx) const {
    return m_sel_verts.contains(idx);
}

void EditableMesh::select_edge(const EdgeKey& ek, bool add) {
    if (!add) m_sel_edges.clear();
    if (m_sel_edges.contains(ek)) m_sel_edges.erase(ek);
    else m_sel_edges.insert(ek);
}

void EditableMesh::deselect_edge(const EdgeKey& ek) {
    m_sel_edges.erase(ek);
}

bool EditableMesh::is_edge_selected(const EdgeKey& ek) const {
    return m_sel_edges.contains(ek);
}

void EditableMesh::select_face(uint32_t tri_idx, bool add) {
    if (tri_idx >= m_tris.size()) return;
    if (!add) m_sel_faces.clear();
    if (m_sel_faces.contains(tri_idx)) m_sel_faces.erase(tri_idx);
    else m_sel_faces.insert(tri_idx);
}

void EditableMesh::deselect_face(uint32_t tri_idx) {
    m_sel_faces.erase(tri_idx);
}

bool EditableMesh::is_face_selected(uint32_t tri_idx) const {
    return m_sel_faces.contains(tri_idx);
}

void EditableMesh::box_select(float x0, float y0, float x1, float y1,
                               const std::function<bool(const Vec3&, float&, float&)>& project_fn) {
    box_select(x0, y0, x1, y1, SelectionOp::Add, project_fn);
}

void EditableMesh::box_select(float x0, float y0, float x1, float y1, SelectionOp op,
                               const std::function<bool(const Vec3&, float&, float&)>& project_fn) {
    float xmin = std::min(x0, x1), xmax = std::max(x0, x1);
    float ymin = std::min(y0, y1), ymax = std::max(y0, y1);

    if (op == SelectionOp::Replace) {
        clear_selection();
    }

    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i) {
                float sx, sy;
                if (project_fn(m_vertices[i], sx, sy)) {
                    if (sx >= xmin && sx <= xmax && sy >= ymin && sy <= ymax) {
                        if (op == SelectionOp::Subtract) m_sel_verts.erase(i);
                        else m_sel_verts.insert(i);
                    }
                }
            }
            break;
        case SelectMode::Edge:
            for (const auto& [ek, _] : m_edge_to_faces) {
                Vec3 c = edge_center(ek);
                float sx, sy;
                if (project_fn(c, sx, sy)) {
                    if (sx >= xmin && sx <= xmax && sy >= ymin && sy <= ymax) {
                        if (op == SelectionOp::Subtract) m_sel_edges.erase(ek);
                        else m_sel_edges.insert(ek);
                    }
                }
            }
            break;
        case SelectMode::Face:
            for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
                Vec3 c = face_center(i);
                float sx, sy;
                if (project_fn(c, sx, sy)) {
                    if (sx >= xmin && sx <= xmax && sy >= ymin && sy <= ymax) {
                        if (op == SelectionOp::Subtract) m_sel_faces.erase(i);
                        else m_sel_faces.insert(i);
                    }
                }
            }
            break;
    }
}

void EditableMesh::circle_select(float cx, float cy, float radius, bool select,
                                  const std::function<bool(const Vec3&, float&, float&)>& project_fn) {
    circle_select(cx, cy, radius, select ? SelectionOp::Add : SelectionOp::Subtract, project_fn);
}

void EditableMesh::circle_select(float cx, float cy, float radius, SelectionOp op,
                                  const std::function<bool(const Vec3&, float&, float&)>& project_fn) {
    float r2 = radius * radius;
    if (op == SelectionOp::Replace) {
        clear_selection();
    }
    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i) {
                float sx, sy;
                if (project_fn(m_vertices[i], sx, sy)) {
                    float dx = sx - cx, dy = sy - cy;
                    if (dx*dx + dy*dy <= r2) {
                        if (op == SelectionOp::Subtract) m_sel_verts.erase(i);
                        else m_sel_verts.insert(i);
                    }
                }
            }
            break;
        case SelectMode::Edge:
            for (const auto& [ek, _] : m_edge_to_faces) {
                Vec3 c = edge_center(ek);
                float sx, sy;
                if (project_fn(c, sx, sy)) {
                    float dx = sx - cx, dy = sy - cy;
                    if (dx*dx + dy*dy <= r2) {
                        if (op == SelectionOp::Subtract) m_sel_edges.erase(ek);
                        else m_sel_edges.insert(ek);
                    }
                }
            }
            break;
        case SelectMode::Face:
            for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
                Vec3 c = face_center(i);
                float sx, sy;
                if (project_fn(c, sx, sy)) {
                    float dx = sx - cx, dy = sy - cy;
                    if (dx*dx + dy*dy <= r2) {
                        if (op == SelectionOp::Subtract) m_sel_faces.erase(i);
                        else m_sel_faces.insert(i);
                    }
                }
            }
            break;
    }
}

void EditableMesh::polygon_select(const std::vector<std::array<float, 2>>& polygon, SelectionOp op,
                                  const std::function<bool(const Vec3&, float&, float&)>& project_fn) {
    if (polygon.size() < 3) return;

    auto inside = [&](float x, float y) {
        bool result = false;
        size_t j = polygon.size() - 1;
        for (size_t i = 0; i < polygon.size(); ++i) {
            const float xi = polygon[i][0];
            const float yi = polygon[i][1];
            const float xj = polygon[j][0];
            const float yj = polygon[j][1];
            const bool crosses = ((yi > y) != (yj > y)) &&
                (x < (xj - xi) * (y - yi) / ((yj - yi) == 0.0f ? 1e-6f : (yj - yi)) + xi);
            if (crosses) result = !result;
            j = i;
        }
        return result;
    };

    if (op == SelectionOp::Replace) {
        clear_selection();
    }

    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i) {
                float sx, sy;
                if (project_fn(m_vertices[i], sx, sy) && inside(sx, sy)) {
                    if (op == SelectionOp::Subtract) m_sel_verts.erase(i);
                    else m_sel_verts.insert(i);
                }
            }
            break;
        case SelectMode::Edge:
            for (const auto& [ek, _] : m_edge_to_faces) {
                Vec3 c = edge_center(ek);
                float sx, sy;
                if (project_fn(c, sx, sy) && inside(sx, sy)) {
                    if (op == SelectionOp::Subtract) m_sel_edges.erase(ek);
                    else m_sel_edges.insert(ek);
                }
            }
            break;
        case SelectMode::Face:
            for (uint32_t i = 0; i < (uint32_t)m_tris.size(); ++i) {
                Vec3 c = face_center(i);
                float sx, sy;
                if (project_fn(c, sx, sy) && inside(sx, sy)) {
                    if (op == SelectionOp::Subtract) m_sel_faces.erase(i);
                    else m_sel_faces.insert(i);
                }
            }
            break;
    }
}

// ============================================================================
// Basic edit ops
// ============================================================================

bool EditableMesh::set_vertex_pos(uint32_t idx, const Vec3& p) {
    if (idx >= m_vertices.size()) return false;
    m_vertices[idx] = p;
    return true;
}

void EditableMesh::translate_selected(const Vec3& delta) {
    // Collect all vertices affected by the current selection
    std::unordered_set<uint32_t> affected;
    switch (m_select_mode) {
        case SelectMode::Vertex:
            affected = m_sel_verts;
            break;
        case SelectMode::Edge:
            for (const auto& ek : m_sel_edges) {
                affected.insert(ek.v0);
                affected.insert(ek.v1);
            }
            break;
        case SelectMode::Face:
            for (uint32_t fi : m_sel_faces) {
                if (fi < m_tris.size()) {
                    affected.insert(m_tris[fi].v0);
                    affected.insert(m_tris[fi].v1);
                    affected.insert(m_tris[fi].v2);
                }
            }
            break;
    }
    for (uint32_t idx : affected) {
        if (idx < m_vertices.size()) m_vertices[idx] = m_vertices[idx] + delta;
    }
}

void EditableMesh::rotate_selected(const Vec3& pivot, const Vec3& axis, float angle_deg) {
    float rad = angle_deg * (float)M_PI / 180.0f;
    float c = std::cos(rad), s = std::sin(rad);
    Vec3 ax = axis.normalized();

    auto rotate_point = [&](const Vec3& p) -> Vec3 {
        Vec3 d = p - pivot;
        return pivot + d * c + ax.cross(d) * s + ax * (ax.dot(d)) * (1 - c);
    };

    std::unordered_set<uint32_t> affected;
    switch (m_select_mode) {
        case SelectMode::Vertex: affected = m_sel_verts; break;
        case SelectMode::Edge:
            for (const auto& ek : m_sel_edges) { affected.insert(ek.v0); affected.insert(ek.v1); }
            break;
        case SelectMode::Face:
            for (uint32_t fi : m_sel_faces) {
                if (fi < m_tris.size()) {
                    affected.insert(m_tris[fi].v0); affected.insert(m_tris[fi].v1); affected.insert(m_tris[fi].v2);
                }
            }
            break;
    }
    for (uint32_t idx : affected) {
        if (idx < m_vertices.size()) m_vertices[idx] = rotate_point(m_vertices[idx]);
    }
}

void EditableMesh::scale_selected(const Vec3& pivot, const Vec3& factor) {
    std::unordered_set<uint32_t> affected;
    switch (m_select_mode) {
        case SelectMode::Vertex: affected = m_sel_verts; break;
        case SelectMode::Edge:
            for (const auto& ek : m_sel_edges) { affected.insert(ek.v0); affected.insert(ek.v1); }
            break;
        case SelectMode::Face:
            for (uint32_t fi : m_sel_faces) {
                if (fi < m_tris.size()) {
                    affected.insert(m_tris[fi].v0); affected.insert(m_tris[fi].v1); affected.insert(m_tris[fi].v2);
                }
            }
            break;
    }
    for (uint32_t idx : affected) {
        if (idx < m_vertices.size()) {
            Vec3 d = m_vertices[idx] - pivot;
            m_vertices[idx] = pivot + Vec3{d.x * factor.x, d.y * factor.y, d.z * factor.z};
        }
    }
}

bool EditableMesh::add_vertex(const Vec3& p, uint32_t* out_idx) {
    m_vertices.push_back(p);
    if (out_idx) *out_idx = static_cast<uint32_t>(m_vertices.size() - 1);
    return true;
}

bool EditableMesh::add_triangle(uint32_t a, uint32_t b, uint32_t c) {
    if (a >= m_vertices.size() || b >= m_vertices.size() || c >= m_vertices.size()) return false;
    if (a == b || b == c || a == c) return false; // degenerate
    m_tris.push_back(Triangle{a, b, c});
    rebuild_edge_map();
    return true;
}

// ============================================================================
// Delete operations
// ============================================================================

void EditableMesh::remove_face(uint32_t tri_idx) {
    if (tri_idx >= m_tris.size()) return;
    // Swap with last and pop
    if (tri_idx < m_tris.size() - 1) {
        m_tris[tri_idx] = m_tris.back();
        // Update selection index
        if (m_sel_faces.contains((uint32_t)m_tris.size() - 1)) {
            m_sel_faces.erase((uint32_t)m_tris.size() - 1);
            m_sel_faces.insert(tri_idx);
        }
    }
    m_tris.pop_back();
}

void EditableMesh::remove_degenerate_triangles() {
    size_t write = 0;
    for (size_t i = 0; i < m_tris.size(); ++i) {
        const auto& t = m_tris[i];
        if (t.v0 != t.v1 && t.v1 != t.v2 && t.v0 != t.v2 &&
            t.v0 < m_vertices.size() && t.v1 < m_vertices.size() && t.v2 < m_vertices.size()) {
            m_tris[write++] = m_tris[i];
        }
    }
    m_tris.resize(write);
}

void EditableMesh::compact_unused_vertices() {
    if (m_vertices.empty()) return;
    remove_degenerate_triangles();

    std::vector<bool> used(m_vertices.size(), false);
    for (const Triangle& t : m_tris) {
        if (t.v0 < used.size()) used[t.v0] = true;
        if (t.v1 < used.size()) used[t.v1] = true;
        if (t.v2 < used.size()) used[t.v2] = true;
    }

    std::vector<uint32_t> remap(m_vertices.size(), UINT32_MAX);
    std::vector<Vec3> compacted;
    compacted.reserve(m_vertices.size());
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_vertices.size()); ++i) {
        if (!used[i]) continue;
        remap[i] = static_cast<uint32_t>(compacted.size());
        compacted.push_back(m_vertices[i]);
    }

    for (Triangle& t : m_tris) {
        t.v0 = remap[t.v0];
        t.v1 = remap[t.v1];
        t.v2 = remap[t.v2];
    }

    m_vertices = std::move(compacted);
    clear_selection();
    rebuild_edge_map();
}

void EditableMesh::delete_loose_vertices() {
    compact_unused_vertices();
}

void EditableMesh::delete_selected_vertices() {
    if (m_sel_verts.empty()) return;

    // Mark vertices to delete
    std::vector<bool> deleted(m_vertices.size(), false);
    for (uint32_t v : m_sel_verts) {
        if (v < m_vertices.size()) deleted[v] = true;
    }

    // Remove triangles that reference deleted vertices
    size_t write = 0;
    for (size_t i = 0; i < m_tris.size(); ++i) {
        if (!deleted[m_tris[i].v0] && !deleted[m_tris[i].v1] && !deleted[m_tris[i].v2]) {
            m_tris[write++] = m_tris[i];
        }
    }
    m_tris.resize(write);

    // Compact vertices: build remapping
    std::vector<uint32_t> remap(m_vertices.size(), UINT32_MAX);
    uint32_t new_count = 0;
    for (uint32_t i = 0; i < (uint32_t)m_vertices.size(); ++i) {
        if (!deleted[i]) {
            remap[i] = new_count;
            m_vertices[new_count] = m_vertices[i];
            new_count++;
        }
    }
    m_vertices.resize(new_count);

    // Remap triangle indices
    for (auto& t : m_tris) {
        t.v0 = remap[t.v0];
        t.v1 = remap[t.v1];
        t.v2 = remap[t.v2];
    }

    clear_selection();
    rebuild_edge_map();
}

void EditableMesh::delete_selected_edges() {
    if (m_sel_edges.empty()) return;

    // Remove triangles that contain any selected edge
    size_t write = 0;
    for (size_t i = 0; i < m_tris.size(); ++i) {
        const auto& t = m_tris[i];
        EdgeKey e0(t.v0, t.v1), e1(t.v1, t.v2), e2(t.v2, t.v0);
        if (!m_sel_edges.contains(e0) && !m_sel_edges.contains(e1) && !m_sel_edges.contains(e2)) {
            m_tris[write++] = m_tris[i];
        }
    }
    m_tris.resize(write);

    clear_selection();
    rebuild_edge_map();
}

void EditableMesh::delete_selected_faces() {
    if (m_sel_faces.empty()) return;

    // Sort selected faces in descending order for safe removal
    std::vector<uint32_t> sorted(m_sel_faces.begin(), m_sel_faces.end());
    std::sort(sorted.begin(), sorted.end(), std::greater<uint32_t>());

    for (uint32_t fi : sorted) {
        remove_face(fi);
    }

    clear_selection();
    rebuild_edge_map();
}

void EditableMesh::delete_selected() {
    switch (m_select_mode) {
        case SelectMode::Vertex: delete_selected_vertices(); break;
        case SelectMode::Edge:   delete_selected_edges(); break;
        case SelectMode::Face:   delete_selected_faces(); break;
    }
}

// ============================================================================
// Mesh editing operations
// ============================================================================

Vec3 EditableMesh::face_normal(uint32_t tri_idx) const {
    if (tri_idx >= m_tris.size()) return {0, 1, 0};
    const auto& t = m_tris[tri_idx];
    Vec3 a = m_vertices[t.v0];
    Vec3 b = m_vertices[t.v1];
    Vec3 c = m_vertices[t.v2];
    Vec3 n = (b - a).cross(c - a);
    float len = n.length();
    return len > 1e-9f ? n / len : Vec3{0, 1, 0};
}

Vec3 EditableMesh::face_center(uint32_t tri_idx) const {
    if (tri_idx >= m_tris.size()) return {};
    const auto& t = m_tris[tri_idx];
    return (m_vertices[t.v0] + m_vertices[t.v1] + m_vertices[t.v2]) * (1.0f / 3.0f);
}

Vec3 EditableMesh::edge_center(const EdgeKey& ek) const {
    if (ek.v0 >= m_vertices.size() || ek.v1 >= m_vertices.size()) return {};
    return (m_vertices[ek.v0] + m_vertices[ek.v1]) * 0.5f;
}

void EditableMesh::get_selection_center(Vec3& out) const {
    out = {0, 0, 0};
    int count = 0;
    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (uint32_t v : m_sel_verts) {
                if (v < m_vertices.size()) {
                    out = out + m_vertices[v];
                    count++;
                }
            }
            break;
        case SelectMode::Edge:
            for (const auto& ek : m_sel_edges) { out = out + edge_center(ek); count++; }
            break;
        case SelectMode::Face:
            for (uint32_t fi : m_sel_faces) {
                if (fi < m_tris.size()) {
                    out = out + face_center(fi);
                    count++;
                }
            }
            break;
    }
    if (count > 0) out = out * (1.0f / (float)count);
}

std::vector<uint32_t> EditableMesh::extrude_faces(const std::vector<uint32_t>& face_indices, float distance) {
    if (face_indices.empty()) return {};

    // Collect all boundary vertices of the face set
    std::unordered_set<uint32_t> face_set(face_indices.begin(), face_indices.end());
    std::unordered_map<uint32_t, uint32_t> vert_to_new;

    // Duplicate vertices of selected faces
    for (uint32_t fi : face_indices) {
        if (fi >= m_tris.size()) continue;
        const auto& t = m_tris[fi];
        for (int i = 0; i < 3; ++i) {
            uint32_t v = (i == 0) ? t.v0 : (i == 1) ? t.v1 : t.v2;
            if (!vert_to_new.contains(v)) {
                Vec3 n = face_normal(fi);
                uint32_t nv;
                add_vertex(m_vertices[v] + n * distance, &nv);
                vert_to_new[v] = nv;
            }
        }
    }

    // For each boundary edge (edge shared with a non-selected face), create side faces
    for (uint32_t fi : face_indices) {
        if (fi >= m_tris.size()) continue;
        const auto& t = m_tris[fi];
        EdgeKey edges[3] = { EdgeKey(t.v0, t.v1), EdgeKey(t.v1, t.v2), EdgeKey(t.v2, t.v0) };
        uint32_t verts[3] = { t.v0, t.v1, t.v2 };

        for (int e = 0; e < 3; ++e) {
            // Check if this edge is boundary (not shared with another selected face)
            bool is_boundary = true;
            auto it = m_edge_to_faces.find(edges[e]);
            if (it != m_edge_to_faces.end()) {
                for (uint32_t adj : it->second) {
                    if (adj != fi && face_set.contains(adj)) {
                        is_boundary = false;
                        break;
                    }
                }
            }
            if (!is_boundary) continue;

            uint32_t v0 = verts[e];
            uint32_t v1 = verts[(e + 1) % 3];
            uint32_t n0 = vert_to_new[v0];
            uint32_t n1 = vert_to_new[v1];

            add_triangle(v0, n1, n0);
            add_triangle(v0, v1, n1);
        }
    }

    // Replace old face vertices with new vertices
    for (uint32_t fi : face_indices) {
        if (fi >= m_tris.size()) continue;
        auto& t = m_tris[fi];
        t.v0 = vert_to_new[t.v0];
        t.v1 = vert_to_new[t.v1];
        t.v2 = vert_to_new[t.v2];
    }

    rebuild_edge_map();

    // Return the extruded face indices (same indices, vertices updated)
    return face_indices;
}

std::vector<uint32_t> EditableMesh::extrude_faces_individual(const std::vector<uint32_t>& face_indices, float distance) {
    std::vector<uint32_t> new_faces;
    for (uint32_t fi : face_indices) {
        if (fi >= m_tris.size()) continue;
        const auto& t = m_tris[fi];
        Vec3 n = face_normal(fi);

        uint32_t a, b, c;
        add_vertex(m_vertices[t.v0] + n * distance, &a);
        add_vertex(m_vertices[t.v1] + n * distance, &b);
        add_vertex(m_vertices[t.v2] + n * distance, &c);

        // Top face
        add_triangle(a, b, c);

        // Side faces
        add_triangle(t.v0, t.v1, b);
        add_triangle(t.v0, b, a);
        add_triangle(t.v1, t.v2, c);
        add_triangle(t.v1, c, b);
        add_triangle(t.v2, t.v0, a);
        add_triangle(t.v2, a, c);

        new_faces.push_back((uint32_t)m_tris.size() - 7);
    }
    rebuild_edge_map();
    return new_faces;
}

std::vector<uint32_t> EditableMesh::extrude_edges(const std::vector<EdgeKey>& edges, float distance) {
    std::vector<uint32_t> new_faces;
    for (const auto& ek : edges) {
        if (ek.v0 >= m_vertices.size() || ek.v1 >= m_vertices.size()) continue;

        // Average normal of adjacent faces, or use up
        Vec3 n{0, 1, 0};
        auto it = m_edge_to_faces.find(ek);
        if (it != m_edge_to_faces.end() && !it->second.empty()) {
            n = {0, 0, 0};
            for (uint32_t fi : it->second) n = n + face_normal(fi);
            n = n.normalized();
            if (n.length() < 0.5f) n = {0, 1, 0};
        }

        uint32_t a, b;
        add_vertex(m_vertices[ek.v0] + n * distance, &a);
        add_vertex(m_vertices[ek.v1] + n * distance, &b);

        add_triangle(ek.v0, ek.v1, b);
        add_triangle(ek.v0, b, a);

        new_faces.push_back((uint32_t)m_tris.size() - 2);
    }
    rebuild_edge_map();
    return new_faces;
}

void EditableMesh::extrude_selected(float distance) {
    switch (m_select_mode) {
        case SelectMode::Face: {
            auto sel = std::vector<uint32_t>(m_sel_faces.begin(), m_sel_faces.end());
            extrude_faces(sel, distance);
            break;
        }
        case SelectMode::Edge: {
            auto sel = std::vector<EdgeKey>(m_sel_edges.begin(), m_sel_edges.end());
            extrude_edges(sel, distance);
            break;
        }
        case SelectMode::Vertex:
            // Extrude vertices = duplicate and connect with edges (as degenerate thin faces)
            for (uint32_t v : m_sel_verts) {
                if (v >= m_vertices.size()) continue;
                Vec3 n{0, 1, 0}; // Default direction
                // Use average of adjacent face normals
                auto vf = get_vertex_faces(v);
                if (!vf.empty()) {
                    Vec3 avg{0, 0, 0};
                    for (uint32_t fi : vf) avg = avg + face_normal(fi);
                    n = avg.normalized();
                    if (n.length() < 0.5f) n = {0, 1, 0};
                }
                uint32_t nv;
                add_vertex(m_vertices[v] + n * distance, &nv);
                // Create a small diamond-like shape around the vertex
                // For simplicity, just create a tiny triangle
                // In practice this is more complex
            }
            break;
    }
}

std::vector<uint32_t> EditableMesh::inset_faces(const std::vector<uint32_t>& face_indices, float thickness, float depth) {
    if (face_indices.empty()) return {};

    std::vector<uint32_t> new_inner_faces;

    for (uint32_t fi : face_indices) {
        if (fi >= m_tris.size()) continue;
        const Triangle t = m_tris[fi];
        Vec3 fc = face_center(fi);
        Vec3 n = face_normal(fi);

        // Create inset vertices
        uint32_t a, b, c;
        Vec3 va = m_vertices[t.v0] + (fc - m_vertices[t.v0]) * thickness + n * depth;
        Vec3 vb = m_vertices[t.v1] + (fc - m_vertices[t.v1]) * thickness + n * depth;
        Vec3 vc = m_vertices[t.v2] + (fc - m_vertices[t.v2]) * thickness + n * depth;
        add_vertex(va, &a);
        add_vertex(vb, &b);
        add_vertex(vc, &c);

        // Inner face
        add_triangle(a, b, c);
        new_inner_faces.push_back((uint32_t)m_tris.size() - 1);

        // Side faces connecting outer to inner
        add_triangle(t.v0, t.v1, b);
        add_triangle(t.v0, b, a);
        add_triangle(t.v1, t.v2, c);
        add_triangle(t.v1, c, b);
        add_triangle(t.v2, t.v0, a);
        add_triangle(t.v2, a, c);
    }

    rebuild_edge_map();
    return new_inner_faces;
}

void EditableMesh::bevel_edges(const std::vector<EdgeKey>& edges, float amount, int segments) {
    if (edges.empty()) return;
    segments = std::max(1, segments);

    for (const auto& ek : edges) {
        if (ek.v0 >= m_vertices.size() || ek.v1 >= m_vertices.size()) continue;
        Vec3 p0 = m_vertices[ek.v0];
        Vec3 p1 = m_vertices[ek.v1];
        Vec3 dir = (p1 - p0).normalized();
        Vec3 mid = (p0 + p1) * 0.5f;

        // Create bevel vertices along the edge
        std::vector<uint32_t> bevel_verts;
        for (int s = 0; s <= segments; ++s) {
            float t = (float)s / (float)segments;
            Vec3 p = p0 + (p1 - p0) * t;

            // Offset perpendicular to edge
            auto adj = get_edge_faces(ek);
            Vec3 offset_dir{0, 0, 0};
            for (uint32_t fi : adj) offset_dir = offset_dir + face_normal(fi);
            offset_dir = offset_dir.normalized();
            if (offset_dir.length() < 0.5f) offset_dir = Vec3{0, 1, 0};

            Vec3 bp = p + offset_dir * amount;
            uint32_t nv;
            add_vertex(bp, &nv);
            bevel_verts.push_back(nv);
        }

        // Create faces between original and bevel vertices
        auto adj = get_edge_faces(ek);
        for (uint32_t fi : adj) {
            if (fi >= m_tris.size()) continue;
            const auto& tri = m_tris[fi];
            // Find which edge of the triangle matches
            bool edge_found = false;
            uint32_t ev0 = ek.v0, ev1 = ek.v1;
            uint32_t other_v = tri.v0;
            if ((tri.v0 == ev0 && tri.v1 == ev1) || (tri.v0 == ev1 && tri.v1 == ev0)) {
                other_v = tri.v2;
            } else if ((tri.v1 == ev0 && tri.v2 == ev1) || (tri.v1 == ev1 && tri.v2 == ev0)) {
                other_v = tri.v0;
            } else if ((tri.v2 == ev0 && tri.v0 == ev1) || (tri.v2 == ev1 && tri.v0 == ev0)) {
                other_v = tri.v1;
            } else {
                continue;
            }

            for (int s = 0; s < segments; ++s) {
                add_triangle(other_v, bevel_verts[s], bevel_verts[s + 1]);
            }
        }
    }

    rebuild_edge_map();
}

int EditableMesh::loop_cut(const EdgeKey& start_edge, float position) {
    position = std::clamp(position, 0.0f, 1.0f);

    // Trace a loop of edges across the mesh
    std::vector<EdgeKey> loop;
    EdgeKey current = start_edge;

    // Simple loop tracing: follow the "next" edge across adjacent faces
    std::unordered_set<EdgeKey, EdgeKey::Hash> visited;
    for (int safety = 0; safety < 1000; ++safety) {
        if (visited.contains(current)) break;
        visited.insert(current);
        loop.push_back(current);

        // Find the next edge in the loop
        auto adj = get_edge_faces(current);
        if (adj.empty()) break;

        // Get face, find the "opposite" edge
        bool found_next = false;
        for (uint32_t fi : adj) {
            if (fi >= m_tris.size()) continue;
            const auto& t = m_tris[fi];
            // Get edges of this face
            EdgeKey edges[3] = { EdgeKey(t.v0, t.v1), EdgeKey(t.v1, t.v2), EdgeKey(t.v2, t.v0) };
            for (int e = 0; e < 3; ++e) {
                if (edges[e] == current) {
                    // The opposite edge in a triangle shares the vertex opposite to current edge
                    // Find vertex not in current edge
                    uint32_t opposite = t.v0;
                    if (opposite == current.v0 || opposite == current.v1) opposite = t.v1;
                    if (opposite == current.v0 || opposite == current.v1) opposite = t.v2;

                    // The two edges incident to 'opposite'
                    for (int e2 = 0; e2 < 3; ++e2) {
                        if (e2 == e) continue;
                        if (edges[e2].v0 == opposite || edges[e2].v1 == opposite) {
                            if (!visited.contains(edges[e2])) {
                                current = edges[e2];
                                found_next = true;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if (found_next) break;
        }
        if (!found_next) break;
    }

    if (loop.empty()) return 0;

    // Insert new vertices on each edge of the loop
    std::vector<uint32_t> new_verts;
    for (const auto& ek : loop) {
        Vec3 p = m_vertices[ek.v0] * (1 - position) + m_vertices[ek.v1] * position;
        uint32_t nv;
        add_vertex(p, &nv);
        new_verts.push_back(nv);
    }

    // Split each triangle that intersects the loop
    // For each loop edge, find adjacent faces and split them
    for (size_t i = 0; i < loop.size(); ++i) {
        const auto& ek = loop[i];
        uint32_t nv = new_verts[i];
        auto adj = get_edge_faces(ek);

        for (uint32_t fi : adj) {
            if (fi >= m_tris.size()) continue;
            auto& t = m_tris[fi];

            // Find which edge of the triangle this is
            uint32_t ev0 = ek.v0, ev1 = ek.v1;
            uint32_t other;
            int edge_idx = -1;

            if (EdgeKey(t.v0, t.v1) == ek) { other = t.v2; edge_idx = 0; }
            else if (EdgeKey(t.v1, t.v2) == ek) { other = t.v0; edge_idx = 1; }
            else if (EdgeKey(t.v2, t.v0) == ek) { other = t.v1; edge_idx = 2; }
            else continue;

            // Replace original triangle with two sub-triangles
            uint32_t a = (edge_idx == 0) ? t.v0 : (edge_idx == 1) ? t.v1 : t.v2;
            uint32_t b = (edge_idx == 0) ? t.v1 : (edge_idx == 1) ? t.v2 : t.v0;

            // Triangle 1: a, nv, other
            t = Triangle{a, nv, other};
            // Triangle 2: nv, b, other
            add_triangle(nv, b, other);
        }
    }

    rebuild_edge_map();
    return (int)new_verts.size();
}

void EditableMesh::subdivide_edges(const std::vector<EdgeKey>& edges, int cuts) {
    if (edges.empty() || cuts <= 0) return;

    std::unordered_map<EdgeKey, std::vector<uint32_t>, EdgeKey::Hash> edge_new_verts;

    for (const auto& ek : edges) {
        if (ek.v0 >= m_vertices.size() || ek.v1 >= m_vertices.size()) continue;
        std::vector<uint32_t> pts;
        pts.push_back(ek.v0);
        for (int c = 1; c <= cuts; ++c) {
            float t = (float)c / (float)(cuts + 1);
            Vec3 p = m_vertices[ek.v0] * (1 - t) + m_vertices[ek.v1] * t;
            uint32_t nv;
            add_vertex(p, &nv);
            pts.push_back(nv);
        }
        pts.push_back(ek.v1);
        edge_new_verts[ek] = std::move(pts);
    }

    // For each face, if any of its edges are subdivided, split the face
    // This is simplified: we just split the triangles along subdivided edges
    for (auto& [ek, pts] : edge_new_verts) {
        auto adj = get_edge_faces(ek);
        for (uint32_t fi : adj) {
            if (fi >= m_tris.size()) continue;
            auto& t = m_tris[fi];

            // Find the edge and the opposite vertex
            uint32_t other;
            if (EdgeKey(t.v0, t.v1) == ek) other = t.v2;
            else if (EdgeKey(t.v1, t.v2) == ek) other = t.v0;
            else if (EdgeKey(t.v2, t.v0) == ek) other = t.v1;
            else continue;

            // Replace with fan of triangles
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                if (i == 0) {
                    t = Triangle{pts[i], pts[i + 1], other};
                } else {
                    add_triangle(pts[i], pts[i + 1], other);
                }
            }
        }
    }

    rebuild_edge_map();
}

uint32_t EditableMesh::merge_selected_vertices_to_center() {
    if (m_sel_verts.size() < 2) return m_sel_verts.empty() ? 0 : *m_sel_verts.begin();

    // Calculate center
    Vec3 center{0, 0, 0};
    for (uint32_t v : m_sel_verts) center = center + m_vertices[v];
    center = center * (1.0f / (float)m_sel_verts.size());

    // Use first selected vertex as the target
    uint32_t target = *m_sel_verts.begin();
    m_vertices[target] = center;

    // Remap all other selected vertices to target
    std::unordered_set<uint32_t> to_merge;
    for (auto it = m_sel_verts.begin(); it != m_sel_verts.end(); ++it) {
        if (*it != target) to_merge.insert(*it);
    }

    // Remap triangles
    for (auto& t : m_tris) {
        if (to_merge.contains(t.v0)) t.v0 = target;
        if (to_merge.contains(t.v1)) t.v1 = target;
        if (to_merge.contains(t.v2)) t.v2 = target;
    }

    remove_degenerate_triangles();
    clear_selection();
    m_sel_verts.insert(target);
    rebuild_edge_map();
    return target;
}

uint32_t EditableMesh::merge_selected_vertices_to_first() {
    if (m_sel_verts.size() < 2) return m_sel_verts.empty() ? 0 : *m_sel_verts.begin();

    uint32_t target = *m_sel_verts.begin();
    std::unordered_set<uint32_t> to_merge;
    for (auto it = m_sel_verts.begin(); it != m_sel_verts.end(); ++it) {
        if (*it != target) to_merge.insert(*it);
    }

    for (auto& t : m_tris) {
        if (to_merge.contains(t.v0)) t.v0 = target;
        if (to_merge.contains(t.v1)) t.v1 = target;
        if (to_merge.contains(t.v2)) t.v2 = target;
    }

    remove_degenerate_triangles();
    clear_selection();
    m_sel_verts.insert(target);
    rebuild_edge_map();
    return target;
}

void EditableMesh::flip_selected_normals() {
    for (uint32_t fi : m_sel_faces) {
        if (fi >= m_tris.size()) continue;
        auto& t = m_tris[fi];
        std::swap(t.v0, t.v2); // Flip winding
    }
}

void EditableMesh::recalculate_normals() {
    // Make normals consistent: flood fill from first face
    if (m_tris.empty()) return;

    rebuild_edge_map();

    // Build adjacency
    std::vector<bool> visited(m_tris.size(), false);
    std::vector<int> queue;
    queue.push_back(0);
    visited[0] = true;

    while (!queue.empty()) {
        int fi = queue.back();
        queue.pop_back();
        const auto& t = m_tris[fi];
        EdgeKey edges[3] = { EdgeKey(t.v0, t.v1), EdgeKey(t.v1, t.v2), EdgeKey(t.v2, t.v0) };

        for (int e = 0; e < 3; ++e) {
            auto adj = get_edge_faces(edges[e]);
            for (uint32_t adj_fi : adj) {
                if (visited[adj_fi]) continue;

                // Check if normals are consistent
                const auto& at = m_tris[adj_fi];
                Vec3 n1 = face_normal(fi);
                Vec3 n2 = face_normal(adj_fi);

                // If dot product is negative, flip the adjacent face
                if (n1.dot(n2) < 0) {
                    std::swap(m_tris[adj_fi].v0, m_tris[adj_fi].v2);
                }

                visited[adj_fi] = true;
                queue.push_back(adj_fi);
            }
        }
    }

    rebuild_edge_map();
}

void EditableMesh::duplicate_selected() {
    std::unordered_map<uint32_t, uint32_t> old_to_new;
    const auto selected_edges = m_sel_edges;
    const auto selected_faces = m_sel_faces;
    std::vector<uint32_t> new_faces;

    switch (m_select_mode) {
        case SelectMode::Vertex: {
            for (uint32_t v : m_sel_verts) {
                uint32_t nv;
                add_vertex(m_vertices[v], &nv);
                old_to_new[v] = nv;
            }
            break;
        }
        case SelectMode::Face: {
            // Collect all vertices of selected faces
            std::unordered_set<uint32_t> face_verts;
            for (uint32_t fi : m_sel_faces) {
                if (fi >= m_tris.size()) continue;
                face_verts.insert(m_tris[fi].v0);
                face_verts.insert(m_tris[fi].v1);
                face_verts.insert(m_tris[fi].v2);
            }
            for (uint32_t v : face_verts) {
                uint32_t nv;
                add_vertex(m_vertices[v], &nv);
                old_to_new[v] = nv;
            }
            // Duplicate faces
            for (uint32_t fi : selected_faces) {
                if (fi >= m_tris.size()) continue;
                const auto& t = m_tris[fi];
                uint32_t new_face = static_cast<uint32_t>(m_tris.size());
                add_triangle(old_to_new[t.v0], old_to_new[t.v1], old_to_new[t.v2]);
                new_faces.push_back(new_face);
            }
            break;
        }
        case SelectMode::Edge: {
            // Collect vertices
            std::unordered_set<uint32_t> edge_verts;
            for (const auto& ek : m_sel_edges) {
                edge_verts.insert(ek.v0);
                edge_verts.insert(ek.v1);
            }
            for (uint32_t v : edge_verts) {
                uint32_t nv;
                add_vertex(m_vertices[v], &nv);
                old_to_new[v] = nv;
            }
            // Duplicate faces that contain selected edges
            std::set<uint32_t> faces_to_dup;
            for (const auto& ek : m_sel_edges) {
                for (uint32_t fi : get_edge_faces(ek)) faces_to_dup.insert(fi);
            }
            for (uint32_t fi : faces_to_dup) {
                if (fi >= m_tris.size()) continue;
                const auto& t = m_tris[fi];
                add_triangle(
                    old_to_new.count(t.v0) ? old_to_new[t.v0] : t.v0,
                    old_to_new.count(t.v1) ? old_to_new[t.v1] : t.v1,
                    old_to_new.count(t.v2) ? old_to_new[t.v2] : t.v2
                );
            }
            break;
        }
    }

    // Update selection to new elements
    clear_selection();
    switch (m_select_mode) {
        case SelectMode::Vertex:
            for (auto& [o, n] : old_to_new) m_sel_verts.insert(n);
            break;
        case SelectMode::Edge:
            for (const auto& ek : selected_edges) {
                if (old_to_new.count(ek.v0) && old_to_new.count(ek.v1))
                    m_sel_edges.insert(EdgeKey(old_to_new[ek.v0], old_to_new[ek.v1]));
            }
            break;
        case SelectMode::Face:
            for (uint32_t fi : new_faces) m_sel_faces.insert(fi);
            break;
    }

    rebuild_edge_map();
}

// ============================================================================
// Undo/Redo
// ============================================================================

void EditableMesh::take_snapshot(Snapshot& s) {
    s.vertices = m_vertices;
    s.tris = m_tris;
}

void EditableMesh::restore_snapshot(const Snapshot& s) {
    m_vertices = s.vertices;
    m_tris = s.tris;
    clear_selection();
    rebuild_edge_map();
}

void EditableMesh::push_undo(const std::string& name) {
    Snapshot s;
    take_snapshot(s);
    s.name = name;
    m_undo_stack.push_back(std::move(s));
    if (m_undo_stack.size() > MAX_UNDO) m_undo_stack.pop_front();
    m_redo_stack.clear();
}

bool EditableMesh::undo() {
    if (m_undo_stack.empty()) return false;
    Snapshot current;
    take_snapshot(current);
    current.name = m_undo_stack.back().name;
    m_redo_stack.push_back(std::move(current));
    restore_snapshot(m_undo_stack.back());
    m_undo_stack.pop_back();
    return true;
}

bool EditableMesh::redo() {
    if (m_redo_stack.empty()) return false;
    Snapshot current;
    take_snapshot(current);
    m_undo_stack.push_back(std::move(current));
    restore_snapshot(m_redo_stack.back());
    m_redo_stack.pop_back();
    return true;
}

bool EditableMesh::can_undo() const { return !m_undo_stack.empty(); }
bool EditableMesh::can_redo() const { return !m_redo_stack.empty(); }

std::string EditableMesh::undo_name() const {
    return m_undo_stack.empty() ? "" : m_undo_stack.back().name;
}

std::string EditableMesh::redo_name() const {
    return m_redo_stack.empty() ? "" : m_redo_stack.back().name;
}

void EditableMesh::clear_undo_stack() {
    m_undo_stack.clear();
    m_redo_stack.clear();
}

} // namespace mechatron
