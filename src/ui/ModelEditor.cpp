#include "ModelEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "core/Component.hpp"
#include "core/SystemConfig.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <filesystem>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>
#include "renderer/Shader.hpp"
#include "renderer/Mesh.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <set>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <shlobj.h>
#include <commdlg.h>
#endif

namespace mechatron {

// ============================================================================
// Static helpers
// ============================================================================
static bool load_asset_mesh_by_id(ModelAssetLibrary& lib, CADKernel& cad,
                                   const std::string& asset_id, const std::string& scope,
                                   MeshData& out) {
    return lib.load_asset_mesh(cad, asset_id, scope, out, nullptr);
}

static std::string trim_trailing_newlines(std::string text) {
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
        text.pop_back();
    }
    return text;
}

#ifndef _WIN32
static std::string run_dialog_command(const char* command) {
    std::array<char, 512> buffer{};
    std::string result;
    FILE* pipe = popen(command, "r");
    if (!pipe) return {};
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
        result += buffer.data();
    }
    pclose(pipe);
    return trim_trailing_newlines(result);
}
#endif

static std::string choose_export_folder() {
#ifdef _WIN32
    BROWSEINFOA bi{};
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    bi.lpszTitle = "Select export folder";
    PIDLIST_ABSOLUTE pidl = SHBrowseForFolderA(&bi);
    if (!pidl) return {};

    char path[MAX_PATH] = {};
    const bool ok = SHGetPathFromIDListA(pidl, path);
    CoTaskMemFree(pidl);
    return ok ? std::string(path) : std::string();
#elif defined(__APPLE__)
    return run_dialog_command("osascript -e 'POSIX path of (choose folder with prompt \"Select export folder\")' 2>/dev/null");
#else
    std::string folder = run_dialog_command("zenity --file-selection --directory --title='Select export folder' 2>/dev/null");
    if (!folder.empty()) return folder;
    return run_dialog_command("kdialog --getexistingdirectory . 'Select export folder' 2>/dev/null");
#endif
}

static std::string choose_import_model_file() {
#ifdef _WIN32
    char path[MAX_PATH] = {};
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFile = path;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = "Select model file";
    ofn.lpstrFilter = "Model Files\0*.stl;*.obj;*.step;*.stp;*.iges;*.igs;*.brep\0All Files\0*.*\0";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    return GetOpenFileNameA(&ofn) ? std::string(path) : std::string();
#elif defined(__APPLE__)
    return run_dialog_command(
        "osascript -e 'POSIX path of (choose file with prompt \"Select model file\" of type {\"stl\", \"obj\", \"step\", \"stp\", \"iges\", \"igs\", \"brep\"})' 2>/dev/null");
#else
    std::string file = run_dialog_command("zenity --file-selection --title='Select model file' --file-filter='Model files | *.stl *.obj *.step *.stp *.iges *.igs *.brep' 2>/dev/null");
    if (!file.empty()) return file;
    return run_dialog_command("kdialog --getopenfilename . 'Model files (*.stl *.obj *.step *.stp *.iges *.igs *.brep)' 2>/dev/null");
#endif
}

static bool valid_export_name(const std::string& name) {
    if (name.empty() || name == "." || name == "..") return false;
    return name.find('/') == std::string::npos && name.find('\\') == std::string::npos;
}

static std::filesystem::path export_path_for(const std::string& folder, std::string filename, int export_type) {
    static const char* extensions[] = {".stl", ".obj", ".step"};
    const std::string wanted_ext = extensions[std::clamp(export_type, 0, 2)];
    std::filesystem::path path(filename);
    if (path.extension().empty()) {
        filename += wanted_ext;
    }
    return std::filesystem::path(folder) / filename;
}

// ============================================================================
// Shader / FBO helpers (shared viewport state, static)
// ============================================================================
struct SharedView {
    bool shader_inited = false;
    Shader shader;
    GLuint fbo = 0, tex = 0, rbo = 0;
    int w = 0, h = 0;
    bool fbo_complete = false;
    // Wireframe shader
    bool wire_shader_inited = false;
    Shader wire_shader;

    void ensure_shader() {
        if (shader_inited) return;
        static const char* vs = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            uniform mat4 uModel;
            uniform mat4 uView;
            uniform mat4 uProj;
            out vec3 Normal;
            out vec3 FragPos;
            void main() {
                FragPos = vec3(uModel * vec4(aPos, 1.0));
                Normal = mat3(transpose(inverse(uModel))) * aNormal;
                gl_Position = uProj * uView * vec4(FragPos, 1.0);
            }
        )";
        static const char* fs = R"(
            #version 330 core
            in vec3 Normal;
            in vec3 FragPos;
            out vec4 FragColor;
            uniform vec3 uColor;
            uniform vec3 uViewPos;
            uniform float uAlpha;
            void main() {
                vec3 n = normalize(Normal);
                vec3 lightDir = normalize(vec3(0.4, 1.0, 0.3));
                float diff = max(dot(n, lightDir), 0.0);
                vec3 col = 0.25*uColor + diff*0.6*uColor;
                vec3 viewDir = normalize(uViewPos - FragPos);
                vec3 reflectDir = reflect(-lightDir, n);
                float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
                col += spec * vec3(0.4);
                FragColor = vec4(col, uAlpha);
            }
        )";
        if (!shader.load_from_source(vs, fs))
            spdlog::warn("ModelEditor shader compile failed");
        shader_inited = true;
    }

    void ensure_wire_shader() {
        if (wire_shader_inited) return;
        static const char* vs = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            uniform mat4 uMVP;
            void main() { gl_Position = uMVP * vec4(aPos, 1.0); }
        )";
        static const char* fs = R"(
            #version 330 core
            out vec4 FragColor;
            uniform vec4 uLineColor;
            void main() { FragColor = uLineColor; }
        )";
        if (!wire_shader.load_from_source(vs, fs))
            spdlog::warn("ModelEditor wire shader compile failed");
        wire_shader_inited = true;
    }

    void ensure_fbo(int pw, int ph) {
        if (fbo && w == pw && h == ph) return;
        w = pw; h = ph;
        if (!fbo) glGenFramebuffers(1, &fbo);
        if (!tex) glGenTextures(1, &tex);
        if (!rbo) glGenRenderbuffers(1, &rbo);
        glBindTexture(GL_TEXTURE_2D, tex);
        // Use a sized internal format to avoid incomplete FBO issues on some GL drivers.
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, pw, ph, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, pw, ph);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
        fbo_complete = (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
        if (!fbo_complete) {
            spdlog::warn("ModelEditor framebuffer is incomplete ({}x{})", pw, ph);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
    }
};

static SharedView g_view;

// Build renderable Mesh from MeshData
static void build_render_mesh(Mesh& rm, const MeshData& md) {
    if (rm.vao) rm.cleanup();
    rm.vertices.clear();
    rm.indices.clear();
    rm.vertices.reserve(md.triangles.size() * 3);
    rm.indices.reserve(md.triangles.size() * 3);
    auto vtx = [&](uint32_t idx) -> Vertex {
        Vec3 p = md.vertices[idx];
        Vec3 n = (idx < md.normals.size()) ? md.normals[idx] : Vec3{0, 1, 0};
        return Vertex{{p.x, p.y, p.z}, {n.x, n.y, n.z}};
    };
    uint32_t base = 0;
    for (const auto& t : md.triangles) {
        if (t.v0 >= md.vertices.size() || t.v1 >= md.vertices.size() || t.v2 >= md.vertices.size()) continue;
        if (t.v0 == t.v1 || t.v1 == t.v2 || t.v2 == t.v0) continue;
        rm.vertices.push_back(vtx(t.v0));
        rm.vertices.push_back(vtx(t.v1));
        rm.vertices.push_back(vtx(t.v2));
        rm.indices.insert(rm.indices.end(), {base, base + 1, base + 2});
        base += 3;
    }
    if (!rm.vertices.empty() && !rm.indices.empty()) rm.upload();
}

// Build wireframe mesh (lines) from triangles for overlay
static void build_wire_mesh(Mesh& wm, const MeshData& md) {
    if (wm.vao) wm.cleanup();
    wm.vertices.clear();
    wm.indices.clear();
    for (const auto& t : md.triangles) {
        if (t.v0 >= md.vertices.size() || t.v1 >= md.vertices.size() || t.v2 >= md.vertices.size()) continue;
        if (t.v0 == t.v1 || t.v1 == t.v2 || t.v2 == t.v0) continue;
        Vec3 p0 = md.vertices[t.v0], p1 = md.vertices[t.v1], p2 = md.vertices[t.v2];
        Vec3 n{0, 1, 0};
        uint32_t b = (uint32_t)wm.vertices.size();
        wm.vertices.push_back({{p0.x, p0.y, p0.z}, {n.x, n.y, n.z}});
        wm.vertices.push_back({{p1.x, p1.y, p1.z}, {n.x, n.y, n.z}});
        wm.vertices.push_back({{p2.x, p2.y, p2.z}, {n.x, n.y, n.z}});
        wm.indices.insert(wm.indices.end(), {b, b+1, b+1, b+2, b+2, b});
    }
    if (!wm.vertices.empty() && !wm.indices.empty()) wm.upload();
}

// ============================================================================
// Load effective model for selected component type
// ============================================================================
void ModelEditor::load_effective_model(SimulationOrchestrator& orchestrator) {
    ModelAssetLibrary::MappingEntry effective{};
    const bool has_effective = (!m_selected_component_type.empty() &&
        m_lib.resolve_effective_asset_for_component(m_selected_component_type, effective));

    ModelAssetLibrary::MappingEntry edit_target{};
    bool has_edit_target = false;
    if (m_force_fallback_box) {
        has_edit_target = false;
    } else if (!m_selected_asset_id.empty()) {
        edit_target = {m_selected_asset_id, m_selected_asset_scope.empty() ? "user" : m_selected_asset_scope};
        has_edit_target = true;
    } else if (has_effective) {
        edit_target = effective;
        has_edit_target = true;
        m_selected_asset_id = effective.asset_id;
        m_selected_asset_scope = effective.scope;
    }

    const std::string type_key = m_selected_component_type.empty() ? std::string("<none>") : m_selected_component_type;
    const std::string key = has_edit_target
        ? (edit_target.scope + ":" + edit_target.asset_id)
        : (type_key + ":" + (m_force_fallback_box ? std::string("forced:fallback:box") : std::string("fallback:box")));

    if (m_edit_loaded && m_edit_asset_id == key) return;

    m_edit_asset_id = key;
    m_edit_loaded = false;
    m_edit_mesh.clear_selection();
    m_edit_mesh.clear_undo_stack();

    MeshData md;
    bool loaded = false;
    m_stack.modifiers.clear();
    m_preview_valid = false;
    if (has_edit_target) {
        std::string err;
        loaded = m_lib.load_asset_mesh(m_cad, edit_target.asset_id, edit_target.scope, md, &err);
        ModelAssetMeta meta;
        if (m_lib.load_asset_meta(edit_target.asset_id, edit_target.scope, meta, nullptr)) {
            m_stack = meta.modifiers;
        }
        if (!loaded) {
            m_error = err;
        } else {
            m_error.clear();
            m_loaded_mesh_source_label = edit_target.asset_id + " [" + edit_target.scope + "]";
        }
    }
    if (!loaded) {
        auto g = m_cad.create_box(1.0f, 1.0f, 1.0f);
        md = g->mesh;
        md.unify_vertices();
        md.calculate_normals();
        if (!has_edit_target) m_error.clear();
        m_loaded_mesh_source_label = has_edit_target
            ? "fallback box (asset load failed)"
            : (m_force_fallback_box ? "editable fallback box" : "fallback box (no default/user model)");
    }
    if (!m_edit_mesh.from_meshdata(md)) {
        m_error = "Failed to load editable mesh.";
        return;
    }
    m_edit_loaded = true;
    m_vp.mesh_valid = false;
    frame_current_mesh();
}

// ============================================================================
// Projection helper
// ============================================================================
bool ModelEditor::project_to_screen(const Vec3& world, float& sx, float& sy) {
    glm::vec4 p(world.x, world.y, world.z, 1.0f);
    glm::mat4 vm; memcpy(&vm, m_vp.view_matrix, 64);
    glm::mat4 pm; memcpy(&pm, m_vp.proj_matrix, 64);
    glm::vec4 clip = pm * vm * p;
    if (clip.w <= 0.001f) return false;
    glm::vec3 ndc(clip.x / clip.w, clip.y / clip.w, clip.z / clip.w);
    float hw = m_vp.w * 0.5f, hh = m_vp.h * 0.5f;
    sx = (ndc.x * 0.5f + 0.5f) * m_vp.w;
    sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * m_vp.h;
    return true;
}

// ============================================================================
// Picking helpers
// ============================================================================
int ModelEditor::pick_vertex_3d(float screen_x, float screen_y) {
    int best = -1;
    float best_dist = 25.0f; // max pixels
    const auto& verts = m_edit_mesh.vertices();
    for (int i = 0; i < (int)verts.size(); ++i) {
        float sx, sy;
        if (project_to_screen(verts[i], sx, sy)) {
            float dx = sx - screen_x, dy = sy - screen_y;
            float d = std::sqrt(dx*dx + dy*dy);
            if (d < best_dist) { best_dist = d; best = i; }
        }
    }
    return best;
}

EdgeKey ModelEditor::pick_edge_3d(float screen_x, float screen_y) {
    EdgeKey best;
    float best_dist = 25.0f;
    auto edges = m_edit_mesh.get_edges();
    for (const auto& ek : edges) {
        Vec3 c = m_edit_mesh.edge_center(ek);
        float sx, sy;
        if (project_to_screen(c, sx, sy)) {
            float dx = sx - screen_x, dy = sy - screen_y;
            float d = std::sqrt(dx*dx + dy*dy);
            if (d < best_dist) { best_dist = d; best = ek; }
        }
    }
    return best;
}

int ModelEditor::pick_face_3d(float screen_x, float screen_y) {
    int best = -1;
    float best_dist = 25.0f;
    for (uint32_t i = 0; i < (uint32_t)m_edit_mesh.triangle_count(); ++i) {
        Vec3 c = m_edit_mesh.face_center(i);
        float sx, sy;
        if (project_to_screen(c, sx, sy)) {
            float dx = sx - screen_x, dy = sy - screen_y;
            float d = std::sqrt(dx*dx + dy*dy);
            if (d < best_dist) { best_dist = d; best = (int)i; }
        }
    }
    return best;
}

// ============================================================================
// Update viewport mesh from edit mesh
// ============================================================================
void ModelEditor::update_viewport_mesh() {
    MeshData md = m_edit_mesh.to_meshdata();
    md.calculate_normals();

    m_preview_error.clear();
    if (m_show_modifier_preview && !m_stack.modifiers.empty()) {
        auto loader = [&](const std::string& aid, const std::string& scope, MeshData& out) -> bool {
            return load_asset_mesh_by_id(m_lib, m_cad, aid, scope, out);
        };
        MeshData preview;
        std::string err;
        if (m_mod_engine.apply(m_cad, md, m_stack, loader, preview, &err)) {
            md = std::move(preview);
            m_preview_mesh = md;
            m_preview_valid = true;
        } else {
            m_preview_valid = false;
            m_preview_error = err;
        }
    } else {
        m_preview_valid = false;
    }

    static Mesh rm, wm;
    build_render_mesh(rm, md);
    build_wire_mesh(wm, md);
    m_vp.mesh = &rm; // store pointer (static mesh)
    m_vp.wire_mesh = &wm;
    m_vp.mesh_valid = true;
}

void ModelEditor::frame_current_mesh() {
    if (!m_edit_loaded) return;

    MeshData md = m_edit_mesh.to_meshdata();
    if (md.is_empty()) return;

    Vec3 mn, mx;
    md.get_bounds(mn, mx);
    m_vp.target[0] = (mn.x + mx.x) * 0.5f;
    m_vp.target[1] = (mn.y + mx.y) * 0.5f;
    m_vp.target[2] = (mn.z + mx.z) * 0.5f;

    const float diagonal = std::max(0.25f, (mx - mn).length());
    const float radius = diagonal * 0.5f;
    m_vp.dist = std::max(1.5f, radius * 3.0f);
    m_vp.yaw = 0.6f;
    m_vp.pitch = 0.35f;
    m_vp.pan_x = 0.0f;
    m_vp.pan_y = 0.0f;
    m_vp.zoom_2d = 90.0f;
}

void ModelEditor::frame_selection() {
    if (!m_edit_loaded) return;

    const auto& verts = m_edit_mesh.vertices();
    const auto& tris = m_edit_mesh.triangles();
    if (verts.empty()) return;

    std::set<uint32_t> selected_vertices;
    switch (m_edit_mesh.select_mode()) {
        case SelectMode::Vertex:
            for (uint32_t v : m_edit_mesh.selected_vertices()) selected_vertices.insert(v);
            break;
        case SelectMode::Edge:
            for (const EdgeKey& e : m_edit_mesh.selected_edges()) {
                selected_vertices.insert(e.v0);
                selected_vertices.insert(e.v1);
            }
            break;
        case SelectMode::Face:
            for (uint32_t f : m_edit_mesh.selected_faces()) {
                if (f >= tris.size()) continue;
                selected_vertices.insert(tris[f].v0);
                selected_vertices.insert(tris[f].v1);
                selected_vertices.insert(tris[f].v2);
            }
            break;
    }

    if (selected_vertices.empty()) {
        frame_current_mesh();
        return;
    }

    bool have_bounds = false;
    Vec3 mn, mx;
    for (uint32_t idx : selected_vertices) {
        if (idx >= verts.size()) continue;
        const Vec3& v = verts[idx];
        if (!have_bounds) {
            mn = mx = v;
            have_bounds = true;
        } else {
            mn.x = std::min(mn.x, v.x);
            mn.y = std::min(mn.y, v.y);
            mn.z = std::min(mn.z, v.z);
            mx.x = std::max(mx.x, v.x);
            mx.y = std::max(mx.y, v.y);
            mx.z = std::max(mx.z, v.z);
        }
    }
    if (!have_bounds) {
        frame_current_mesh();
        return;
    }

    m_vp.target[0] = (mn.x + mx.x) * 0.5f;
    m_vp.target[1] = (mn.y + mx.y) * 0.5f;
    m_vp.target[2] = (mn.z + mx.z) * 0.5f;
    const float diagonal = std::max(0.1f, (mx - mn).length());
    m_vp.dist = std::max(0.4f, diagonal * 3.0f);
    m_vp.pan_x = 0.0f;
    m_vp.pan_y = 0.0f;
    m_vp.zoom_2d = std::clamp(160.0f / diagonal, 10.0f, 600.0f);
}

// ============================================================================
// Render 3D viewport into FBO
// ============================================================================
void ModelEditor::render_3d_view(int pw, int ph) {
    GLint prev_fbo = 0;
    GLint prev_viewport[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);
    glGetIntegerv(GL_VIEWPORT, prev_viewport);
    GLboolean prev_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean prev_cull = glIsEnabled(GL_CULL_FACE);
    GLboolean prev_blend = glIsEnabled(GL_BLEND);

    g_view.ensure_shader();
    g_view.ensure_wire_shader();
    g_view.ensure_fbo(pw, ph);
    if (!g_view.fbo_complete) {
        glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)prev_fbo);
        glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
        prev_depth ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST);
        prev_cull ? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
        prev_blend ? glEnable(GL_BLEND) : glDisable(GL_BLEND);
        return;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, g_view.fbo);
    glViewport(0, 0, g_view.w, g_view.h);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Camera
    float aspect = (float)g_view.w / (float)g_view.h;
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.01f, 500.0f);
    glm::vec3 target(m_vp.target[0], m_vp.target[1], m_vp.target[2]);
    glm::vec3 eye(
        target.x + m_vp.dist * std::cos(m_vp.pitch) * std::cos(m_vp.yaw),
        target.y + m_vp.dist * std::sin(m_vp.pitch),
        target.z + m_vp.dist * std::cos(m_vp.pitch) * std::sin(m_vp.yaw)
    );
    glm::mat4 view = glm::lookAt(eye, target, glm::vec3(0, 1, 0));
    glm::mat4 model(1.0f);

    // Cache for picking
    memcpy(m_vp.view_matrix, &view[0][0], 16 * sizeof(float));
    memcpy(m_vp.proj_matrix, &proj[0][0], 16 * sizeof(float));
    m_vp.vp[0] = 0; m_vp.vp[1] = 0; m_vp.vp[2] = g_view.w; m_vp.vp[3] = g_view.h;
    m_vp.w = g_view.w; m_vp.h = g_view.h;

    // Draw grid
    if (m_vp.show_grid) {
        static Mesh grid_mesh;
        static bool grid_built = false;
        if (!grid_built) {
            grid_mesh = Mesh::create_grid(10.0f, 20);
            grid_built = true;
        }
        g_view.wire_shader.bind();
        glm::mat4 grid_mvp = proj * view;
        g_view.wire_shader.set_uniform("uMVP", grid_mvp);
        g_view.wire_shader.set_uniform("uLineColor", glm::vec4(0.25f, 0.25f, 0.28f, 0.5f));
        grid_mesh.draw_lines();
    }

    // Draw solid mesh
    if (m_vp.mesh_valid && m_vp.mesh) {
        Mesh* rm = (Mesh*)m_vp.mesh;
        g_view.shader.bind();
        g_view.shader.set_uniform("uModel", model);
        g_view.shader.set_uniform("uView", view);
        g_view.shader.set_uniform("uProj", proj);
        g_view.shader.set_uniform("uViewPos", eye);
        g_view.shader.set_uniform("uColor", glm::vec3(0.7f, 0.72f, 0.76f));
        g_view.shader.set_uniform("uAlpha", 1.0f);
        rm->draw();

        // Wireframe overlay
        if (m_vp.show_wireframe) {
            Mesh* wm = m_vp.wire_mesh ? (Mesh*)m_vp.wire_mesh : nullptr;
            g_view.wire_shader.bind();
            glm::mat4 mvp = proj * view * model;
            g_view.wire_shader.set_uniform("uMVP", mvp);
            g_view.wire_shader.set_uniform("uLineColor", glm::vec4(0.08f, 0.09f, 0.11f, 0.8f));
            glLineWidth(1.0f);
            if (wm) wm->draw_lines();
        }
    }

    // Draw selection highlights
    if (m_edit_loaded) {
        MeshData md = m_edit_mesh.to_meshdata();
        md.calculate_normals();

        SelectMode sm = m_edit_mesh.select_mode();

        // Highlight selected vertices as points
        if (sm == SelectMode::Vertex && !m_edit_mesh.selected_vertices().empty()) {
            Mesh sel_mesh;
            sel_mesh.vertices.clear();
            sel_mesh.indices.clear();
            for (uint32_t vi : m_edit_mesh.selected_vertices()) {
                if (vi < md.vertices.size()) {
                    Vec3 p = md.vertices[vi];
                    sel_mesh.vertices.push_back({{p.x, p.y, p.z}, {0, 1, 0}});
                    sel_mesh.indices.push_back((uint32_t)sel_mesh.indices.size());
                }
            }
            if (!sel_mesh.vertices.empty()) {
                sel_mesh.upload();
                g_view.wire_shader.bind();
                glm::mat4 mvp = proj * view;
                g_view.wire_shader.set_uniform("uMVP", mvp);
                g_view.wire_shader.set_uniform("uLineColor", glm::vec4(1.0f, 0.85f, 0.2f, 1.0f));
                sel_mesh.draw_points(8.0f);
                sel_mesh.cleanup();
            }
        }

        // Highlight selected edges
        if (sm == SelectMode::Edge && !m_edit_mesh.selected_edges().empty()) {
            Mesh sel_mesh;
            sel_mesh.vertices.clear();
            sel_mesh.indices.clear();
            for (const auto& ek : m_edit_mesh.selected_edges()) {
                if (ek.v0 < md.vertices.size() && ek.v1 < md.vertices.size()) {
                    Vec3 p0 = md.vertices[ek.v0], p1 = md.vertices[ek.v1];
                    Vec3 n{0, 1, 0};
                    uint32_t b = (uint32_t)sel_mesh.vertices.size();
                    sel_mesh.vertices.push_back({{p0.x, p0.y, p0.z}, {n.x, n.y, n.z}});
                    sel_mesh.vertices.push_back({{p1.x, p1.y, p1.z}, {n.x, n.y, n.z}});
                    sel_mesh.indices.insert(sel_mesh.indices.end(), {b, b+1});
                }
            }
            if (!sel_mesh.vertices.empty()) {
                sel_mesh.upload();
                g_view.wire_shader.bind();
                glm::mat4 mvp = proj * view;
                g_view.wire_shader.set_uniform("uMVP", mvp);
                g_view.wire_shader.set_uniform("uLineColor", glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
                glLineWidth(3.0f);
                glBindVertexArray(sel_mesh.vao);
                glDrawElements(GL_LINES, (GLsizei)sel_mesh.indices.size(), GL_UNSIGNED_INT, nullptr);
                glBindVertexArray(0);
                sel_mesh.cleanup();
            }
        }

        // Highlight selected faces
        if (sm == SelectMode::Face && !m_edit_mesh.selected_faces().empty()) {
            Mesh sel_mesh;
            sel_mesh.vertices.clear();
            sel_mesh.indices.clear();
            uint32_t b = 0;
            for (uint32_t fi : m_edit_mesh.selected_faces()) {
                if (fi < md.triangles.size()) {
                    const auto& t = md.triangles[fi];
                    for (int i = 0; i < 3; ++i) {
                        uint32_t vi = (i == 0) ? t.v0 : (i == 1) ? t.v1 : t.v2;
                        Vec3 p = md.vertices[vi];
                        Vec3 n = (vi < md.normals.size()) ? md.normals[vi] : Vec3{0, 1, 0};
                        sel_mesh.vertices.push_back({{p.x, p.y, p.z}, {n.x, n.y, n.z}});
                    }
                    sel_mesh.indices.insert(sel_mesh.indices.end(), {b, b+1, b+2});
                    b += 3;
                }
            }
            if (!sel_mesh.vertices.empty()) {
                sel_mesh.upload();
                g_view.shader.bind();
                g_view.shader.set_uniform("uModel", model);
                g_view.shader.set_uniform("uView", view);
                g_view.shader.set_uniform("uProj", proj);
                g_view.shader.set_uniform("uViewPos", eye);
                g_view.shader.set_uniform("uColor", glm::vec3(1.0f, 0.5f, 0.15f));
                g_view.shader.set_uniform("uAlpha", 0.5f);
                sel_mesh.draw();
                sel_mesh.cleanup();
            }
        }
    }

    glDisable(GL_BLEND);
    glUseProgram(0);
    glBindVertexArray(0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    prev_depth ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST);
    prev_cull ? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
    prev_blend ? glEnable(GL_BLEND) : glDisable(GL_BLEND);
    glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)prev_fbo);
    glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
}

// ============================================================================
// Render 2D viewport (ImGui draw list)
// ============================================================================
void ModelEditor::render_2d_view(int pw, int ph) {
    ImGui::BeginChild("Edit2D", ImVec2((float)pw, (float)ph), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 sz = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("edit2d_btn", sz,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    bool hov = ImGui::IsItemHovered();
    ImVec2 mp = ImGui::GetMousePos();

    // Pan with RMB drag
    if (hov && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        ImVec2 d = ImGui::GetIO().MouseDelta;
        m_vp.pan_x += d.x;
        m_vp.pan_y += d.y;
    }
    // Zoom with wheel
    if (hov && ImGui::GetIO().MouseWheel != 0.0f) {
        float z = (ImGui::GetIO().MouseWheel > 0) ? 1.1f : 0.9f;
        m_vp.zoom_2d = std::clamp(m_vp.zoom_2d * z, 5.0f, 600.0f);
    }

    // Choose projection plane based on view_mode
    // 1=top (XZ), 2=front (XY), 3=side (YZ)
    auto to2d = [&](const Vec3& v) -> ImVec2 {
        float u, v2;
        if (m_vp.view_mode == 1) { u = v.x; v2 = -v.z; }       // top
        else if (m_vp.view_mode == 2) { u = v.x; v2 = -v.y; }   // front
        else { u = v.z; v2 = -v.y; }                              // side
        float x = u * m_vp.zoom_2d + (p0.x + sz.x * 0.5f + m_vp.pan_x);
        float y = v2 * m_vp.zoom_2d + (p0.y + sz.y * 0.5f + m_vp.pan_y);
        return ImVec2(x, y);
    };

    const auto& verts = m_edit_mesh.vertices();
    const auto& tris = m_edit_mesh.triangles();
    SelectMode sm = m_edit_mesh.select_mode();

    // Draw filled selected faces
    for (uint32_t fi : m_edit_mesh.selected_faces()) {
        if (fi < tris.size()) {
            const auto& t = tris[fi];
            ImVec2 a = to2d(verts[t.v0]), b = to2d(verts[t.v1]), c = to2d(verts[t.v2]);
            dl->AddTriangleFilled(a, b, c, IM_COL32(255, 130, 40, 100));
        }
    }

    // Draw all triangles as wireframe
    for (const auto& t : tris) {
        ImVec2 a = to2d(verts[t.v0]);
        ImVec2 b = to2d(verts[t.v1]);
        ImVec2 c = to2d(verts[t.v2]);
        dl->AddLine(a, b, IM_COL32(180, 180, 190, 100), 1.0f);
        dl->AddLine(b, c, IM_COL32(180, 180, 190, 100), 1.0f);
        dl->AddLine(c, a, IM_COL32(180, 180, 190, 100), 1.0f);
    }

    // Highlight selected edges
    for (const auto& ek : m_edit_mesh.selected_edges()) {
        if (ek.v0 < verts.size() && ek.v1 < verts.size()) {
            ImVec2 a = to2d(verts[ek.v0]), b = to2d(verts[ek.v1]);
            dl->AddLine(a, b, IM_COL32(255, 130, 40, 220), 3.0f);
        }
    }

    // Draw vertices
    for (int i = 0; i < (int)verts.size(); ++i) {
        ImVec2 p = to2d(verts[i]);
        ImU32 col;
        if (sm == SelectMode::Vertex && m_edit_mesh.is_vertex_selected((uint32_t)i))
            col = IM_COL32(255, 220, 80, 240);
        else if (sm == SelectMode::Edge || sm == SelectMode::Face)
            col = IM_COL32(140, 170, 200, 150);
        else
            col = IM_COL32(120, 180, 255, 180);
        dl->AddCircleFilled(p, 3.5f, col);
    }

    // Box select
    if (hov && m_active_tool == 1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_box_selecting = true;
        m_box_start[0] = mp.x;
        m_box_start[1] = mp.y;
        m_box_end[0] = mp.x;
        m_box_end[1] = mp.y;
        m_selection_started = true;
        const bool add = ImGui::GetIO().KeyShift;
        const bool sub = ImGui::GetIO().KeyCtrl;
        m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
        m_selection_cleared = false;
    }
    if (m_box_selecting && m_active_tool == 1) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            m_box_end[0] = mp.x;
            m_box_end[1] = mp.y;
        } else {
            auto project = [&](const Vec3& world, float& sx, float& sy) -> bool {
                ImVec2 projected = to2d(world);
                sx = projected.x;
                sy = projected.y;
                return true;
            };
            m_edit_mesh.box_select(m_box_start[0], m_box_start[1],
                                   m_box_end[0], m_box_end[1], m_select_op, project);
            m_box_selecting = false;
            m_selection_started = false;
            m_selection_cleared = false;
        }
    }

    // Circle select
    if (hov && m_active_tool == 2) {
        dl->AddCircle(mp, m_circle_radius, IM_COL32(255, 220, 80, 180), 48, 1.5f);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (!m_selection_started) {
                m_selection_started = true;
                const bool add = ImGui::GetIO().KeyShift;
                const bool sub = ImGui::GetIO().KeyCtrl;
                m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
                m_selection_cleared = false;
            }
            auto project = [&](const Vec3& world, float& sx, float& sy) -> bool {
                ImVec2 projected = to2d(world);
                sx = projected.x;
                sy = projected.y;
                return true;
            };
            if (m_select_op == SelectionOp::Replace && !m_selection_cleared) {
                m_edit_mesh.clear_selection();
                m_selection_cleared = true;
            }
            const SelectionOp op = (m_select_op == SelectionOp::Replace) ? SelectionOp::Add : m_select_op;
            m_edit_mesh.circle_select(mp.x, mp.y, m_circle_radius, op, project);
        } else {
            m_selection_started = false;
            m_selection_cleared = false;
        }
    }

    // Lasso select
    if (hov && m_active_tool == 3) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            m_lasso_selecting = true;
            m_lasso_points.clear();
            const bool add = ImGui::GetIO().KeyShift;
            const bool sub = ImGui::GetIO().KeyCtrl;
            m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
        }
        if (m_lasso_selecting && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (m_lasso_points.empty()) {
                m_lasso_points.push_back({mp.x, mp.y});
            } else {
                const auto& last = m_lasso_points.back();
                const float dx = mp.x - last[0];
                const float dy = mp.y - last[1];
                if (dx * dx + dy * dy >= 16.0f) {
                    m_lasso_points.push_back({mp.x, mp.y});
                }
            }
        }
        if (m_lasso_selecting && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            auto project = [&](const Vec3& world, float& sx, float& sy) -> bool {
                ImVec2 projected = to2d(world);
                sx = projected.x;
                sy = projected.y;
                return true;
            };
            m_edit_mesh.polygon_select(m_lasso_points, m_select_op, project);
            m_lasso_selecting = false;
            m_lasso_points.clear();
        }
    }
    if (m_lasso_selecting && m_lasso_points.size() > 1) {
        for (size_t i = 1; i < m_lasso_points.size(); ++i) {
            dl->AddLine(ImVec2(m_lasso_points[i - 1][0], m_lasso_points[i - 1][1]),
                        ImVec2(m_lasso_points[i][0], m_lasso_points[i][1]),
                        IM_COL32(255, 220, 80, 210), 1.5f);
        }
    }

    // Click picking
    if (hov && m_active_tool == 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !m_box_selecting) {
        const bool add = ImGui::GetIO().KeyShift;
        const bool sub = ImGui::GetIO().KeyCtrl;
        if (!add && !sub) {
            // Replace selection by default
        }
        if (sm == SelectMode::Vertex) {
            int hit = -1; float best = 1e9f;
            for (int i = 0; i < (int)verts.size(); ++i) {
                ImVec2 p = to2d(verts[i]);
                float dx = mp.x - p.x, dy = mp.y - p.y;
                float d2 = dx*dx + dy*dy;
                if (d2 < best && d2 < 144.0f) { best = d2; hit = i; }
            }
            if (hit >= 0) {
                if (sub) m_edit_mesh.deselect_vertex((uint32_t)hit);
                else m_edit_mesh.select_vertex((uint32_t)hit, add);
            } else if (!add && !sub) {
                m_edit_mesh.clear_selection();
            }
        } else if (sm == SelectMode::Edge) {
            // Pick nearest edge midpoint
            auto edges = m_edit_mesh.get_edges();
            EdgeKey best_ek; float best = 1e9f; bool found = false;
            for (const auto& ek : edges) {
                Vec3 c = m_edit_mesh.edge_center(ek);
                ImVec2 p = to2d(c);
                float dx = mp.x - p.x, dy = mp.y - p.y;
                float d2 = dx*dx + dy*dy;
                if (d2 < best && d2 < 225.0f) { best = d2; best_ek = ek; found = true; }
            }
            if (found) {
                if (sub) m_edit_mesh.deselect_edge(best_ek);
                else m_edit_mesh.select_edge(best_ek, add);
            } else if (!add && !sub) {
                m_edit_mesh.clear_selection();
            }
        } else if (sm == SelectMode::Face) {
            int hit = -1; float best = 1e9f;
            for (uint32_t i = 0; i < (uint32_t)tris.size(); ++i) {
                Vec3 c = m_edit_mesh.face_center(i);
                ImVec2 p = to2d(c);
                float dx = mp.x - p.x, dy = mp.y - p.y;
                float d2 = dx*dx + dy*dy;
                if (d2 < best && d2 < 400.0f) { best = d2; hit = (int)i; }
            }
            if (hit >= 0) {
                if (sub) m_edit_mesh.deselect_face((uint32_t)hit);
                else m_edit_mesh.select_face((uint32_t)hit, add);
            } else if (!add && !sub) {
                m_edit_mesh.clear_selection();
            }
        }
    }

    // Box select (drag)
    if (m_box_selecting && hov) {
        ImVec2 bs(m_box_start[0], m_box_start[1]);
        ImVec2 be(m_box_end[0], m_box_end[1]);
        dl->AddRect(bs, be, IM_COL32(255, 220, 80, 180), 0.0f, 0, 1.5f);
    }

    ImGui::EndChild();
}

// ============================================================================
// Toolbar (top bar)
// ============================================================================
void ModelEditor::render_toolbar() {
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 2));

    // Select mode
    const char* modes[] = {"Vertex", "Edge", "Face"};
    int sm = (int)m_edit_mesh.select_mode();
    ImGui::SetNextItemWidth(80);
    if (ImGui::Combo("Mode", &sm, modes, 3)) {
        m_edit_mesh.set_select_mode((SelectMode)sm);
        m_edit_mesh.clear_selection();
    }

    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    // Transform mode
    const char* tmodes[] = {"Move", "Rotate", "Scale"};
    int tm = (int)m_transform_mode;
    ImGui::SetNextItemWidth(80);
    if (ImGui::Combo("Transform", &tm, tmodes, 3))
        m_transform_mode = (TransformMode)tm;

    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    // Tool buttons
    ImGui::PushStyleColor(ImGuiCol_Button, m_active_tool == 0 ? ImVec4(0.4f, 0.6f, 0.9f, 1) : ImVec4(0.3f, 0.3f, 0.35f, 1));
    if (ImGui::Button("Select")) m_active_tool = 0;
    ImGui::PopStyleColor();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, m_active_tool == 1 ? ImVec4(0.4f, 0.6f, 0.9f, 1) : ImVec4(0.3f, 0.3f, 0.35f, 1));
    if (ImGui::Button("Box Sel")) m_active_tool = 1;
    ImGui::PopStyleColor();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, m_active_tool == 2 ? ImVec4(0.4f, 0.6f, 0.9f, 1) : ImVec4(0.3f, 0.3f, 0.35f, 1));
    if (ImGui::Button("Circle Sel")) m_active_tool = 2;
    ImGui::PopStyleColor();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, m_active_tool == 3 ? ImVec4(0.4f, 0.6f, 0.9f, 1) : ImVec4(0.3f, 0.3f, 0.35f, 1));
    if (ImGui::Button("Lasso")) m_active_tool = 3;
    ImGui::PopStyleColor();
    ImGui::SameLine();

    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    // Undo / Redo
    if (ImGui::Button("Undo")) {
        if (m_edit_mesh.undo()) {
            m_vp.mesh_valid = false;
            m_preview_valid = false;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Redo")) {
        if (m_edit_mesh.redo()) {
            m_vp.mesh_valid = false;
            m_preview_valid = false;
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(%d/%d)", (int)m_edit_mesh.can_undo(), (int)m_edit_mesh.can_redo());

    ImGui::SameLine();
    ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
    ImGui::SameLine();

    // View controls
    const char* views[] = {"3D", "2D Top", "2D Front", "2D Side"};
    ImGui::SetNextItemWidth(90);
    if (ImGui::Combo("View", &m_vp.view_mode, views, 4))
        m_vp.mesh_valid = false; // force rebuild

    ImGui::SameLine();
    ImGui::Checkbox("Wire", &m_vp.show_wireframe);
    ImGui::SameLine();
    ImGui::Checkbox("Grid", &m_vp.show_grid);
    ImGui::SameLine();
    ImGui::Checkbox("Snap", &m_snap_enabled);
    if (m_snap_enabled) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::DragFloat("Step", &m_snap_increment, 0.01f, 0.001f, 100.0f, "%.3f");
    }
    ImGui::SameLine();
    const char* axes[] = {"None", "X", "Y", "Z"};
    ImGui::SetNextItemWidth(70);
    ImGui::Combo("Axis", &m_axis_constraint, axes, 4);

    ImGui::PopStyleVar();
}

// ============================================================================
// Tool Shelf (left panel)
// ============================================================================
void ModelEditor::render_tool_shelf() {
    ImGui::Text("Mesh Operations");
    ImGui::Separator();

    // Extrude
    static float extrude_dist = 0.1f;
    ImGui::DragFloat("Extrude Dist", &extrude_dist, 0.01f, -10.0f, 10.0f);
    if (ImGui::Button("Extrude Selected")) {
        m_edit_mesh.push_undo("Extrude");
        m_edit_mesh.extrude_selected(extrude_dist);
        m_vp.mesh_valid = false;
    }

    // Inset
    static float inset_thick = 0.1f, inset_depth = 0.0f;
    ImGui::DragFloat("Inset Thick", &inset_thick, 0.01f, 0.0f, 10.0f);
    ImGui::DragFloat("Inset Depth", &inset_depth, 0.01f, -10.0f, 10.0f);
    if (ImGui::Button("Inset Faces")) {
        m_edit_mesh.push_undo("Inset");
        auto sel = std::vector<uint32_t>(m_edit_mesh.selected_faces().begin(),
                                          m_edit_mesh.selected_faces().end());
        m_edit_mesh.inset_faces(sel, inset_thick, inset_depth);
        m_vp.mesh_valid = false;
    }

    // Subdivide
    static int subd_cuts = 1;
    ImGui::DragInt("Subdivide Cuts", &subd_cuts, 1, 1, 10);
    if (ImGui::Button("Subdivide Edges")) {
        m_edit_mesh.push_undo("Subdivide");
        auto sel = std::vector<EdgeKey>(m_edit_mesh.selected_edges().begin(),
                                         m_edit_mesh.selected_edges().end());
        m_edit_mesh.subdivide_edges(sel, subd_cuts);
        m_vp.mesh_valid = false;
    }

    // Loop Cut
    static float loop_pos = 0.5f;
    ImGui::DragFloat("Loop Position", &loop_pos, 0.01f, 0.0f, 1.0f);
    if (ImGui::Button("Loop Cut (first sel edge)")) {
        auto& sel = m_edit_mesh.selected_edges();
        if (!sel.empty()) {
            m_edit_mesh.push_undo("Loop Cut");
            m_edit_mesh.loop_cut(*sel.begin(), loop_pos);
            m_vp.mesh_valid = false;
        }
    }

    // Bevel
    static float bevel_amt = 0.05f;
    static int bevel_seg = 1;
    ImGui::DragFloat("Bevel Amt", &bevel_amt, 0.005f, 0.0f, 2.0f);
    ImGui::DragInt("Bevel Seg", &bevel_seg, 1, 1, 8);
    if (ImGui::Button("Bevel Edges")) {
        m_edit_mesh.push_undo("Bevel");
        auto sel = std::vector<EdgeKey>(m_edit_mesh.selected_edges().begin(),
                                         m_edit_mesh.selected_edges().end());
        m_edit_mesh.bevel_edges(sel, bevel_amt, bevel_seg);
        m_vp.mesh_valid = false;
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Delete
    if (ImGui::Button("Delete Selected")) {
        m_edit_mesh.push_undo("Delete");
        m_edit_mesh.delete_selected();
        m_vp.mesh_valid = false;
    }

    // Duplicate
    ImGui::SameLine();
    if (ImGui::Button("Duplicate")) {
        m_edit_mesh.push_undo("Duplicate");
        m_edit_mesh.duplicate_selected();
        m_vp.mesh_valid = false;
    }
    if (ImGui::Button("Delete Loose")) {
        m_edit_mesh.push_undo("Delete Loose");
        m_edit_mesh.delete_loose_vertices();
        m_vp.mesh_valid = false;
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Merge
    if (ImGui::Button("Merge at Center")) {
        m_edit_mesh.push_undo("Merge Center");
        m_edit_mesh.merge_selected_vertices_to_center();
        m_vp.mesh_valid = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Merge at First")) {
        m_edit_mesh.push_undo("Merge First");
        m_edit_mesh.merge_selected_vertices_to_first();
        m_vp.mesh_valid = false;
    }

    // Normals
    if (ImGui::Button("Flip Normals")) {
        m_edit_mesh.push_undo("Flip Normals");
        m_edit_mesh.flip_selected_normals();
        m_vp.mesh_valid = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Recalc Normals")) {
        m_edit_mesh.push_undo("Recalc Normals");
        m_edit_mesh.recalculate_normals();
        m_vp.mesh_valid = false;
    }

    // Selection ops
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Selection");
    if (m_active_tool == 2) {
        ImGui::DragFloat("Circle Radius", &m_circle_radius, 1.0f, 4.0f, 200.0f);
    }
    if (ImGui::Button("Select All")) m_edit_mesh.select_all();
    ImGui::SameLine();
    if (ImGui::Button("Deselect All")) m_edit_mesh.clear_selection();
    ImGui::SameLine();
    if (ImGui::Button("Invert")) m_edit_mesh.invert_selection();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Add");
    static float new_v[3] = {0, 0, 0};
    ImGui::DragFloat3("New Vertex", new_v, 0.01f);
    if (ImGui::Button("Add Vertex")) {
        m_edit_mesh.push_undo("Add Vertex");
        m_edit_mesh.add_vertex(Vec3{new_v[0], new_v[1], new_v[2]});
        m_vp.mesh_valid = false;
    }
}

// ============================================================================
// Properties Panel (right side)
// ============================================================================
void ModelEditor::render_properties_panel() {
    ImGui::Text("Properties");
    ImGui::Separator();

    if (!m_edit_loaded) {
        ImGui::TextDisabled("No mesh loaded.");
        return;
    }

    // Mesh info
    ImGui::Text("Mesh Info");
    ImGui::TextDisabled("V: %zu  E: %zu  F: %zu",
        m_edit_mesh.vertex_count(), m_edit_mesh.edge_count(), m_edit_mesh.triangle_count());

    ImGui::Spacing();

    // Transform
    ImGui::Text("Transform");
    SelectMode sm = m_edit_mesh.select_mode();

    Vec3 sel_center{0, 0, 0};
    m_edit_mesh.get_selection_center(sel_center);
    ImGui::TextDisabled("Pivot: (%.2f, %.2f, %.2f)", sel_center.x, sel_center.y, sel_center.z);

    if (m_transform_mode == TransformMode::Translate) {
        if (ImGui::DragFloat3("Translate", m_transform_delta, 0.01f)) {}
        if (ImGui::Button("Apply Translate")) {
            m_edit_mesh.push_undo("Translate");
            Vec3 delta{m_transform_delta[0], m_transform_delta[1], m_transform_delta[2]};
            if (m_axis_constraint == 1) delta = Vec3{delta.x, 0.0f, 0.0f};
            else if (m_axis_constraint == 2) delta = Vec3{0.0f, delta.y, 0.0f};
            else if (m_axis_constraint == 3) delta = Vec3{0.0f, 0.0f, delta.z};
            if (m_snap_enabled && m_snap_increment > 0.0f) {
                auto snap = [&](float value) {
                    return std::round(value / m_snap_increment) * m_snap_increment;
                };
                delta = Vec3{snap(delta.x), snap(delta.y), snap(delta.z)};
            }
            m_edit_mesh.translate_selected(delta);
            m_transform_delta[0] = m_transform_delta[1] = m_transform_delta[2] = 0;
            m_vp.mesh_valid = false;
        }
    } else if (m_transform_mode == TransformMode::Rotate) {
        ImGui::DragFloat("Angle (deg)", &m_rotate_angle, 1.0f, -360.0f, 360.0f);
        static float rot_axis[3] = {0, 1, 0};
        ImGui::DragFloat3("Axis", rot_axis, 0.01f);
        if (ImGui::Button("Apply Rotate")) {
            m_edit_mesh.push_undo("Rotate");
            m_edit_mesh.rotate_selected(sel_center,
                Vec3{rot_axis[0], rot_axis[1], rot_axis[2]}, m_rotate_angle);
            m_vp.mesh_valid = false;
        }
    } else if (m_transform_mode == TransformMode::Scale) {
        if (ImGui::DragFloat3("Scale", m_scale_factor, 0.01f, 0.01f, 100.0f)) {}
        if (ImGui::Button("Apply Scale")) {
            m_edit_mesh.push_undo("Scale");
            m_edit_mesh.scale_selected(sel_center,
                Vec3{m_scale_factor[0], m_scale_factor[1], m_scale_factor[2]});
            m_scale_factor[0] = m_scale_factor[1] = m_scale_factor[2] = 1.0f;
            m_vp.mesh_valid = false;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Vertex list (scrollable, limited display)
    ImGui::Text("Elements");
    SelectMode current_mode = m_edit_mesh.select_mode();
    if (current_mode == SelectMode::Vertex) {
        ImGui::BeginChild("VertList", ImVec2(0, 150), true);
        const auto& verts = m_edit_mesh.vertices();
        for (int i = 0; i < (int)verts.size(); ++i) {
            bool sel = m_edit_mesh.is_vertex_selected((uint32_t)i);
            char label[128];
            snprintf(label, sizeof(label), "V%d: (%.2f, %.2f, %.2f)", i, verts[i].x, verts[i].y, verts[i].z);
            if (ImGui::Selectable(label, sel)) {
                m_edit_mesh.select_vertex((uint32_t)i, ImGui::GetIO().KeyCtrl);
            }
        }
        ImGui::EndChild();
    } else if (current_mode == SelectMode::Edge) {
        ImGui::BeginChild("EdgeList", ImVec2(0, 150), true);
        auto edges = m_edit_mesh.get_edges();
        for (int i = 0; i < (int)edges.size(); ++i) {
            const auto& ek = edges[i];
            bool sel = m_edit_mesh.is_edge_selected(ek);
            char label[128];
            snprintf(label, sizeof(label), "E%d: (%u-%u)", i, ek.v0, ek.v1);
            if (ImGui::Selectable(label, sel)) {
                m_edit_mesh.select_edge(ek, ImGui::GetIO().KeyCtrl);
            }
        }
        ImGui::EndChild();
    } else {
        ImGui::BeginChild("FaceList", ImVec2(0, 150), true);
        const auto& tris = m_edit_mesh.triangles();
        for (int i = 0; i < (int)tris.size(); ++i) {
            bool sel = m_edit_mesh.is_face_selected((uint32_t)i);
            char label[128];
            snprintf(label, sizeof(label), "F%d: (%u,%u,%u)", i, tris[i].v0, tris[i].v1, tris[i].v2);
            if (ImGui::Selectable(label, sel)) {
                m_edit_mesh.select_face((uint32_t)i, ImGui::GetIO().KeyCtrl);
            }
        }
        ImGui::EndChild();
    }
}

// ============================================================================
// Status Bar (bottom)
// ============================================================================
void ModelEditor::render_status_bar() {
    ImGui::Separator();
    if (!m_edit_loaded) return;

    SelectMode sm = m_edit_mesh.select_mode();
    const char* sm_name = (sm == SelectMode::Vertex) ? "Vertex" : (sm == SelectMode::Edge) ? "Edge" : "Face";

    int sel_count = 0;
    if (sm == SelectMode::Vertex) sel_count = (int)m_edit_mesh.selected_vertices().size();
    else if (sm == SelectMode::Edge) sel_count = (int)m_edit_mesh.selected_edges().size();
    else sel_count = (int)m_edit_mesh.selected_faces().size();

    ImGui::Text("Mode: %s  |  Sel: %d  |  V: %zu  E: %zu  F: %zu  |  Undo: %d  Redo: %d",
        sm_name, sel_count,
        m_edit_mesh.vertex_count(), m_edit_mesh.edge_count(), m_edit_mesh.triangle_count(),
        (int)m_edit_mesh.can_undo(), (int)m_edit_mesh.can_redo());

    if (!m_error.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "  Err: %s", m_error.c_str());
    }
}

void ModelEditor::handle_hotkeys() {
    if (!m_edit_loaded) return;

    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) return;

    const bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    const bool hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);
    if (!focused && !hovered) return;

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) {
        if (m_edit_mesh.select_mode() == SelectMode::Edge && !m_edit_mesh.selected_edges().empty()) {
            m_edit_mesh.push_undo("Loop Cut");
            m_edit_mesh.loop_cut(*m_edit_mesh.selected_edges().begin(), 0.5f);
            m_vp.mesh_valid = false;
            m_preview_valid = false;
        }
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_G)) {
        m_transform_mode = TransformMode::Translate;
    }
    if (!io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_R)) {
        m_transform_mode = TransformMode::Rotate;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_S)) {
        m_transform_mode = TransformMode::Scale;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_X)) {
        m_axis_constraint = 1;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Y)) {
        m_axis_constraint = 2;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Z)) {
        m_axis_constraint = 3;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F)) {
        frame_selection();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_A)) {
        m_edit_mesh.select_all();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        m_edit_mesh.clear_selection();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_E)) {
        m_edit_mesh.push_undo("Extrude");
        m_edit_mesh.extrude_selected(0.1f);
        m_vp.mesh_valid = false;
        m_preview_valid = false;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_I)) {
        if (m_edit_mesh.select_mode() == SelectMode::Face && !m_edit_mesh.selected_faces().empty()) {
            m_edit_mesh.push_undo("Inset");
            auto sel = std::vector<uint32_t>(m_edit_mesh.selected_faces().begin(),
                                             m_edit_mesh.selected_faces().end());
            m_edit_mesh.inset_faces(sel, 0.1f, 0.0f);
            m_vp.mesh_valid = false;
            m_preview_valid = false;
        }
    }
}

// ============================================================================
// Main render
// ============================================================================
void ModelEditor::render(SimulationOrchestrator& orchestrator) {
    if (!m_loaded) {
        m_lib.load_mapping();
        m_loaded = true;
    }

    // Load effective model
    load_effective_model(orchestrator);

    // Update viewport mesh if needed
    if (m_edit_loaded && !m_vp.mesh_valid) {
        update_viewport_mesh();
    }

    // ---- Layout ----
    handle_hotkeys();

    // ---- Layout ----
    // Full-width toolbar at top
    render_toolbar();
    ImGui::Separator();

    // Main area: tool shelf (left) | viewport (center) | properties (right)
    float shelf_w = 220.0f;
    float props_w = 240.0f;

    // Left: Tool shelf
    ImGui::BeginChild("ToolShelf", ImVec2(shelf_w, 0), true);
    render_tool_shelf();

    // ---- Import / Primitives / Modifiers (bottom of tool shelf) ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Import / Create");
    ImGui::Separator();

    // Import
    static char import_buf[512] = "";
    static char asset_buf[128] = "";
    ImGui::InputText("Path", import_buf, sizeof(import_buf));
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        const std::string selected_file = choose_import_model_file();
        if (!selected_file.empty()) {
            snprintf(import_buf, sizeof(import_buf), "%s", selected_file.c_str());
        }
    }
    ImGui::InputText("Asset ID", asset_buf, sizeof(asset_buf));
    if (ImGui::Button("Import As Asset")) {
        m_error.clear();
        if (import_buf[0] == '\0' || asset_buf[0] == '\0') {
            m_error = "Path and Asset ID required.";
        } else {
            std::string err;
            if (!m_lib.import_as_asset(m_cad, import_buf, asset_buf, &err)) {
                m_error = err;
            } else {
                m_selected_asset_id = asset_buf;
                m_selected_asset_scope = "user";
                m_edit_loaded = false;
                m_error.clear();
            }
        }
    }

    ImGui::Spacing();
    static char export_name_buf[256] = "";
    static int export_type = 0;
    const char* export_types[] = {"STL", "OBJ", "STEP"};
    ImGui::InputText("Export File Name", export_name_buf, sizeof(export_name_buf));
    ImGui::Combo("Export Type", &export_type, export_types, 3);
    ImGui::TextDisabled("Choose the destination folder after pressing Export.");
    if (ImGui::Button("Export Current Mesh...")) {
        m_error.clear();
        const std::string export_name = export_name_buf;
        if (export_name.empty()) {
            m_error = "Export file name required.";
        } else if (!valid_export_name(export_name)) {
            m_error = "Export file name must not contain path separators.";
        } else {
            MeshData out = m_edit_mesh.to_meshdata();
            if (!m_stack.modifiers.empty()) {
                auto loader = [&](const std::string& aid, const std::string& scope, MeshData& mesh) -> bool {
                    return load_asset_mesh_by_id(m_lib, m_cad, aid, scope, mesh);
                };
                MeshData modified;
                std::string err;
                if (m_mod_engine.apply(m_cad, out, m_stack, loader, modified, &err)) {
                    out = std::move(modified);
                } else {
                    m_error = err;
                }
            }
            if (m_error.empty()) {
                const std::string folder = choose_export_folder();
                if (folder.empty()) {
                    m_error.clear();
                } else {
                    const std::filesystem::path export_path = export_path_for(folder, export_name, export_type);
                    bool ok = false;
                    if (export_type == 0) ok = m_cad.export_stl(export_path.string(), out);
                    else if (export_type == 1) ok = m_cad.export_obj(export_path.string(), out);
                    else ok = m_cad.export_step(export_path.string(), out);
                    if (!ok) m_error = m_cad.error().empty() ? "Export failed." : m_cad.error();
                }
            }
        }
    }

    // Primitives
    ImGui::Spacing();
    static int prim = 0;
    static float dim1 = 1.0f, dim2 = 1.0f, dim3 = 1.0f;
    static char prim_buf[128] = "";
    const char* prims[] = {"Box", "Sphere", "Cylinder"};
    ImGui::Combo("Primitive", &prim, prims, 3);
    if (prim == 0) {
        ImGui::DragFloat("W", &dim1, 0.05f, 0.01f, 100.0f);
        ImGui::DragFloat("H", &dim2, 0.05f, 0.01f, 100.0f);
        ImGui::DragFloat("D", &dim3, 0.05f, 0.01f, 100.0f);
    } else if (prim == 1) {
        ImGui::DragFloat("Radius", &dim1, 0.05f, 0.01f, 100.0f);
        int seg = (int)dim2;
        ImGui::DragInt("Segments", &seg, 1, 8, 128);
        dim2 = (float)seg;
    } else {
        ImGui::DragFloat("Radius", &dim1, 0.05f, 0.01f, 100.0f);
        ImGui::DragFloat("Height", &dim2, 0.05f, 0.01f, 100.0f);
        int seg = (int)dim3;
        ImGui::DragInt("Segments", &seg, 1, 8, 128);
        dim3 = (float)seg;
    }
    ImGui::InputText("New Asset ID", prim_buf, sizeof(prim_buf));
    if (ImGui::Button("Create Primitive")) {
        std::string aid = prim_buf;
        if (aid.empty()) { m_error = "Asset ID required."; }
        else {
            std::shared_ptr<Geometry> g;
            if (prim == 0) g = m_cad.create_box(dim1, dim2, dim3);
            else if (prim == 1) g = m_cad.create_sphere(dim1, (int)dim2);
            else g = m_cad.create_cylinder(dim1, dim2, (int)dim3);
            ModelAssetMeta meta;
            meta.id = aid; meta.scope = "user"; meta.label = aid; meta.format = "stl";
            std::string err;
            if (!m_lib.save_mesh_as_asset(m_cad, g->mesh, aid, "user", meta, &err)) {
                m_error = err;
            } else {
                m_selected_asset_id = aid;
                m_selected_asset_scope = "user";
                m_edit_loaded = false;
                m_error.clear();
            }
        }
    }

    ImGui::EndChild(); // ToolShelf

    ImGui::SameLine();

    // Center: Viewport
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float vp_w = avail.x - props_w;
    if (vp_w < 200.0f) vp_w = 200.0f;
    int pw = (int)vp_w;
    int ph = (int)ImGui::GetContentRegionAvail().y - 40; // leave room for status bar
    if (ph < 200) ph = 200;

    ImGui::BeginChild("EditMeshViewport", ImVec2(vp_w, (float)ph), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // Inline viewport controls (live inside the edit mesh area).
    ImGui::SetCursorPos(ImVec2(8.0f, 6.0f));
    ImGui::BeginGroup();
    {
        const char* views_inline[] = {"3D", "2D Top", "2D Front", "2D Side"};
        ImGui::SetNextItemWidth(110);
        if (ImGui::Combo("##vp_view", &m_vp.view_mode, views_inline, 4)) {
            m_vp.mesh_valid = false;
            m_preview_valid = false;
        }
        ImGui::SameLine();
        ImGui::Checkbox("Wire", &m_vp.show_wireframe);
        ImGui::SameLine();
        ImGui::Checkbox("Grid", &m_vp.show_grid);
        ImGui::SameLine();
        ImGui::Checkbox("Snap", &m_snap_enabled);
    }
    ImGui::EndGroup();
    // Reserve header space so the controls don't overlap the model.
    ImGui::Dummy(ImVec2(1.0f, 28.0f));

    ImDrawList* overlay = ImGui::GetWindowDrawList();
    if (m_vp.view_mode == 0) {
        // 3D view
        render_3d_view(pw, ph);
        if (g_view.tex != 0 && g_view.fbo_complete) {
            ImGui::Image((ImTextureID)(uintptr_t)g_view.tex,
                         ImVec2((float)pw, (float)ph), ImVec2(0, 1), ImVec2(1, 0));
        } else {
            // Avoid a "dead black panel" when FBO isn't ready (driver/GL init issues).
            // We still reserve the space and draw a lightweight grid placeholder.
            ImGui::Dummy(ImVec2((float)pw, (float)ph));
            ImVec2 rmin = ImGui::GetItemRectMin();
            ImVec2 rmax = ImGui::GetItemRectMax();
            overlay->AddRectFilled(rmin, rmax, IM_COL32(18, 18, 20, 255));
            const float step = 24.0f;
            for (float x = rmin.x; x < rmax.x; x += step)
                overlay->AddLine(ImVec2(x, rmin.y), ImVec2(x, rmax.y), IM_COL32(40, 45, 55, 120));
            for (float y = rmin.y; y < rmax.y; y += step)
                overlay->AddLine(ImVec2(rmin.x, y), ImVec2(rmax.x, y), IM_COL32(40, 45, 55, 120));
            overlay->AddText(ImVec2(rmin.x + 10.0f, rmin.y + 10.0f),
                             IM_COL32(255, 110, 110, 220),
                             "3D viewport unavailable (framebuffer not ready).");
        }
        bool hov = ImGui::IsItemHovered();
        ImVec2 rect_min = ImGui::GetItemRectMin();
        ImVec2 rect_max = ImGui::GetItemRectMax();
        overlay->AddText(ImVec2(rect_min.x + 8.0f, rect_min.y + 8.0f),
                         IM_COL32(210, 220, 235, 220),
                         ("Edit Mesh 3D - " + m_loaded_mesh_source_label).c_str());
        if (m_edit_loaded) {
            char stats[128];
            snprintf(stats, sizeof(stats), "V:%zu E:%zu F:%zu",
                     m_edit_mesh.vertex_count(), m_edit_mesh.edge_count(), m_edit_mesh.triangle_count());
            overlay->AddText(ImVec2(rect_min.x + 8.0f, rect_min.y + 28.0f),
                             IM_COL32(160, 175, 195, 210), stats);
        }
        overlay->AddRect(rect_min, rect_max, IM_COL32(70, 105, 150, 180));

        // Camera orbit (RMB drag)
        if (hov && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            ImVec2 d = ImGui::GetIO().MouseDelta;
            m_vp.yaw += d.x * 0.01f;
            m_vp.pitch -= d.y * 0.01f;
            m_vp.pitch = std::clamp(m_vp.pitch, -1.4f, 1.4f);
        }
        // Zoom
        if (hov && ImGui::GetIO().MouseWheel != 0.0f) {
            m_vp.dist *= (ImGui::GetIO().MouseWheel > 0) ? 0.9f : 1.1f;
            m_vp.dist = std::clamp(m_vp.dist, 0.05f, 500.0f);
        }
        // Pan (MMB drag)
        if (hov && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
            ImVec2 d = ImGui::GetIO().MouseDelta;
            m_vp.target[0] -= d.x * 0.01f * m_vp.dist * 0.1f;
            m_vp.target[1] += d.y * 0.01f * m_vp.dist * 0.1f;
        }
        ImVec2 mp = ImGui::GetMousePos();
        if (hov && m_active_tool == 2) {
            ImGui::GetWindowDrawList()->AddCircle(mp, m_circle_radius, IM_COL32(255, 220, 80, 180), 48, 1.5f);
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                if (!m_selection_started) {
                    m_selection_started = true;
                    const bool add = ImGui::GetIO().KeyShift;
                    const bool sub = ImGui::GetIO().KeyCtrl;
                    m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
                    m_selection_cleared = false;
                }
                float sx = mp.x - rect_min.x;
                float sy = mp.y - rect_min.y;
                auto project = [this](const Vec3& world, float& out_x, float& out_y) -> bool {
                    return project_to_screen(world, out_x, out_y);
                };
                if (m_select_op == SelectionOp::Replace && !m_selection_cleared) {
                    m_edit_mesh.clear_selection();
                    m_selection_cleared = true;
                }
                const SelectionOp op = (m_select_op == SelectionOp::Replace) ? SelectionOp::Add : m_select_op;
                m_edit_mesh.circle_select(sx, sy, m_circle_radius, op, project);
            } else {
                m_selection_started = false;
                m_selection_cleared = false;
            }
        }

        // Lasso select in projected 3D screen space
        if (hov && m_active_tool == 3) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                m_lasso_selecting = true;
                m_lasso_points.clear();
                const bool add = ImGui::GetIO().KeyShift;
                const bool sub = ImGui::GetIO().KeyCtrl;
                m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
            }
            if (m_lasso_selecting && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                const float rx = mp.x - rect_min.x;
                const float ry = mp.y - rect_min.y;
                if (m_lasso_points.empty()) {
                    m_lasso_points.push_back({rx, ry});
                } else {
                    const auto& last = m_lasso_points.back();
                    const float dx = rx - last[0];
                    const float dy = ry - last[1];
                    if (dx * dx + dy * dy >= 16.0f) {
                        m_lasso_points.push_back({rx, ry});
                    }
                }
            }
            if (m_lasso_selecting && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                auto project = [this](const Vec3& world, float& out_x, float& out_y) -> bool {
                    return project_to_screen(world, out_x, out_y);
                };
                m_edit_mesh.polygon_select(m_lasso_points, m_select_op, project);
                m_lasso_selecting = false;
                m_lasso_points.clear();
            }
        }
        if (m_lasso_selecting && m_lasso_points.size() > 1) {
            for (size_t i = 1; i < m_lasso_points.size(); ++i) {
                overlay->AddLine(ImVec2(rect_min.x + m_lasso_points[i - 1][0], rect_min.y + m_lasso_points[i - 1][1]),
                                 ImVec2(rect_min.x + m_lasso_points[i][0], rect_min.y + m_lasso_points[i][1]),
                                 IM_COL32(255, 220, 80, 210), 1.5f);
            }
        }

        // Click picking in 3D (LMB)
        if (hov && m_active_tool == 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            float sx = mp.x - rect_min.x;
            float sy = mp.y - rect_min.y;
            const bool add = ImGui::GetIO().KeyShift;
            const bool sub = ImGui::GetIO().KeyCtrl;

            SelectMode sm = m_edit_mesh.select_mode();
            if (sm == SelectMode::Vertex) {
                int hit = pick_vertex_3d(sx, sy);
                if (hit >= 0) {
                    if (sub) m_edit_mesh.deselect_vertex((uint32_t)hit);
                    else m_edit_mesh.select_vertex((uint32_t)hit, add);
                }
                else if (!add && !sub) m_edit_mesh.clear_selection();
            } else if (sm == SelectMode::Edge) {
                EdgeKey ek = pick_edge_3d(sx, sy);
                if (ek.v0 != ek.v1) {
                    if (sub) m_edit_mesh.deselect_edge(ek);
                    else m_edit_mesh.select_edge(ek, add);
                }
                else if (!add && !sub) m_edit_mesh.clear_selection();
            } else {
                int hit = pick_face_3d(sx, sy);
                if (hit >= 0) {
                    if (sub) m_edit_mesh.deselect_face((uint32_t)hit);
                    else m_edit_mesh.select_face((uint32_t)hit, add);
                }
                else if (!add && !sub) m_edit_mesh.clear_selection();
            }
        }

        // Box select in 3D
        if (hov && m_active_tool == 1) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                m_box_selecting = true;
                ImVec2 mp = ImGui::GetMousePos();
                ImVec2 rmin = ImGui::GetItemRectMin();
                m_box_start[0] = mp.x - rmin.x;
                m_box_start[1] = mp.y - rmin.y;
                m_box_end[0] = m_box_start[0];
                m_box_end[1] = m_box_start[1];
                const bool add = ImGui::GetIO().KeyShift;
                const bool sub = ImGui::GetIO().KeyCtrl;
                m_select_op = sub ? SelectionOp::Subtract : (add ? SelectionOp::Add : SelectionOp::Replace);
            }
        }
        if (m_box_selecting) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                ImVec2 mp = ImGui::GetMousePos();
                ImVec2 rmin = ImGui::GetItemRectMin();
                m_box_end[0] = mp.x - rmin.x;
                m_box_end[1] = mp.y - rmin.y;
            } else {
                m_box_selecting = false;
                // Apply box select
                auto proj = [this](const Vec3& w, float& sx, float& sy) -> bool {
                    return project_to_screen(w, sx, sy);
                };
                m_edit_mesh.box_select(m_box_start[0], m_box_start[1],
                                       m_box_end[0], m_box_end[1], m_select_op, proj);
            }
            ImVec2 bs(rect_min.x + m_box_start[0], rect_min.y + m_box_start[1]);
            ImVec2 be(rect_min.x + m_box_end[0], rect_min.y + m_box_end[1]);
            overlay->AddRect(bs, be, IM_COL32(255, 220, 80, 180), 0.0f, 0, 1.5f);
        }
    } else {
        // 2D view
        render_2d_view(pw, ph);
    }

    // Frame button
    ImGui::SetCursorPos(ImVec2(ImGui::GetWindowContentRegionMin().x + 4, ImGui::GetWindowContentRegionMin().y + 4));
    if (ImGui::Button("Frame")) {
        frame_current_mesh();
    }
    ImGui::SameLine();
    if (ImGui::Button("Frame Sel")) {
        frame_selection();
    }

    ImGui::EndChild(); // EditMeshViewport

    ImGui::SameLine();

    // Right: Properties
    ImGui::BeginChild("Properties", ImVec2(props_w, 0), true);
    render_properties_panel();

    // ---- Modifier Stack ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Modifiers");
    ImGui::Separator();
    if (ImGui::Checkbox("Preview Stack", &m_show_modifier_preview)) {
        m_vp.mesh_valid = false;
    }
    if (!m_preview_error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.35f, 0.35f, 1), "%s", m_preview_error.c_str());
    }

    // Add modifier combo
    static int mod_type = 0;
    const char* mod_names[] = {"Boolean", "Mirror", "Subdivision", "Bevel", "Weld", "Array", "Solidify"};
    ImGui::SetNextItemWidth(120);
    ImGui::Combo("Type", &mod_type, mod_names, 7);

    if (mod_type == 0) { // Boolean
        static int bool_op = 0;
        static char other_buf[128] = "";
        const char* ops[] = {"Union", "Subtract", "Intersect"};
        ImGui::Combo("Op", &bool_op, ops, 3);
        ImGui::InputText("Other Asset", other_buf, sizeof(other_buf));
        if (ImGui::Button("+ Add Boolean")) {
            ModifierBoolean b;
            b.op = (bool_op == 1) ? ModifierBoolean::Op::Subtract :
                   (bool_op == 2) ? ModifierBoolean::Op::Intersect :
                                    ModifierBoolean::Op::Union;
            b.other_asset_id = other_buf;
            m_stack.modifiers.push_back(b);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else if (mod_type == 1) { // Mirror
        static bool mx = true, my = false, mz = false;
        ImGui::Checkbox("X", &mx); ImGui::SameLine();
        ImGui::Checkbox("Y", &my); ImGui::SameLine();
        ImGui::Checkbox("Z", &mz);
        if (ImGui::Button("+ Add Mirror")) {
            ModifierMirror mm;
            mm.x = mx; mm.y = my; mm.z = mz;
            m_stack.modifiers.push_back(mm);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else if (mod_type == 2) { // Subdivision
        static int subd_level = 1;
        ImGui::DragInt("Levels", &subd_level, 1, 1, 6);
        if (ImGui::Button("+ Add Subdivision")) {
            ModifierSubdivision s;
            s.levels = subd_level;
            m_stack.modifiers.push_back(s);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else if (mod_type == 3) { // Bevel
        static float bevel_a = 0.05f;
        ImGui::DragFloat("Amount", &bevel_a, 0.005f, 0.001f, 5.0f);
        if (ImGui::Button("+ Add Bevel")) {
            ModifierBevel b;
            b.amount = bevel_a;
            m_stack.modifiers.push_back(b);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else if (mod_type == 4) { // Weld
        static float weld_threshold = 0.0001f;
        ImGui::DragFloat("Threshold", &weld_threshold, 0.0001f, 0.000001f, 1.0f, "%.6f");
        if (ImGui::Button("+ Add Weld")) {
            ModifierWeld w;
            w.threshold = weld_threshold;
            m_stack.modifiers.push_back(w);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else if (mod_type == 5) { // Array
        static int array_count = 2;
        static float array_offset[3] = {1.0f, 0.0f, 0.0f};
        ImGui::DragInt("Count", &array_count, 1, 1, 256);
        ImGui::DragFloat3("Offset", array_offset, 0.01f);
        if (ImGui::Button("+ Add Array")) {
            ModifierArray a;
            a.count = array_count;
            a.offset = Vec3{array_offset[0], array_offset[1], array_offset[2]};
            m_stack.modifiers.push_back(a);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    } else { // Solidify
        static float solid_thickness = 0.05f;
        static float solid_offset = 1.0f;
        ImGui::DragFloat("Thickness", &solid_thickness, 0.005f, 0.0001f, 10.0f);
        ImGui::DragFloat("Offset Factor", &solid_offset, 0.01f, -1.0f, 1.0f);
        if (ImGui::Button("+ Add Solidify")) {
            ModifierSolidify s;
            s.thickness = solid_thickness;
            s.offset = solid_offset;
            m_stack.modifiers.push_back(s);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    }

    // List current modifiers
    for (int i = 0; i < (int)m_stack.modifiers.size(); ++i) {
        auto& m = m_stack.modifiers[i];
        char label[64];
        if (std::holds_alternative<ModifierBoolean>(m))
            snprintf(label, sizeof(label), "%d: Boolean", i);
        else if (std::holds_alternative<ModifierMirror>(m))
            snprintf(label, sizeof(label), "%d: Mirror", i);
        else if (std::holds_alternative<ModifierSubdivision>(m))
            snprintf(label, sizeof(label), "%d: Subdivision", i);
        else if (std::holds_alternative<ModifierBevel>(m))
            snprintf(label, sizeof(label), "%d: Bevel", i);
        else if (std::holds_alternative<ModifierWeld>(m))
            snprintf(label, sizeof(label), "%d: Weld", i);
        else if (std::holds_alternative<ModifierArray>(m))
            snprintf(label, sizeof(label), "%d: Array", i);
        else
            snprintf(label, sizeof(label), "%d: Solidify", i);

        ImGui::PushID(i);
        if (ImGui::SmallButton("X")) {
            m_stack.modifiers.erase(m_stack.modifiers.begin() + i);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
            ImGui::PopID();
            break;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("^") && i > 0) {
            std::swap(m_stack.modifiers[i], m_stack.modifiers[i - 1]);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
            ImGui::PopID();
            break;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("v") && i + 1 < (int)m_stack.modifiers.size()) {
            std::swap(m_stack.modifiers[i], m_stack.modifiers[i + 1]);
            m_preview_valid = false;
            m_vp.mesh_valid = false;
            ImGui::PopID();
            break;
        }
        ImGui::SameLine();
        ImGui::Text("%s", label);
        ImGui::PopID();
    }

    if (!m_stack.modifiers.empty()) {
        if (ImGui::Button("Apply Modifiers")) {
            m_edit_mesh.push_undo("Apply Modifiers");
            MeshData base = m_edit_mesh.to_meshdata();
            auto loader = [&](const std::string& aid, const std::string& scope, MeshData& out) -> bool {
                return load_asset_mesh_by_id(m_lib, m_cad, aid, scope, out);
            };
            MeshData result;
            std::string err;
            if (m_mod_engine.apply(m_cad, base, m_stack, loader, result, &err)) {
                m_edit_mesh.from_meshdata(result);
                m_stack.modifiers.clear();
                m_preview_valid = false;
                m_vp.mesh_valid = false;
            } else {
                m_error = err;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Stack")) {
            m_stack.modifiers.clear();
            m_preview_valid = false;
            m_vp.mesh_valid = false;
        }
    }

    // ---- Component Assignment ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Component Mapping");
    ImGui::Separator();

    std::vector<std::string> types;
    auto add_type = [&](const std::string& t) {
        if (std::find(types.begin(), types.end(), t) == types.end()) types.push_back(t);
    };
    orchestrator.registry().for_each([&](Component& c) {
        add_type(std::string(c.component_type()));
    });
    for (const auto& t : SystemConfig::instance().all_type_ids()) {
        add_type(t);
    }
    if (types.empty()) {
        m_selected_component_type.clear();
        ImGui::TextDisabled("No component types available.");
    } else if (m_selected_component_type.empty()) {
        m_selected_component_type = types.front();
    }
    if (!types.empty()) {
        const char* preview = m_selected_component_type.c_str();
        if (ImGui::BeginCombo("Comp Type", preview)) {
            for (auto& t : types) {
                bool sel = (t == m_selected_component_type);
                if (ImGui::Selectable(t.c_str(), sel)) {
                    m_selected_component_type = t;
                    m_force_fallback_box = false;
                    ModelAssetLibrary::MappingEntry effective{};
                    if (m_lib.resolve_effective_asset_for_component(m_selected_component_type, effective)) {
                        m_selected_asset_id = effective.asset_id;
                        m_selected_asset_scope = effective.scope;
                    } else {
                        m_selected_asset_id.clear();
                        m_selected_asset_scope = "user";
                    }
                    m_edit_loaded = false;
                }
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    auto assets = m_lib.list_assets();
    std::string asset_preview = m_force_fallback_box
        ? std::string("(new editable box override)")
        : m_selected_asset_id.empty()
        ? std::string("(component default / fallback box)")
        : (m_selected_asset_id + " [" + m_selected_asset_scope + "]");
    const char* ap = asset_preview.c_str();
    if (ImGui::BeginCombo("Asset", ap)) {
        if (ImGui::Selectable("(component default / fallback box)", m_selected_asset_id.empty() && !m_force_fallback_box)) {
            m_selected_asset_id.clear();
            m_selected_asset_scope = "user";
            m_force_fallback_box = false;
            m_edit_loaded = false;
        }
        if (ImGui::Selectable("(new editable box override)", m_force_fallback_box)) {
            m_selected_asset_id.clear();
            m_selected_asset_scope = "user";
            m_force_fallback_box = true;
            m_edit_loaded = false;
        }
        for (auto& a : assets) {
            const std::string scope = a.scope.empty() ? "user" : a.scope;
            bool sel = (a.id == m_selected_asset_id && scope == m_selected_asset_scope);
            std::string label = a.id + " [" + scope + "] (" + a.format + ")";
            if (ImGui::Selectable(label.c_str(), sel)) {
                m_selected_asset_id = a.id;
                m_selected_asset_scope = scope;
                m_force_fallback_box = false;
                m_edit_loaded = false;
            }
            if (sel) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (ImGui::Button("Assign")) {
        if (!m_selected_component_type.empty() && !m_selected_asset_id.empty()) {
            m_lib.set_component_asset(m_selected_component_type, m_selected_asset_id, m_selected_asset_scope);
            m_lib.save_mapping();
            m_edit_loaded = false;
        } else if (!m_selected_component_type.empty()) {
            m_error = "Select an explicit asset to assign, or save this fallback as a user override.";
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        if (!m_selected_component_type.empty()) {
            m_lib.clear_component_asset(m_selected_component_type);
            m_lib.save_mapping();
            m_selected_asset_id.clear();
            m_selected_asset_scope = "user";
            m_force_fallback_box = false;
            m_edit_loaded = false;
        }
    }

    // Save buttons
    ImGui::Spacing();
    ImGui::Separator();
    if (ImGui::Button("Save As User Override")) {
        if (m_selected_component_type.empty()) {
            m_error = "No component type selected.";
        } else {
            MeshData out = m_edit_mesh.to_meshdata();
            ModelAssetMeta meta;
            meta.id = m_selected_component_type;
            meta.scope = "user";
            meta.label = m_selected_component_type;
            meta.format = "stl";
            meta.modifiers = m_stack;
            std::string err;
            if (!m_lib.save_mesh_as_asset(m_cad, out, m_selected_component_type, "user", meta, &err))
                m_error = err;
            else {
                m_lib.set_component_asset(m_selected_component_type, m_selected_component_type, "user");
                m_lib.save_mapping();
                m_selected_asset_id = m_selected_component_type;
                m_selected_asset_scope = "user";
                m_force_fallback_box = false;
                m_error.clear();
                m_edit_loaded = false; // force reload
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save To Asset")) {
        if (m_selected_asset_id.empty()) {
            m_error = "No asset selected.";
        } else if (m_selected_asset_scope == "default") {
            m_error = "Default model assets are read-only. Use Save As User Override.";
        } else {
            MeshData out = m_edit_mesh.to_meshdata();
            ModelAssetMeta meta;
            std::string err;
            if (!m_lib.load_asset_meta(m_selected_asset_id, m_selected_asset_scope, meta, &err)) {
                meta.id = m_selected_asset_id;
                meta.scope = m_selected_asset_scope;
                meta.label = m_selected_asset_id;
                meta.format = "stl";
            }
            meta.modifiers = m_stack;
            if (!m_lib.save_mesh_as_asset(m_cad, out, m_selected_asset_id, m_selected_asset_scope, meta, &err))
                m_error = err;
            else {
                m_error.clear();
                m_edit_loaded = false;
            }
        }
    }

    if (!m_selected_component_type.empty() && m_lib.asset_exists(m_selected_component_type, "user")) {
        if (ImGui::Button("Reset User Override")) {
            std::string err;
            if (!m_lib.delete_user_asset(m_selected_component_type, &err)) {
                m_error = err;
            } else {
                auto mapped = m_lib.get_component_asset(m_selected_component_type);
                if (mapped && mapped->scope == "user" && mapped->asset_id == m_selected_component_type) {
                    m_lib.clear_component_asset(m_selected_component_type);
                    m_lib.save_mapping();
                }
                m_selected_asset_id.clear();
                m_selected_asset_scope = "user";
                m_force_fallback_box = false;
                m_edit_loaded = false;
                m_error.clear();
            }
        }
    }

    if (!m_error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", m_error.c_str());
    }

    ImGui::EndChild(); // Properties

    // Status bar at bottom
    render_status_bar();
}

} // namespace mechatron
