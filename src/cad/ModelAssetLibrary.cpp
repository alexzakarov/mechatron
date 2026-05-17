#include "ModelAssetLibrary.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <cctype>

namespace mechatron {

const char* ModelAssetLibrary::default_dir() { return "assets/models_default"; }
const char* ModelAssetLibrary::user_dir() { return "assets/models_user"; }
const char* ModelAssetLibrary::mapping_file() { return "assets/component_model_map.json"; }

namespace {

std::filesystem::path resolve_workspace_path(const char* relative) {
    namespace fs = std::filesystem;
    const fs::path rel(relative);
    if (rel.is_absolute()) return rel;

    std::error_code ec;
    fs::path current = fs::current_path(ec);
    if (ec) return rel;

    fs::path packaged_assets_root;
    for (fs::path p = current; !p.empty(); p = p.parent_path()) {
        if (packaged_assets_root.empty() && fs::exists(p / "assets", ec)) {
            packaged_assets_root = p;
        }

        // Prefer the source/project root during development, even when the app
        // is launched from build/bin where a copied assets directory also exists.
        if (fs::exists(p / "CMakeLists.txt", ec) && fs::exists(p / "src", ec) && fs::exists(p / "assets", ec)) {
            return p / rel;
        }

        if (p == p.root_path()) break;
    }

    if (!packaged_assets_root.empty()) return packaged_assets_root / rel;
    return rel;
}

std::filesystem::path default_root() {
    return resolve_workspace_path(ModelAssetLibrary::default_dir());
}

std::filesystem::path user_root() {
    return resolve_workspace_path(ModelAssetLibrary::user_dir());
}

std::filesystem::path mapping_path() {
    return resolve_workspace_path(ModelAssetLibrary::mapping_file());
}

std::filesystem::path scoped_root(const std::string& scope) {
    return scope == "default" ? default_root() : user_root();
}

bool is_safe_asset_id(const std::string& asset_id) {
    if (asset_id.empty() || asset_id == "." || asset_id == "..") return false;
    if (asset_id.find('/') != std::string::npos || asset_id.find('\\') != std::string::npos) return false;
    if (asset_id.find("..") != std::string::npos) return false;
    for (unsigned char c : asset_id) {
        if (std::isalnum(c) || c == '_' || c == '-' || c == '.') continue;
        return false;
    }
    return true;
}

bool require_safe_asset_id(const std::string& asset_id, std::string* out_error) {
    if (is_safe_asset_id(asset_id)) return true;
    if (out_error) *out_error = "Invalid asset id. Use letters, numbers, '.', '_' or '-' only.";
    return false;
}

bool validate_mesh_indices(const MeshData& mesh, std::string* out_error) {
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        if (out_error) *out_error = "Mesh is empty.";
        return false;
    }

    const uint32_t vertex_count = static_cast<uint32_t>(mesh.vertices.size());
    for (size_t i = 0; i < mesh.triangles.size(); ++i) {
        const Triangle& t = mesh.triangles[i];
        if (t.v0 >= vertex_count || t.v1 >= vertex_count || t.v2 >= vertex_count) {
            if (out_error) *out_error = "Mesh contains a triangle with an invalid vertex index.";
            return false;
        }
        if (t.v0 == t.v1 || t.v1 == t.v2 || t.v2 == t.v0) {
            if (out_error) *out_error = "Mesh contains a degenerate triangle.";
            return false;
        }
    }
    return true;
}

} // namespace

bool ModelAssetLibrary::load_mapping() {
    m_component_to_asset.clear();
    std::ifstream f(mapping_path());
    if (!f.is_open()) return true; // optional file

    nlohmann::json j;
    try {
        f >> j;
    } catch (...) {
        return false;
    }
    if (!j.is_object()) return false;

    for (auto it = j.begin(); it != j.end(); ++it) {
        if (!it.value().is_object()) continue;
        MappingEntry e;
        e.asset_id = it.value().value("asset_id", "");
        e.scope = it.value().value("scope", "user");
        if (!e.asset_id.empty()) {
            m_component_to_asset[it.key()] = e;
        }
    }
    return true;
}

bool ModelAssetLibrary::save_mapping() const {
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [ctype, e] : m_component_to_asset) {
        j[ctype] = {{"asset_id", e.asset_id}, {"scope", e.scope}};
    }
    std::error_code ec;
    std::filesystem::create_directories(mapping_path().parent_path(), ec);
    std::ofstream f(mapping_path());
    if (!f.is_open()) return false;
    f << j.dump(2);
    return true;
}

uint64_t ModelAssetLibrary::mapping_revision() const {
    std::error_code ec;
    auto t = std::filesystem::last_write_time(mapping_path(), ec);
    if (ec) return 0;
    return static_cast<uint64_t>(t.time_since_epoch().count());
}

std::vector<ModelAssetMeta> ModelAssetLibrary::list_assets() const {
    namespace fs = std::filesystem;
    std::vector<ModelAssetMeta> out;
    auto scan = [&](const fs::path& base, const std::string& scope) {
        std::error_code ec;
        if (!fs::exists(base, ec)) return;
        for (auto& entry : fs::directory_iterator(base, ec)) {
            if (ec) break;
            if (!entry.is_directory()) continue;
            const fs::path meta = entry.path() / "meta.json";
            std::ifstream f(meta.string());
            if (!f.is_open()) continue;
            nlohmann::json j;
            try { f >> j; } catch (...) { continue; }
            ModelAssetMeta m;
            m.id = j.value("id", entry.path().filename().string());
            m.scope = scope;
            m.label = j.value("label", m.id);
            m.source_path = j.value("source_path", "");
            m.format = j.value("format", "stl");
            // modifiers are optional; UI can load them via load_asset_meta
            out.push_back(std::move(m));
        }
    };
    scan(default_root(), "default");
    scan(user_root(), "user");
    return out;
}

static std::string lower_ext(const std::string& path) {
    auto dot = path.find_last_of('.');
    if (dot == std::string::npos) return "";
    std::string ext = path.substr(dot + 1);
    for (auto& c : ext) c = (char)std::tolower((unsigned char)c);
    return ext;
}

bool ModelAssetLibrary::import_as_asset(CADKernel& cad, const std::string& src_path, const std::string& asset_id, std::string* out_error) {
    namespace fs = std::filesystem;
    if (!require_safe_asset_id(asset_id, out_error)) return false;
    std::error_code ec;
    fs::create_directories(user_root(), ec);
    const fs::path asset_dir = user_root() / asset_id;
    fs::create_directories(asset_dir, ec);

    MeshData mesh;
    const std::string ext = lower_ext(src_path);
    bool ok = false;
    if (ext == "stl") ok = cad.import_stl(src_path, mesh);
    else if (ext == "step" || ext == "stp") ok = cad.import_step(src_path, mesh);
    else if (ext == "iges" || ext == "igs") ok = cad.import_iges(src_path, mesh);
    else if (ext == "brep") ok = cad.import_brep(src_path, mesh);
    else if (ext == "obj") ok = cad.import_obj(src_path, mesh);

    if (!ok || mesh.is_empty()) {
        if (out_error) *out_error = cad.error().empty() ? "Import failed" : cad.error();
        return false;
    }

    // Normalize mesh
    mesh.unify_vertices();
    mesh.calculate_normals();

    const fs::path out_mesh = asset_dir / "mesh.stl";
    if (!cad.export_stl(out_mesh.string(), mesh)) {
        if (out_error) *out_error = cad.error().empty() ? "Export failed" : cad.error();
        return false;
    }

    ModelAssetMeta meta;
    meta.id = asset_id;
    meta.scope = "user";
    meta.label = asset_id;
    meta.source_path = src_path;
    meta.format = "stl";
    if (!save_mesh_as_asset(cad, mesh, asset_id, "user", meta, out_error)) {
        return false;
    }
    return true;
}

static nlohmann::json modifier_to_json(const Modifier& m) {
    nlohmann::json j;
    if (std::holds_alternative<ModifierBoolean>(m)) {
        const auto& b = std::get<ModifierBoolean>(m);
        j["type"] = "boolean";
        j["op"] = (b.op == ModifierBoolean::Op::Subtract) ? "subtract" : (b.op == ModifierBoolean::Op::Intersect) ? "intersect" : "union";
        j["other_asset_id"] = b.other_asset_id;
        j["other_scope"] = b.other_scope;
        return j;
    }
    if (std::holds_alternative<ModifierMirror>(m)) {
        const auto& mm = std::get<ModifierMirror>(m);
        j["type"] = "mirror";
        j["x"] = mm.x; j["y"] = mm.y; j["z"] = mm.z;
        return j;
    }
    if (std::holds_alternative<ModifierSubdivision>(m)) {
        const auto& s = std::get<ModifierSubdivision>(m);
        j["type"] = "subdivision";
        j["levels"] = s.levels;
        return j;
    }
    if (std::holds_alternative<ModifierBevel>(m)) {
        const auto& b = std::get<ModifierBevel>(m);
        j["type"] = "bevel";
        j["amount"] = b.amount;
        return j;
    }
    if (std::holds_alternative<ModifierWeld>(m)) {
        const auto& w = std::get<ModifierWeld>(m);
        j["type"] = "weld";
        j["threshold"] = w.threshold;
        return j;
    }
    if (std::holds_alternative<ModifierArray>(m)) {
        const auto& a = std::get<ModifierArray>(m);
        j["type"] = "array";
        j["count"] = a.count;
        j["offset"] = {a.offset.x, a.offset.y, a.offset.z};
        return j;
    }
    const auto& s = std::get<ModifierSolidify>(m);
    j["type"] = "solidify";
    j["thickness"] = s.thickness;
    j["offset_factor"] = s.offset;
    return j;
}

static bool json_to_modifier(const nlohmann::json& j, Modifier& out) {
    if (!j.is_object()) return false;
    const std::string type = j.value("type", "");
    if (type == "boolean") {
        ModifierBoolean b;
        const std::string op = j.value("op", "union");
        b.op = (op == "subtract") ? ModifierBoolean::Op::Subtract : (op == "intersect") ? ModifierBoolean::Op::Intersect : ModifierBoolean::Op::Union;
        b.other_asset_id = j.value("other_asset_id", "");
        b.other_scope = j.value("other_scope", "user");
        out = b;
        return true;
    }
    if (type == "mirror") {
        ModifierMirror m;
        m.x = j.value("x", true);
        m.y = j.value("y", false);
        m.z = j.value("z", false);
        out = m;
        return true;
    }
    if (type == "subdivision") {
        ModifierSubdivision s;
        s.levels = j.value("levels", 1);
        out = s;
        return true;
    }
    if (type == "bevel") {
        ModifierBevel b;
        b.amount = j.value("amount", 0.05f);
        out = b;
        return true;
    }
    if (type == "weld") {
        ModifierWeld w;
        w.threshold = j.value("threshold", 0.0001f);
        out = w;
        return true;
    }
    if (type == "array") {
        ModifierArray a;
        a.count = j.value("count", 2);
        if (j.contains("offset") && j["offset"].is_array() && j["offset"].size() >= 3) {
            a.offset = Vec3{
                j["offset"][0].get<float>(),
                j["offset"][1].get<float>(),
                j["offset"][2].get<float>()
            };
        }
        out = a;
        return true;
    }
    if (type == "solidify") {
        ModifierSolidify s;
        s.thickness = j.value("thickness", 0.05f);
        s.offset = j.value("offset_factor", 1.0f);
        out = s;
        return true;
    }
    return false;
}

bool ModelAssetLibrary::save_mesh_as_asset(CADKernel& cad, const MeshData& mesh_in, const std::string& asset_id, const std::string& scope, const ModelAssetMeta& meta_in, std::string* out_error) {
    namespace fs = std::filesystem;
    if (!require_safe_asset_id(asset_id, out_error)) return false;
    std::error_code ec;
    const fs::path base = scoped_root(scope);
    fs::create_directories(base, ec);
    const fs::path asset_dir = base / asset_id;
    fs::create_directories(asset_dir, ec);

    if (!validate_mesh_indices(mesh_in, out_error)) return false;

    MeshData mesh = mesh_in;
    if (mesh.normals.empty()) mesh.calculate_normals();

    const fs::path out_mesh = asset_dir / "mesh.stl";
    if (!cad.export_stl(out_mesh.string(), mesh)) {
        if (out_error) *out_error = cad.error().empty() ? "Export failed" : cad.error();
        return false;
    }

    nlohmann::json meta;
    meta["id"] = meta_in.id.empty() ? asset_id : meta_in.id;
    meta["scope"] = scope;
    meta["label"] = meta_in.label.empty() ? asset_id : meta_in.label;
    meta["source_path"] = meta_in.source_path;
    meta["format"] = meta_in.format.empty() ? "stl" : meta_in.format;
    meta["modifiers"] = nlohmann::json::array();
    for (const auto& m : meta_in.modifiers.modifiers) {
        meta["modifiers"].push_back(modifier_to_json(m));
    }

    std::ofstream mf((asset_dir / "meta.json").string());
    if (!mf.is_open()) {
        if (out_error) *out_error = "Failed to write meta.json";
        return false;
    }
    mf << meta.dump(2);
    return true;
}

bool ModelAssetLibrary::load_asset_meta(const std::string& asset_id, const std::string& scope, ModelAssetMeta& out_meta, std::string* out_error) const {
    namespace fs = std::filesystem;
    if (!require_safe_asset_id(asset_id, out_error)) return false;
    const fs::path base = scoped_root(scope);
    const fs::path meta = base / asset_id / "meta.json";
    std::ifstream f(meta.string());
    if (!f.is_open()) {
        if (out_error) *out_error = "meta.json not found";
        return false;
    }
    nlohmann::json j;
    try { f >> j; } catch (...) {
        if (out_error) *out_error = "meta.json parse failed";
        return false;
    }

    out_meta = ModelAssetMeta{};
    out_meta.id = j.value("id", asset_id);
    out_meta.scope = scope;
    out_meta.label = j.value("label", out_meta.id);
    out_meta.source_path = j.value("source_path", "");
    out_meta.format = j.value("format", "stl");
    if (j.contains("modifiers") && j["modifiers"].is_array()) {
        for (const auto& mj : j["modifiers"]) {
            Modifier m;
            if (json_to_modifier(mj, m)) out_meta.modifiers.modifiers.push_back(std::move(m));
        }
    }
    return true;
}

bool ModelAssetLibrary::load_asset_mesh(CADKernel& cad, const std::string& asset_id, const std::string& scope, MeshData& out_mesh, std::string* out_error) const {
    namespace fs = std::filesystem;
    if (!require_safe_asset_id(asset_id, out_error)) return false;
    const fs::path base = scoped_root(scope);
    const fs::path stl = base / asset_id / "mesh.stl";
    std::error_code ec;
    if (!fs::exists(stl, ec)) {
        if (out_error) *out_error = "mesh.stl missing";
        return false;
    }
    if (!cad.import_stl(stl.string(), out_mesh)) {
        if (out_error) *out_error = cad.error().empty() ? "Failed to load mesh.stl" : cad.error();
        return false;
    }
    if (!validate_mesh_indices(out_mesh, out_error)) return false;
    if (out_mesh.normals.empty()) out_mesh.calculate_normals();
    return true;
}

bool ModelAssetLibrary::delete_user_asset(const std::string& asset_id, std::string* out_error) {
    namespace fs = std::filesystem;
    if (!require_safe_asset_id(asset_id, out_error)) return false;

    const fs::path asset_dir = user_root() / asset_id;
    std::error_code ec;
    if (!fs::exists(asset_dir, ec)) return true;
    fs::remove_all(asset_dir, ec);
    if (ec) {
        if (out_error) *out_error = "Failed to delete user asset: " + ec.message();
        return false;
    }
    return true;
}

void ModelAssetLibrary::set_component_asset(const std::string& component_type, const std::string& asset_id, const std::string& scope) {
    m_component_to_asset[component_type] = MappingEntry{asset_id, scope};
}

void ModelAssetLibrary::clear_component_asset(const std::string& component_type) {
    m_component_to_asset.erase(component_type);
}

std::optional<ModelAssetLibrary::MappingEntry> ModelAssetLibrary::get_component_asset(const std::string& component_type) const {
    auto it = m_component_to_asset.find(component_type);
    if (it == m_component_to_asset.end()) return std::nullopt;
    return it->second;
}

bool ModelAssetLibrary::load_mesh_for_component(CADKernel& cad, const std::string& component_type, MeshData& out_mesh, std::string* out_error) const {
    MappingEntry effective{};
    if (!resolve_effective_asset_for_component(component_type, effective)) {
        if (out_error) *out_error = "No model asset mapped for component type: " + component_type;
        return false;
    }
    return load_asset_mesh(cad, effective.asset_id, effective.scope, out_mesh, out_error);
}

uint64_t ModelAssetLibrary::asset_revision(const std::string& asset_id, const std::string& scope) const {
    namespace fs = std::filesystem;
    if (!is_safe_asset_id(asset_id)) return 0;
    const fs::path base = scoped_root(scope);
    const fs::path asset_dir = base / asset_id;
    const fs::path mesh = asset_dir / "mesh.stl";
    const fs::path meta = asset_dir / "meta.json";

    auto stamp = [](const fs::path& path) -> uint64_t {
        std::error_code ec;
        auto t = fs::last_write_time(path, ec);
        if (ec) return 0;
        return static_cast<uint64_t>(t.time_since_epoch().count());
    };

    uint64_t h = 1469598103934665603ull;
    auto mix = [&](uint64_t value) {
        h ^= value;
        h *= 1099511628211ull;
    };
    mix(stamp(mesh));
    mix(stamp(meta));
    return h;
}

bool ModelAssetLibrary::asset_exists(const std::string& asset_id, const std::string& scope) const {
    namespace fs = std::filesystem;
    if (!is_safe_asset_id(asset_id)) return false;
    const fs::path base = scoped_root(scope);
    const fs::path meta = base / asset_id / "meta.json";
    std::error_code ec;
    return fs::exists(meta, ec);
}

bool ModelAssetLibrary::resolve_effective_asset_for_component(const std::string& component_type, MappingEntry& out_entry) const {
    if (auto m = get_component_asset(component_type)) {
        out_entry = *m;
        return true;
    }
    // Convention: component_type may have an implicit asset with the same id.
    if (asset_exists(component_type, "user")) {
        out_entry = MappingEntry{component_type, "user"};
        return true;
    }
    if (asset_exists(component_type, "default")) {
        out_entry = MappingEntry{component_type, "default"};
        return true;
    }
    return false;
}

} // namespace mechatron
