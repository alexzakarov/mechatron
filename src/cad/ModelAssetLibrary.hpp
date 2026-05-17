#pragma once

#include "CADKernel.hpp"
#include "Modifiers.hpp"
#include <string>
#include <unordered_map>
#include <optional>
#include <vector>
#include <cstdint>

namespace mechatron {

struct ModelAssetMeta {
    std::string id;            // asset id (folder name)
    std::string scope;         // "default" | "user" (where this asset lives)
    std::string label;         // user-visible name
    std::string source_path;   // original import path (optional)
    std::string format;        // "stl" | "step" | ...
    ModifierStack modifiers;   // optional modifier stack (stored in meta.json)
};

// Stores user/imported models on disk and maps component types to model assets.
// Layout:
//   assets/models_default/<asset_id>/{meta.json, mesh.stl}
//   assets/models_user/<asset_id>/{meta.json, mesh.stl}
//   assets/component_model_map.json   (component_type -> {asset_id, scope})
class ModelAssetLibrary {
public:
    static const char* default_dir(); // assets/models_default
    static const char* user_dir();    // assets/models_user
    static const char* mapping_file(); // assets/component_model_map.json

    bool load_mapping();
    bool save_mapping() const;
    uint64_t mapping_revision() const;

    // Enumerate assets present on disk (both default + user).
    std::vector<ModelAssetMeta> list_assets() const;

    // Import a mesh file using CADKernel and save as an asset (STL + meta).
    bool import_as_asset(CADKernel& cad, const std::string& src_path, const std::string& asset_id, std::string* out_error = nullptr);

    // Create/overwrite an asset from an in-memory mesh.
    // scope is "user" or "default".
    bool save_mesh_as_asset(CADKernel& cad, const MeshData& mesh, const std::string& asset_id, const std::string& scope, const ModelAssetMeta& meta, std::string* out_error = nullptr);
    // Convenience: save to user scope.
    bool save_mesh_as_asset(CADKernel& cad, const MeshData& mesh, const std::string& asset_id, const ModelAssetMeta& meta, std::string* out_error = nullptr) {
        return save_mesh_as_asset(cad, mesh, asset_id, "user", meta, out_error);
    }

    // Load asset meta / mesh directly by id+scope.
    bool load_asset_meta(const std::string& asset_id, const std::string& scope, ModelAssetMeta& out_meta, std::string* out_error = nullptr) const;
    bool load_asset_mesh(CADKernel& cad, const std::string& asset_id, const std::string& scope, MeshData& out_mesh, std::string* out_error = nullptr) const;
    bool delete_user_asset(const std::string& asset_id, std::string* out_error = nullptr);

    // Assign a component type to an asset id. scope is "user" or "default".
    void set_component_asset(const std::string& component_type, const std::string& asset_id, const std::string& scope);
    void clear_component_asset(const std::string& component_type);

    struct MappingEntry {
        std::string asset_id;
        std::string scope; // "user" | "default"
    };
    std::optional<MappingEntry> get_component_asset(const std::string& component_type) const;

    // Load mesh for a component type (if mapped). Returns false if not found.
    bool load_mesh_for_component(CADKernel& cad, const std::string& component_type, MeshData& out_mesh, std::string* out_error = nullptr) const;

    // Returns true if meta.json exists for asset_id in the scope.
    bool asset_exists(const std::string& asset_id, const std::string& scope) const;

    // Monotonic-ish disk revision for cache keys. Includes mesh + metadata mtimes.
    uint64_t asset_revision(const std::string& asset_id, const std::string& scope) const;

    // Resolves "effective" asset for a component type:
    // 1) explicit mapping file
    // 2) user override asset with id == component_type
    // 3) default asset with id == component_type
    // Returns false if none exist.
    bool resolve_effective_asset_for_component(const std::string& component_type, MappingEntry& out_entry) const;

private:
    std::unordered_map<std::string, MappingEntry> m_component_to_asset;
};

} // namespace mechatron
