#pragma once

#include "CADKernel.hpp"
#include "Modifiers.hpp"
#include <functional>

namespace mechatron {

// Applies a modifier stack to a mesh:
// - Boolean: uses CADKernel boolean ops (OpenCASCADE-backed when available)
// - Mirror/Subdivision/Bevel: mesh-level modifiers suitable for live preview
class ModifierEngine {
public:
    bool apply(CADKernel& cad, const MeshData& base, const ModifierStack& stack,
               const std::function<bool(const std::string& asset_id, const std::string& scope, MeshData&)>& load_asset_mesh,
               MeshData& out, std::string* out_error = nullptr);
};

} // namespace mechatron
