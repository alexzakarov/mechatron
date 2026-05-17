#pragma once

#include "CADKernel.hpp"
#include <string>
#include <variant>
#include <vector>

namespace mechatron {

struct ModifierBoolean {
    enum class Op { Union, Subtract, Intersect };
    Op op = Op::Union;
    // Reference by asset id (looked up by ModelAssetLibrary in UI layer).
    std::string other_asset_id;
    std::string other_scope = "user"; // "user" | "default"
};

struct ModifierMirror {
    // Mirror across local axis planes.
    bool x = true;
    bool y = false;
    bool z = false;
};

struct ModifierSubdivision {
    int levels = 1; // very small to start
};

struct ModifierBevel {
    float amount = 0.05f; // in model units
};

struct ModifierWeld {
    float threshold = 0.0001f;
};

struct ModifierArray {
    int count = 2;
    Vec3 offset = {1.0f, 0.0f, 0.0f};
};

struct ModifierSolidify {
    float thickness = 0.05f;
    float offset = 1.0f; // -1 inner, 0 centered, 1 outer
};

using Modifier = std::variant<ModifierBoolean, ModifierMirror, ModifierSubdivision, ModifierBevel, ModifierWeld, ModifierArray, ModifierSolidify>;

struct ModifierStack {
    std::vector<Modifier> modifiers;
};

} // namespace mechatron
