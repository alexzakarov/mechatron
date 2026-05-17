#include "SchematicSymbol.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace mechatron {

const char* SchematicSymbolLibrary::user_dir() {
    return "assets/symbols_user";
}

const char* SchematicSymbolLibrary::default_dir() {
    return "assets/symbols_default";
}

bool SchematicSymbolLibrary::load_for_type(const std::string& component_type, SchematicSymbol& out) {
    namespace fs = std::filesystem;
    std::error_code ec;

    const fs::path user = fs::path(user_dir()) / (component_type + ".json");
    if (fs::exists(user, ec) && load_from_path(user.string(), out)) return true;

    const fs::path def = fs::path(default_dir()) / (component_type + ".json");
    if (fs::exists(def, ec) && load_from_path(def.string(), out)) return true;

    return false;
}

bool SchematicSymbolLibrary::load_from_path(const std::string& path, SchematicSymbol& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    nlohmann::json j;
    try {
        f >> j;
    } catch (...) {
        return false;
    }

    out = SchematicSymbol{};
    out.width = j.value("width", out.width);
    out.height = j.value("height", out.height);

    if (j.contains("container") && j["container"].is_object()) {
        out.container.enabled = j["container"].value("enabled", out.container.enabled);
        out.container.corner_radius = j["container"].value("corner_radius", out.container.corner_radius);
    }

    if (j.contains("body") && j["body"].is_array()) {
        for (const auto& p : j["body"]) {
            SchematicSymbolPrimitive prim;
            const std::string type = p.value("type", "line");
            if (type == "rect") prim.type = SchematicSymbolPrimitive::Type::Rect;
            else prim.type = SchematicSymbolPrimitive::Type::Line;
            prim.x1 = p.value("x1", 0.0f);
            prim.y1 = p.value("y1", 0.0f);
            prim.x2 = p.value("x2", 0.0f);
            prim.y2 = p.value("y2", 0.0f);
            prim.thickness = p.value("thickness", 2.0f);
            out.body.push_back(prim);
        }
    }

    if (j.contains("pins") && j["pins"].is_object()) {
        for (auto it = j["pins"].begin(); it != j["pins"].end(); ++it) {
            const std::string key = it.key();
            if (!it.value().is_array() || it.value().size() < 2) continue;
            out.pins[key] = {it.value()[0].get<float>(), it.value()[1].get<float>()};
        }
    }

    return true;
}

} // namespace mechatron
