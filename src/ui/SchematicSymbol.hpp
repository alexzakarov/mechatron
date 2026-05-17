#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace mechatron {

struct SchematicSymbolPrimitive {
    enum class Type { Line, Rect };
    Type type = Type::Line;
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    float thickness = 2.0f;
};

struct SchematicSymbol {
    // Logical size in canvas units. Render code can scale this to node size.
    float width = 80.0f;
    float height = 60.0f;

    // Optional UI container (the blue-ish component rectangle). If disabled, the node is "frameless".
    struct ContainerStyle {
        bool enabled = true;
        float corner_radius = 4.0f;
    } container;

    // Draw primitives inside [0..width]x[0..height]
    std::vector<SchematicSymbolPrimitive> body;

    // Pin anchor positions in the same coordinate system.
    std::unordered_map<std::string, std::pair<float, float>> pins;
};

// Loads schematic symbols from assets and user overrides.
// Search order: user override -> bundled defaults.
class SchematicSymbolLibrary {
public:
    // Base directories (relative to workspace root).
    static const char* user_dir();     // "assets/symbols_user"
    static const char* default_dir();  // "assets/symbols_default"

    // Load symbol for a component type (e.g. "dc_voltage", "resistor").
    // Returns true if found.
    bool load_for_type(const std::string& component_type, SchematicSymbol& out);

private:
    bool load_from_path(const std::string& path, SchematicSymbol& out);
};

} // namespace mechatron
