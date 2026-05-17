#pragma once

#include "core/PluginHost.hpp"
#include "core/Types.hpp"
#include <memory>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <string>

namespace mechatron {

/**
 * @brief Material thermal properties
 *
 * Physical properties for thermal simulation:
 * - density: kg/m^3
 * - specific_heat: J/(kg*K)
 * - thermal_conductivity: W/(m*K)
 */
struct MaterialProperties {
    std::string name;
    float density;           // kg/m^3
    float specific_heat;     // J/(kg*K)
    float thermal_conductivity; // W/(m*K)
};

/**
 * @brief Material property library
 *
 * Database of common engineering materials for thermal simulation.
 */
class MaterialLibrary {
public:
    // Get material properties by name
    static const MaterialProperties& get(const std::string& name);

    // Check if material exists
    static bool has(const std::string& name);

    // Get all available material names
    static std::vector<std::string> list_materials();

    // Add or override a material
    static void add(const MaterialProperties& material);

private:
    static std::unordered_map<std::string, MaterialProperties>& database();
    static void initialize_defaults();
};

/**
 * @brief Thermal Physics Plugin
 *
 * Provides thermal simulation components: heat sources, heat sinks, convection.
 */
class MultiThermalPlugin : public IMechatronPlugin {
public:
    std::string_view name() const override { return "multi_thermal"; }
    std::string_view version() const override { return "1.0.0"; }

    std::vector<ComponentDescriptor> components() const override;
    std::unique_ptr<Component> create(std::string_view type) override;

    void on_register(PluginHost& host) override;
    void on_unregister() override;
};

} // namespace mechatron
