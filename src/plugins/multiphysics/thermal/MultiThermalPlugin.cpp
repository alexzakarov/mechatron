#include "MultiThermalPlugin.hpp"
#include "core/Component.hpp"
#include <unordered_map>
#include <functional>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace mechatron {

// ============================================================================
// Material Library Implementation
// ============================================================================

std::unordered_map<std::string, MaterialProperties>& MaterialLibrary::database() {
    static std::unordered_map<std::string, MaterialProperties> db;
    return db;
}

void MaterialLibrary::initialize_defaults() {
    auto& db = database();

    // Metals
    db["steel"] = {"steel", 7850.0f, 500.0f, 50.0f};        // Carbon steel
    db["stainless_steel"] = {"stainless_steel", 8000.0f, 500.0f, 16.0f};
    db["aluminum"] = {"aluminum", 2700.0f, 900.0f, 237.0f};
    db["copper"] = {"copper", 8960.0f, 385.0f, 401.0f};
    db["brass"] = {"brass", 8500.0f, 380.0f, 120.0f};
    db["bronze"] = {"bronze", 8800.0f, 420.0f, 50.0f};
    db["iron"] = {"iron", 7870.0f, 450.0f, 80.0f};
    db["titanium"] = {"titanium", 4500.0f, 523.0f, 22.0f};
    db["lead"] = {"lead", 11340.0f, 130.0f, 35.0f};
    db["gold"] = {"gold", 19300.0f, 129.0f, 317.0f};
    db["silver"] = {"silver", 10500.0f, 235.0f, 429.0f};

    // Plastics/Polymers
    db["abs"] = {"abs", 1050.0f, 1386.0f, 0.25f};
    db["pvc"] = {"pvc", 1400.0f, 900.0f, 0.19f};
    db["polyethylene"] = {"polyethylene", 950.0f, 2300.0f, 0.33f};
    db["nylon"] = {"nylon", 1150.0f, 1700.0f, 0.25f};
    db["polycarbonate"] = {"polycarbonate", 1200.0f, 1170.0f, 0.20f};
    db["acrylic"] = {"acrylic", 1180.0f, 1470.0f, 0.19f};
    db["ptfe"] = {"ptfe", 2200.0f, 1050.0f, 0.25f};  // Teflon
    db["rubber"] = {"rubber", 1100.0f, 2000.0f, 0.13f};

    // Ceramics/Glass
    db["glass"] = {"glass", 2500.0f, 840.0f, 1.05f};
    db["ceramic"] = {"ceramic", 3000.0f, 800.0f, 3.0f};
    db["concrete"] = {"concrete", 2400.0f, 880.0f, 1.0f};
    db["brick"] = {"brick", 1920.0f, 840.0f, 0.72f};

    // Composites
    db["carbon_fiber"] = {"carbon_fiber", 1750.0f, 710.0f, 8.0f};
    db["fiberglass"] = {"fiberglass", 2000.0f, 835.0f, 0.04f};
    db["wood_oak"] = {"wood_oak", 750.0f, 2000.0f, 0.17f};
    db["wood_pine"] = {"wood_pine", 500.0f, 2300.0f, 0.12f};

    // Thermal interface materials
    db["thermal_grease"] = {"thermal_grease", 2500.0f, 1000.0f, 4.0f};
    db["thermal_pad"] = {"thermal_pad", 2000.0f, 1200.0f, 3.0f};
    db["heatsink_compound"] = {"heatsink_compound", 2200.0f, 1100.0f, 2.5f};

    // Air/Gases (for convection calculations)
    db["air"] = {"air", 1.225f, 1005.0f, 0.026f};
}

const MaterialProperties& MaterialLibrary::get(const std::string& name) {
    // Initialize on first use
    if (database().empty()) {
        initialize_defaults();
    }

    auto it = database().find(name);
    if (it != database().end()) {
        return it->second;
    }

    // Return steel as default if not found
    spdlog::warn("Material '{}' not found in library, using steel as default", name);
    return database()["steel"];
}

bool MaterialLibrary::has(const std::string& name) {
    if (database().empty()) {
        initialize_defaults();
    }
    return database().find(name) != database().end();
}

std::vector<std::string> MaterialLibrary::list_materials() {
    if (database().empty()) {
        initialize_defaults();
    }

    std::vector<std::string> materials;
    materials.reserve(database().size());
    for (const auto& [name, _] : database()) {
        materials.push_back(name);
    }
    std::sort(materials.begin(), materials.end());
    return materials;
}

void MaterialLibrary::add(const MaterialProperties& material) {
    if (database().empty()) {
        initialize_defaults();
    }
    database()[material.name] = material;
}

// ============================================================================
// Thermal Components
// ============================================================================

class HeatSourceComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "heat_source"; }
    std::string_view category() const override { return "thermal"; }

    HeatSourceComponent() {
        // Initialize from material library on construction
        set_material("steel");
    }

    void update(double dt) override {
        // Generate heat: Q = power (Watts)
        // Temperature rises: dT = Q * dt / (mass * specific_heat)
        float dT = m_power * static_cast<float>(dt) / (m_mass * m_specific_heat);
        m_temperature += dT;
    }

    float temperature() const { return m_temperature; }
    void set_power(float watts) { m_power = watts; }
    float power() const { return m_power; }

    // Set material from library
    void set_material(const std::string& material_name) {
        const auto& mat = MaterialLibrary::get(material_name);
        m_material_name = material_name;
        m_specific_heat = mat.specific_heat;
        // Mass is calculated from volume and density
        // Default volume of 0.0000127 m^3 (approx 50cm^3)
        float volume = 0.0000127f;
        m_mass = mat.density * volume;
    }

    void serialize(nlohmann::json& out) const override {
        out["temperature"] = m_temperature;
        out["power"] = m_power;
        out["mass"] = m_mass;
        out["material"] = m_material_name;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("power")) m_power = in["power"];
        if (in.contains("mass")) m_mass = in["mass"];
        if (in.contains("temperature")) m_temperature = in["temperature"];
        if (in.contains("material")) set_material(in["material"]);
    }

private:
    float m_power = 10.0f;          // Watts
    float m_mass = 0.1f;            // kg
    float m_specific_heat = 500.0f; // J/(kg*K)
    float m_temperature = 25.0f;    // Celsius
    std::string m_material_name = "steel";
};

class HeatSinkComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "heat_sink"; }
    std::string_view category() const override { return "thermal"; }

    HeatSinkComponent() {
        // Initialize from material library on construction
        set_material("aluminum");
    }

    void update(double dt) override {
        // Dissipate heat: Q = h * A * (T - T_ambient)
        float heat_loss = m_convection_coeff * m_area * (m_temperature - m_ambient_temp);
        float dT = -heat_loss * static_cast<float>(dt) / (m_mass * m_specific_heat);
        m_temperature += dT;
    }

    float temperature() const { return m_temperature; }
    void set_temperature(float temp) { m_temperature = temp; }

    // Set material from library
    void set_material(const std::string& material_name) {
        const auto& mat = MaterialLibrary::get(material_name);
        m_material_name = material_name;
        m_specific_heat = mat.specific_heat;
        // Mass is calculated from volume and density
        // Default volume of 0.000074 m^3 (approx 74cm^3)
        float volume = 0.000074f;
        m_mass = mat.density * volume;
    }

    // Ambient temperature configuration
    static float& default_ambient_temperature() {
        static float def = 25.0f;  // Celsius
        return def;
    }
    static void set_default_ambient_temperature(float temp_c) { default_ambient_temperature() = temp_c; }

    void serialize(nlohmann::json& out) const override {
        out["temperature"] = m_temperature;
        out["convection_coeff"] = m_convection_coeff;
        out["area"] = m_area;
        out["material"] = m_material_name;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("convection_coeff")) m_convection_coeff = in["convection_coeff"];
        if (in.contains("area")) m_area = in["area"];
        if (in.contains("temperature")) m_temperature = in["temperature"];
        if (in.contains("material")) set_material(in["material"]);
    }

private:
    float m_convection_coeff = 10.0f; // W/(m^2*K) natural convection
    float m_area = 0.01f;             // m^2
    float m_mass = 0.2f;              // kg
    float m_specific_heat = 900.0f;   // J/(kg*K)
    float m_ambient_temp = default_ambient_temperature();  // Celsius
    float m_temperature = default_ambient_temperature();   // Celsius
    std::string m_material_name = "aluminum";
};

class ThermalResistanceComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "thermal_resistance"; }
    std::string_view category() const override { return "thermal"; }

    void update(double dt) override {
        // Fourier's law: Q = k * A * (T_hot - T_cold) / L
        // Thermal resistance: R_th = L / (k * A)
        float heat_flow = (m_hot_temp - m_cold_temp) / m_thermal_resistance;
        m_heat_flow = heat_flow;
    }

    float heat_flow() const { return m_heat_flow; }
    void set_temperatures(float hot, float cold) { m_hot_temp = hot; m_cold_temp = cold; }

    void serialize(nlohmann::json& out) const override {
        out["thermal_resistance"] = m_thermal_resistance;
        out["heat_flow"] = m_heat_flow;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("thermal_resistance")) m_thermal_resistance = in["thermal_resistance"];
    }

private:
    float m_thermal_resistance = 1.0f; // K/W
    float m_hot_temp = 100.0f;
    float m_cold_temp = 25.0f;
    float m_heat_flow = 0.0f;
};

class ConvectionComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "convection"; }
    std::string_view category() const override { return "thermal"; }

    void update(double dt) override {
        // Newton's law of cooling: Q = h * A * (T_surface - T_fluid)
        m_heat_flow = m_h * m_area * (m_surface_temp - m_fluid_temp);
    }

    float heat_flow() const { return m_heat_flow; }

    void serialize(nlohmann::json& out) const override {
        out["h"] = m_h;
        out["area"] = m_area;
        out["heat_flow"] = m_heat_flow;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("h")) m_h = in["h"];
        if (in.contains("area")) m_area = in["area"];
    }

private:
    float m_h = 25.0f;           // W/(m^2*K) forced convection
    float m_area = 0.01f;        // m^2
    float m_surface_temp = 80.0f;
    float m_fluid_temp = 25.0f;
    float m_heat_flow = 0.0f;
};

class RadiationComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "radiation"; }
    std::string_view category() const override { return "thermal"; }

    void update(double dt) override {
        // Stefan-Boltzmann: Q = epsilon * sigma * A * (T_s^4 - T_env^4)
        float T_s = m_surface_temp + 273.15f;
        float T_e = m_ambient_temp + 273.15f;
        m_heat_flow = m_emissivity * m_sigma * m_area * (T_s*T_s*T_s*T_s - T_e*T_e*T_e*T_e);
    }

    float heat_flow() const { return m_heat_flow; }

    void serialize(nlohmann::json& out) const override {
        out["emissivity"] = m_emissivity;
        out["area"] = m_area;
        out["heat_flow"] = m_heat_flow;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("emissivity")) m_emissivity = in["emissivity"];
        if (in.contains("area")) m_area = in["area"];
    }

private:
    float m_emissivity = 0.9f;
    float m_sigma = 5.67e-8f;    // Stefan-Boltzmann constant
    float m_area = 0.01f;        // m^2
    float m_surface_temp = 100.0f;
    float m_ambient_temp = 25.0f;
    float m_heat_flow = 0.0f;
};

// ============================================================================
// Plugin Implementation
// ============================================================================

std::vector<ComponentDescriptor> MultiThermalPlugin::components() const {
    return {
        {"heat_source", "Heat Source", "thermal", "Generates thermal energy"},
        {"heat_sink", "Heat Sink", "thermal", "Dissipates thermal energy"},
        {"thermal_resistance", "Thermal Resistance", "thermal", "Thermal conduction path"},
        {"convection", "Convection", "thermal", "Convective heat transfer"},
        {"radiation", "Radiation", "thermal", "Radiative heat transfer"}
    };
}

std::unique_ptr<Component> MultiThermalPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"heat_source", []() { return make_unique<HeatSourceComponent>(); }},
        {"heat_sink", []() { return make_unique<HeatSinkComponent>(); }},
        {"thermal_resistance", []() { return make_unique<ThermalResistanceComponent>(); }},
        {"convection", []() { return make_unique<ConvectionComponent>(); }},
        {"radiation", []() { return make_unique<RadiationComponent>(); }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void MultiThermalPlugin::on_register(PluginHost& host) {
}

void MultiThermalPlugin::on_unregister() {
}

} // namespace mechatron
