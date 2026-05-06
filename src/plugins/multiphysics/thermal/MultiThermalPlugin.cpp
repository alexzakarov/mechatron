#include "MultiThermalPlugin.hpp"
#include "core/Component.hpp"
#include <unordered_map>
#include <functional>
#include <cmath>
#include <algorithm>

namespace mechatron {

// ============================================================================
// Thermal Components
// ============================================================================

class HeatSourceComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "heat_source"; }
    std::string_view category() const override { return "thermal"; }

    void update(double dt) override {
        // Generate heat: Q = power (Watts)
        // Temperature rises: dT = Q * dt / (mass * specific_heat)
        float dT = m_power * static_cast<float>(dt) / (m_mass * m_specific_heat);
        m_temperature += dT;
    }

    float temperature() const { return m_temperature; }
    void set_power(float watts) { m_power = watts; }
    float power() const { return m_power; }

    void serialize(nlohmann::json& out) const override {
        out["temperature"] = m_temperature;
        out["power"] = m_power;
        out["mass"] = m_mass;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("power")) m_power = in["power"];
        if (in.contains("mass")) m_mass = in["mass"];
        if (in.contains("temperature")) m_temperature = in["temperature"];
    }

private:
    float m_power = 10.0f;          // Watts
    float m_mass = 0.1f;            // kg
    float m_specific_heat = 500.0f; // J/(kg*K) (steel-like)
    float m_temperature = 25.0f;    // Celsius
};

class HeatSinkComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_thermal"; }
    std::string_view component_type() const override { return "heat_sink"; }
    std::string_view category() const override { return "thermal"; }

    void update(double dt) override {
        // Dissipate heat: Q = h * A * (T - T_ambient)
        float heat_loss = m_convection_coeff * m_area * (m_temperature - m_ambient_temp);
        float dT = -heat_loss * static_cast<float>(dt) / (m_mass * m_specific_heat);
        m_temperature += dT;
    }

    float temperature() const { return m_temperature; }
    void set_temperature(float temp) { m_temperature = temp; }

    void serialize(nlohmann::json& out) const override {
        out["temperature"] = m_temperature;
        out["convection_coeff"] = m_convection_coeff;
        out["area"] = m_area;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("convection_coeff")) m_convection_coeff = in["convection_coeff"];
        if (in.contains("area")) m_area = in["area"];
        if (in.contains("temperature")) m_temperature = in["temperature"];
    }

private:
    float m_convection_coeff = 10.0f; // W/(m^2*K) natural convection
    float m_area = 0.01f;             // m^2
    float m_mass = 0.2f;              // kg (aluminum)
    float m_specific_heat = 900.0f;   // J/(kg*K) aluminum
    float m_ambient_temp = 25.0f;     // Celsius
    float m_temperature = 25.0f;      // Celsius
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
