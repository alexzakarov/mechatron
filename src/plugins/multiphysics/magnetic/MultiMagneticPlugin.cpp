#include "MultiMagneticPlugin.hpp"
#include "core/Component.hpp"
#include <unordered_map>
#include <functional>
#include <cmath>

namespace mechatron {

// ============================================================================
// Magnetic Components
// ============================================================================

class SolenoidFieldComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_magnetic"; }
    std::string_view component_type() const override { return "solenoid_field"; }
    std::string_view category() const override { return "magnetic"; }

    void update(double dt) override {
        // B = mu0 * n * I (solenoid field)
        // n = turns / length
        float n = m_turns / m_length;
        m_field_strength = m_mu0 * n * m_current;
    }

    float field_strength() const { return m_field_strength; }
    void set_current(float current) { m_current = current; }

    void serialize(nlohmann::json& out) const override {
        out["field_strength"] = m_field_strength;
        out["turns"] = m_turns;
        out["current"] = m_current;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("turns")) m_turns = in["turns"];
        if (in.contains("current")) m_current = in["current"];
    }

private:
    float m_mu0 = 1.2566e-6f;    // Vacuum permeability (H/m)
    float m_turns = 100.0f;
    float m_length = 0.1f;       // meters
    float m_current = 0.0f;      // Amps
    float m_field_strength = 0.0f;
};

class PermanentMagnetComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_magnetic"; }
    std::string_view component_type() const override { return "permanent_magnet"; }
    std::string_view category() const override { return "magnetic"; }

    void update(double dt) override {
        // B_rem = remanence field strength
        // Field at distance r: B = (B_rem * V) / (4*pi*r^3) (dipole approx)
        // Simplified: just store the remanence
    }

    float field_at_distance(float r) const {
        // Dipole field falls off as 1/r^3
        if (r < 0.001f) r = 0.001f;
        return m_remanence * m_volume / (4.0f * 3.14159f * r * r * r);
    }

    float remanence() const { return m_remanence; }

    void serialize(nlohmann::json& out) const override {
        out["remanence"] = m_remanence;
        out["volume"] = m_volume;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("remanence")) m_remanence = in["remanence"];
        if (in.contains("volume")) m_volume = in["volume"];
    }

private:
    float m_remanence = 1.2f;    // Tesla (NdFeB-like)
    float m_volume = 1e-6f;      // m^3
};

class MagneticCircuitComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_magnetic"; }
    std::string_view component_type() const override { return "magnetic_circuit"; }
    std::string_view category() const override { return "magnetic"; }

    void update(double dt) override {
        // Magnetic Ohm's law: Phi = MMF / R_reluctance
        // MMF = N * I (magnetomotive force)
        // R = l / (mu * A)
        m_flux = m_mmf / m_reluctance;
        m_flux_density = m_flux / m_core_area;
    }

    float flux() const { return m_flux; }
    float flux_density() const { return m_flux_density; }
    void set_mmf(float mmf) { m_mmf = mmf; }

    void serialize(nlohmann::json& out) const override {
        out["flux"] = m_flux;
        out["flux_density"] = m_flux_density;
        out["reluctance"] = m_reluctance;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("reluctance")) m_reluctance = in["reluctance"];
        if (in.contains("mmf")) m_mmf = in["mmf"];
    }

private:
    float m_mmf = 100.0f;            // Ampere-turns
    float m_reluctance = 1e6f;       // A*t/Wb
    float m_core_area = 1e-4f;       // m^2
    float m_flux = 0.0f;
    float m_flux_density = 0.0f;
};

class EddyCurrentComponent : public Component {
public:
    std::string_view plugin_type() const override { return "multi_magnetic"; }
    std::string_view component_type() const override { return "eddy_current"; }
    std::string_view category() const override { return "magnetic"; }

    void update(double dt) override {
        // Eddy current losses: P = k * B^2 * f^2 * d^2 * V
        // k depends on material, d = thickness, V = volume
        float f_dt = static_cast<float>(dt);
        m_power_loss = m_loss_coefficient * m_b_field * m_b_field *
                       m_frequency * m_frequency * m_thickness * m_thickness * m_volume;
        // Energy loss this step
        m_energy_loss += m_power_loss * f_dt;
    }

    float power_loss() const { return m_power_loss; }
    void set_field(float b, float freq) { m_b_field = b; m_frequency = freq; }

    void serialize(nlohmann::json& out) const override {
        out["power_loss"] = m_power_loss;
        out["energy_loss"] = m_energy_loss;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("loss_coefficient")) m_loss_coefficient = in["loss_coefficient"];
        if (in.contains("thickness")) m_thickness = in["thickness"];
    }

private:
    float m_loss_coefficient = 1.0f; // Material constant
    float m_b_field = 1.0f;          // Tesla
    float m_frequency = 50.0f;       // Hz
    float m_thickness = 0.001f;      // meters (lamination)
    float m_volume = 1e-4f;          // m^3
    float m_power_loss = 0.0f;
    float m_energy_loss = 0.0f;
};

// ============================================================================
// Plugin Implementation
// ============================================================================

std::vector<ComponentDescriptor> MultiMagneticPlugin::components() const {
    return {
        {"solenoid_field", "Solenoid Field", "magnetic", "Electromagnetic field from solenoid"},
        {"permanent_magnet", "Permanent Magnet", "magnetic", "Static magnetic field source"},
        {"magnetic_circuit", "Magnetic Circuit", "magnetic", "Magnetic flux path"},
        {"eddy_current", "Eddy Current", "magnetic", "Induced current in conductors"}
    };
}

std::unique_ptr<Component> MultiMagneticPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"solenoid_field", []() { return make_unique<SolenoidFieldComponent>(); }},
        {"permanent_magnet", []() { return make_unique<PermanentMagnetComponent>(); }},
        {"magnetic_circuit", []() { return make_unique<MagneticCircuitComponent>(); }},
        {"eddy_current", []() { return make_unique<EddyCurrentComponent>(); }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void MultiMagneticPlugin::on_register(PluginHost& host) {
}

void MultiMagneticPlugin::on_unregister() {
}

} // namespace mechatron
