#include "SystemConfig.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include "core/Types.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cassert>

namespace mechatron {

SystemConfig& SystemConfig::instance() {
    static SystemConfig config;
    static bool initialized = false;
    if (!initialized) {
        config.initialize_builtin_types();
        initialized = true;
        spdlog::info("[SystemConfig] Initialized system with {} component types",
                   config.m_metadata.size());
    }
    return config;
}

void SystemConfig::register_component(const ComponentMetadata& metadata) {
    // Check for duplicate type IDs
    if (m_metadata.find(metadata.type_id) != m_metadata.end()) {
        spdlog::warn("[SystemConfig] Duplicate type ID: {}", metadata.type_id);
        return;
    }

    m_metadata[metadata.type_id] = metadata;

    // Index by type_index for fast casting (check if not the default void type)
    static const std::type_index void_type = typeid(void);
    if (metadata.circuit_type_index != void_type) {
        m_circuit_type_to_metadata[metadata.circuit_type_index] = &m_metadata[metadata.type_id];
    }
    if (metadata.plugin_type_index != void_type) {
        m_plugin_type_to_metadata[metadata.plugin_type_index] = &m_metadata[metadata.type_id];
    }

    // Register aliases
    for (const auto& alias : metadata.aliases) {
        if (m_metadata.find(alias) == m_metadata.end()) {
            m_metadata[alias] = metadata;  // Copy to alias key
        }
    }

    // Register with ComponentFactory if plugin factory exists
    if (metadata.plugin_factory) {
        ComponentInfo info;
        info.type_name = metadata.type_id;
        info.category = metadata.category;
        info.display_name = metadata.display_name;
        info.description = metadata.description;
        info.factory = metadata.plugin_factory;
        ComponentFactory::instance().register_component(info);
    }

    spdlog::debug("[SystemConfig] Registered: {} ({})",
                metadata.type_id, metadata.display_name);
}

void SystemConfig::register_circuit_component(const ComponentMetadata& metadata) {
    m_metadata[metadata.type_id] = metadata;

    static const std::type_index void_type = typeid(void);
    if (metadata.circuit_type_index != void_type) {
        m_circuit_type_to_metadata[metadata.circuit_type_index] = &m_metadata[metadata.type_id];
    }

    // Register aliases
    for (const auto& alias : metadata.aliases) {
        if (m_metadata.find(alias) == m_metadata.end()) {
            m_metadata[alias] = metadata;
        }
    }
}

void SystemConfig::register_adapter_component(const ComponentMetadata& metadata) {
    m_metadata[metadata.type_id] = metadata;

    static const std::type_index void_type = typeid(void);
    if (metadata.plugin_type_index != void_type) {
        m_plugin_type_to_metadata[metadata.plugin_type_index] = &m_metadata[metadata.type_id];
    }

    // Register with ComponentFactory
    if (metadata.plugin_factory) {
        ComponentInfo info;
        info.type_name = metadata.type_id;
        info.category = metadata.category;
        info.display_name = metadata.display_name;
        info.description = metadata.description;
        info.factory = metadata.plugin_factory;
        ComponentFactory::instance().register_component(info);
    }

    spdlog::debug("[SystemConfig] Registered adapter: {} ({})",
                metadata.type_id, metadata.display_name);
}

const SystemConfig::ComponentMetadata* SystemConfig::get_metadata(std::string_view type_id) const {
    auto it = m_metadata.find(std::string(type_id));
    if (it != m_metadata.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<std::string> SystemConfig::all_type_ids() const {
    std::vector<std::string> result;
    result.reserve(m_metadata.size());

    for (const auto& [type_id, _] : m_metadata) {
        // Only return primary type IDs (not aliases)
        if (type_id == m_metadata.at(type_id).type_id) {
            result.push_back(type_id);
        }
    }

    std::sort(result.begin(), result.end());
    return result;
}

std::vector<std::string> SystemConfig::type_ids_by_category(std::string_view category) const {
    std::vector<std::string> result;

    for (const auto& [type_id, info] : m_metadata) {
        if (info.type_id == type_id && info.category == category) {
            result.push_back(type_id);
        }
    }

    std::sort(result.begin(), result.end());
    return result;
}

bool SystemConfig::is_registered(std::string_view type_id) const {
    return m_metadata.find(std::string(type_id)) != m_metadata.end();
}

std::unique_ptr<Component> SystemConfig::create_plugin_component(std::string_view type_id) const {
    auto info = get_metadata(type_id);
    if (info && info->plugin_factory) {
        return info->plugin_factory();
    }
    return nullptr;
}

std::unique_ptr<CircuitComponent> SystemConfig::create_circuit_component(std::string_view type_id) const {
    auto info = get_metadata(type_id);
    if (info && info->circuit_factory) {
        return info->circuit_factory();
    }
    return nullptr;
}

const SystemConfig::ComponentMetadata* SystemConfig::get_metadata(const CircuitComponent* component) const {
    if (!component) return nullptr;

    auto type_index = std::type_index(typeid(*component));
    auto it = m_circuit_type_to_metadata.find(type_index);
    if (it != m_circuit_type_to_metadata.end()) {
        return it->second;
    }

    return nullptr;
}

std::string SystemConfig::get_category(std::string_view type_id) const {
    auto info = get_metadata(type_id);
    return info ? info->category : "";
}

std::string SystemConfig::get_display_name(std::string_view type_id) const {
    auto info = get_metadata(type_id);
    return info ? std::string(info->display_name) : std::string(type_id);
}

void SystemConfig::initialize_builtin_types() {
    using namespace Type;

    // ============================================================
    // Passive Components
    // ============================================================

    // Resistor
    {
        ComponentMetadata info;
        info.type_id = std::string(RESISTOR);
        info.category = std::string(CATEGORY_PASSIVE);
        info.display_name = "Resistor";
        info.description = "Two-terminal passive component that opposes current flow";
        info.spice_type = "R";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<Resistor>(); };
        info.circuit_type_index = typeid(Resistor);
        info.plugin_type_index = std::type_index(typeid(void));  // Not directly a plugin component
        info.aliases = {"R"};
        info.pins = {
            {"1", "Pin 1", PinDirection::Bidirectional, PinType::Analog},
            {"2", "Pin 2", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"resistance", "Resistance", "float", 1000.0, 0.0, 1e9, "Ω", true}
        };
        register_component(info);
    }

    // Capacitor
    {
        ComponentMetadata info;
        info.type_id = std::string(CAPACITOR);
        info.category = std::string(CATEGORY_PASSIVE);
        info.display_name = "Capacitor";
        info.description = "Two-terminal passive component that stores energy in electric field";
        info.spice_type = "C";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<Capacitor>(); };
        info.circuit_type_index = typeid(Capacitor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"C"};
        info.pins = {
            {"1", "Pin 1", PinDirection::Bidirectional, PinType::Analog},
            {"2", "Pin 2", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"capacitance", "Capacitance", "float", 1e-6, 1e-12, 1.0, "F", true}
        };
        register_component(info);
    }

    // Inductor
    {
        ComponentMetadata info;
        info.type_id = std::string(INDUCTOR);
        info.category = std::string(CATEGORY_PASSIVE);
        info.display_name = "Inductor";
        info.description = "Two-terminal passive component that stores energy in magnetic field";
        info.spice_type = "L";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<Inductor>(); };
        info.circuit_type_index = typeid(Inductor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"L"};
        info.pins = {
            {"1", "Pin 1", PinDirection::Bidirectional, PinType::Analog},
            {"2", "Pin 2", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"inductance", "Inductance", "float", 1e-3, 1e-9, 10.0, "H", true}
        };
        register_component(info);
    }

    // ============================================================
    // Semiconductor Components
    // ============================================================

    // Diode
    {
        ComponentMetadata info;
        info.type_id = std::string(DIODE);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "Diode";
        info.description = "Two-terminal semiconductor that allows current flow in one direction";
        info.spice_type = "D";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<Diode>(); };
        info.circuit_type_index = typeid(Diode);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"D"};
        info.pins = {
            {"anode", "Anode", PinDirection::Bidirectional, PinType::Analog},
            {"cathode", "Cathode", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"forward_voltage", "Forward Voltage", "float", 0.7, 0.1, 5.0, "V", true},
            {"on_resistance", "On Resistance", "float", 0.1, 0.001, 100.0, "Ω", true}
        };
        register_component(info);
    }

    // LED
    {
        ComponentMetadata info;
        info.type_id = std::string(Type::LED);
        info.category = std::string(CATEGORY_OPTOELECTRONIC);
        info.display_name = "LED";
        info.description = "Light-emitting diode that emits light when current flows through it";
        info.spice_type = "D";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<mechatron::LED>(); };
        info.circuit_type_index = typeid(mechatron::LED);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"DLED"};
        info.pins = {
            {"anode", "Anode", PinDirection::Bidirectional, PinType::Analog},
            {"cathode", "Cathode", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"forward_voltage", "Forward Voltage", "float", 2.0, 0.5, 5.0, "V", true}
        };
        register_component(info);
    }

    // Zener Diode
    {
        ComponentMetadata info;
        info.type_id = std::string(ZENER_DIODE);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "Zener Diode";
        info.description = "Diode designed to allow current in reverse direction when breakdown voltage is reached";
        info.spice_type = "D";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<ZenerDiode>(); };
        info.circuit_type_index = typeid(ZenerDiode);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"DZENER"};
        info.pins = {
            {"anode", "Anode", PinDirection::Bidirectional, PinType::Analog},
            {"cathode", "Cathode", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"zener_voltage", "Zener Voltage", "float", 5.1, 1.0, 100.0, "V", true},
            {"forward_voltage", "Forward Voltage", "float", 0.7, 0.1, 5.0, "V", true}
        };
        register_component(info);
    }

    // BJT NPN
    {
        ComponentMetadata info;
        info.type_id = std::string(BJT_NPN);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "NPN Transistor";
        info.description = "Bipolar junction transistor (NPN type)";
        info.spice_type = "NPN";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<BJTTransistor>(BJTTransistor::NPN); };
        info.circuit_type_index = typeid(BJTTransistor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"Q", "NPN", "BJT"};
        info.pins = {
            {"base", "Base", PinDirection::Input, PinType::Analog},
            {"collector", "Collector", PinDirection::Bidirectional, PinType::Analog},
            {"emitter", "Emitter", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"beta", "Beta (hFE)", "float", 100.0, 10.0, 1000.0, "", true}
        };
        register_component(info);
    }

    // BJT PNP
    {
        ComponentMetadata info;
        info.type_id = std::string(BJT_PNP);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "PNP Transistor";
        info.description = "Bipolar junction transistor (PNP type)";
        info.spice_type = "PNP";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<BJTTransistor>(BJTTransistor::PNP); };
        info.circuit_type_index = typeid(BJTTransistor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"Q", "PNP", "BJT"};
        info.pins = {
            {"base", "Base", PinDirection::Input, PinType::Analog},
            {"collector", "Collector", PinDirection::Bidirectional, PinType::Analog},
            {"emitter", "Emitter", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"beta", "Beta (hFE)", "float", 100.0, 10.0, 1000.0, "", true}
        };
        register_component(info);
    }

    // MOSFET N-Channel
    {
        ComponentMetadata info;
        info.type_id = std::string(MOSFET_N);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "N-Channel MOSFET";
        info.description = "Metal-oxide-semiconductor field-effect transistor (N-channel)";
        info.spice_type = "NMOS";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<MOSFETTransistor>(MOSFETTransistor::NChannel); };
        info.circuit_type_index = typeid(MOSFETTransistor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"M", "NMOS", "FET", "MOSFET"};
        info.pins = {
            {"gate", "Gate", PinDirection::Input, PinType::Analog},
            {"drain", "Drain", PinDirection::Bidirectional, PinType::Analog},
            {"source", "Source", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"threshold_voltage", "Threshold Voltage", "float", 2.0, 0.5, 5.0, "V", true},
            {"kp", "Transconductance", "float", 1.0, 0.01, 1000.0, "A/V²", true}
        };
        register_component(info);
    }

    // MOSFET P-Channel
    {
        ComponentMetadata info;
        info.type_id = std::string(MOSFET_P);
        info.category = std::string(CATEGORY_SEMICONDUCTOR);
        info.display_name = "P-Channel MOSFET";
        info.description = "Metal-oxide-semiconductor field-effect transistor (P-channel)";
        info.spice_type = "PMOS";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<MOSFETTransistor>(MOSFETTransistor::PChannel); };
        info.circuit_type_index = typeid(MOSFETTransistor);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"M", "PMOS", "MOSFET"};
        info.pins = {
            {"gate", "Gate", PinDirection::Input, PinType::Analog},
            {"drain", "Drain", PinDirection::Bidirectional, PinType::Analog},
            {"source", "Source", PinDirection::Bidirectional, PinType::Analog}
        };
        info.parameters = {
            {"threshold_voltage", "Threshold Voltage", "float", -2.0, -5.0, -0.5, "V", true},
            {"kp", "Transconductance", "float", 1.0, 0.01, 1000.0, "A/V²", true}
        };
        register_component(info);
    }

    // ============================================================
    // Power Components
    // ============================================================

    // DC Voltage Source
    {
        ComponentMetadata info;
        info.type_id = std::string(DC_VOLTAGE);
        info.category = std::string(CATEGORY_POWER);
        info.display_name = "DC Voltage Source";
        info.description = "Direct current voltage source";
        info.spice_type = "V";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<DCVoltageSource>(); };
        info.circuit_type_index = typeid(DCVoltageSource);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"V", "VDC", "SOURCE"};
        info.pins = {
            {"VOUT", "VOUT", PinDirection::Output, PinType::Power},
            {"GND", "GND", PinDirection::Output, PinType::Ground}
        };
        info.parameters = {
            {"voltage", "Voltage", "float", 5.0, 0.0, 1000.0, "V", true}
        };
        register_component(info);
    }

    // Buck Converter
    {
        ComponentMetadata info;
        info.type_id = std::string(BUCK_CONVERTER);
        info.category = std::string(CATEGORY_POWER);
        info.display_name = "Buck Converter";
        info.description = "Step-down DC-DC converter";
        info.spice_type = "SUBCKT";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<BuckConverter>(); };
        info.circuit_type_index = typeid(BuckConverter);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"BUCK", "STEPDOWN"};
        info.pins = {
            {"VIN", "Input Voltage", PinDirection::Input, PinType::Power},
            {"GND", "Ground", PinDirection::Input, PinType::Ground},
            {"VOUT", "Output Voltage", PinDirection::Output, PinType::Power}
        };
        info.parameters = {
            {"input_voltage", "Input Voltage", "float", 12.0, 0.0, 100.0, "V", true},
            {"duty_cycle", "Duty Cycle", "float", 0.5, 0.0, 1.0, "", true}
        };
        register_component(info);
    }

    // Boost Converter
    {
        ComponentMetadata info;
        info.type_id = std::string(BOOST_CONVERTER);
        info.category = std::string(CATEGORY_POWER);
        info.display_name = "Boost Converter";
        info.description = "Step-up DC-DC converter";
        info.spice_type = "SUBCKT";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<BoostConverter>(); };
        info.circuit_type_index = typeid(BoostConverter);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"BOOST", "STEPUP"};
        info.pins = {
            {"VIN", "Input Voltage", PinDirection::Input, PinType::Power},
            {"GND", "Ground", PinDirection::Input, PinType::Ground},
            {"VOUT", "Output Voltage", PinDirection::Output, PinType::Power}
        };
        info.parameters = {
            {"input_voltage", "Input Voltage", "float", 5.0, 0.0, 100.0, "V", true},
            {"duty_cycle", "Duty Cycle", "float", 0.5, 0.0, 0.95, "", true}
        };
        register_component(info);
    }

    // Motor Driver
    {
        ComponentMetadata info;
        info.type_id = std::string(MOTOR_DRIVER);
        info.category = std::string(CATEGORY_POWER);
        info.display_name = "Motor Driver";
        info.description = "H-bridge motor driver for bidirectional motor control";
        info.spice_type = "SUBCKT";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<MotorDriver>(); };
        info.circuit_type_index = typeid(MotorDriver);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"DRIVER", "HBRIDGE", "HBDRIVER"};
        info.pins = {
            {"PWM", "PWM Input", PinDirection::Input, PinType::Analog},
            {"DIR", "Direction", PinDirection::Input, PinType::Digital},
            {"EN", "Enable", PinDirection::Input, PinType::Digital},
            {"OUT+", "Output Positive", PinDirection::Output, PinType::Power},
            {"OUT-", "Output Negative", PinDirection::Output, PinType::Power},
            {"VCC", "Power Supply", PinDirection::Input, PinType::Power},
            {"GND", "Ground", PinDirection::Input, PinType::Ground}
        };
        info.parameters = {
            {"supply_voltage", "Supply Voltage", "float", 12.0, 0.0, 100.0, "V", true}
        };
        register_component(info);
    }

    // Ground
    {
        ComponentMetadata info;
        info.type_id = std::string(GROUND);
        info.category = std::string(CATEGORY_POWER);
        info.display_name = "Ground";
        info.description = "Circuit ground reference (0V)";
        info.spice_type = "GND";
        info.is_circuit_component = true;
        info.is_adapter_component = false;
        info.circuit_factory = []() { return std::make_unique<Ground>(); };
        info.circuit_type_index = typeid(Ground);
        info.plugin_type_index = std::type_index(typeid(void));
        info.aliases = {"GND", "0"};
        info.pins = {
            {"GND", "Ground", PinDirection::Output, PinType::Ground}
        };
        info.parameters = {};
        register_component(info);
    }
}

} // namespace mechatron
