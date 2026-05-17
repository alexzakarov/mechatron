#pragma once

#include "Component.hpp"
#include "ComponentFactory.hpp"
#include "Registry.hpp"
#include "Types.hpp"
#include "electronics/CircuitSimulator.hpp"
#include <memory>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <vector>
#include <functional>

namespace mechatron {

// Forward declarations for circuit components
class CircuitComponent;
class Resistor;
class Capacitor;
class Inductor;
class Diode;
class LED;
class ZenerDiode;
class BJTTransistor;
class MOSFETTransistor;
class BuckConverter;
class BoostConverter;
class MotorDriver;
class DCVoltageSource;
class Ground;

template<typename T> class CircuitComponentAdapter;

/**
 * Global System Configuration
 *
 * Central configuration hub for all component types in the mechatron system.
 * Unifies circuit simulation components and plugin architecture components.
 * Provides type-safe casting, factory creation, and metadata lookup.
 */
class SystemConfig {
public:
    /**
     * Extended component metadata for both circuit and plugin components
     */
    struct ComponentMetadata {
        // Basic identification
        std::string type_id;              // e.g., "resistor", "dc_motor"
        std::string category;             // e.g., "passive", "actuator"
        std::string display_name;        // e.g., "Resistor", "DC Motor"
        std::string description;         // Human-readable description

        // Circuit simulation specific
        std::string spice_type;          // SPICE type (R, C, D, NPN, NMOS, etc.)
        bool is_circuit_component = false;      // True if this is a CircuitComponent-based component
        bool is_adapter_component = false;       // True if this is an adapter wrapper

        // Factory functions
        std::function<std::unique_ptr<Component>()> plugin_factory;
        std::function<std::unique_ptr<CircuitComponent>()> circuit_factory;

        // Type indices for safe casting
        std::type_index plugin_type_index = std::type_index(typeid(void));
        std::type_index circuit_type_index = std::type_index(typeid(void));

        // Aliases and alternative names
        std::vector<std::string> aliases;

        // Component parameters metadata (for UI generation)
        struct ParameterInfo {
            std::string name;           // Parameter name
            std::string display_name;   // Display name for UI
            std::string type;           // "float", "int", "bool", "string"
            double default_value;       // Default value
            double min_value;           // Minimum value
            double max_value;           // Maximum value
            std::string units;          // Units (Ω, F, V, A, etc.)
            bool visible;               // Show in properties panel
        };
        std::vector<ParameterInfo> parameters;

        // Pin definitions (for UI connection points)
        struct PinDefinition {
            std::string name;           // Pin name
            std::string display_name;   // Display name
            PinDirection direction;     // Pin direction
            PinType type;               // Pin type
        };
        std::vector<PinDefinition> pins;
    };

    /**
     * Get singleton instance
     */
    static SystemConfig& instance();

    /**
     * Initialize all built-in component types
     */
    void initialize_builtin_types();

    /**
     * Register a component type
     */
    void register_component(const ComponentMetadata& metadata);

    /**
     * Get component metadata by type ID
     */
    const ComponentMetadata* get_metadata(std::string_view type_id) const;

    /**
     * Get all registered type IDs
     */
    std::vector<std::string> all_type_ids() const;

    /**
     * Get type IDs by category
     */
    std::vector<std::string> type_ids_by_category(std::string_view category) const;

    /**
     * Check if type is registered
     */
    bool is_registered(std::string_view type_id) const;

    /**
     * Create plugin component by type ID
     */
    std::unique_ptr<Component> create_plugin_component(std::string_view type_id) const;

    /**
     * Create circuit component by type ID
     */
    std::unique_ptr<CircuitComponent> create_circuit_component(std::string_view type_id) const;

    /**
     * Type-safe casting for circuit components
     * Usage: auto* r = SystemConfig::cast_as<Resistor>(component);
     */
    template<typename T>
    static T* cast_as(CircuitComponent* component) {
        if (!component) return nullptr;
        return dynamic_cast<T*>(component);
    }

    /**
     * Check if circuit component is of specific type
     * Usage: bool is_res = SystemConfig::is_type<Resistor>(component);
     */
    template<typename T>
    static bool is_type(const CircuitComponent* component) {
        return cast_as<T>(component) != nullptr;
    }

    /**
     * Type-safe casting for plugin components
     */
    template<typename T>
    static T* cast_as(Component* component) {
        if (!component) return nullptr;
        return dynamic_cast<T*>(component);
    }

    /**
     * Type-safe casting with type string check
     */
    template<typename T>
    static T* cast_as_safe(CircuitComponent* component, std::string_view expected_type) {
        if (!component || component->type() != expected_type) {
            return nullptr;
        }
        return dynamic_cast<T*>(component);
    }

    /**
     * Get metadata from circuit component instance
     */
    const ComponentMetadata* get_metadata(const CircuitComponent* component) const;

    /**
     * Get component category
     */
    std::string get_category(std::string_view type_id) const;

    /**
     * Get component display name
     */
    std::string get_display_name(std::string_view type_id) const;

private:
    SystemConfig() = default;
    std::unordered_map<std::string, ComponentMetadata> m_metadata;
    std::unordered_map<std::type_index, const ComponentMetadata*> m_circuit_type_to_metadata;
    std::unordered_map<std::type_index, const ComponentMetadata*> m_plugin_type_to_metadata;

    void register_circuit_component(const ComponentMetadata& metadata);
    void register_adapter_component(const ComponentMetadata& metadata);
};

// ============================================================
// Component Type String Constants
// Centralized to avoid typos and hardcoding throughout codebase
// ============================================================

namespace Type {
        // Passive components
        constexpr std::string_view RESISTOR = "resistor";
        constexpr std::string_view CAPACITOR = "capacitor";
        constexpr std::string_view INDUCTOR = "inductor";
        constexpr std::string_view POTENTIOMETER = "potentiometer";
        constexpr std::string_view TRANSFORMER = "transformer";

        // Semiconductor components
        constexpr std::string_view DIODE = "diode";
        constexpr std::string_view ZENER_DIODE = "zener_diode";
        constexpr std::string_view LED = "led";
        constexpr std::string_view BJT_NPN = "bjt_npn";
        constexpr std::string_view BJT_PNP = "bjt_pnp";
        constexpr std::string_view MOSFET_N = "mosfet_n";
        constexpr std::string_view MOSFET_P = "mosfet_p";
        constexpr std::string_view JFET_N = "jfet_n";
        constexpr std::string_view JFET_P = "jfet_p";
        constexpr std::string_view OPAMP = "opamp";

        // Power components
        constexpr std::string_view DC_VOLTAGE = "dc_voltage";
        constexpr std::string_view AC_VOLTAGE = "ac_voltage";
        constexpr std::string_view PULSE_VOLTAGE = "pulse_voltage";
        constexpr std::string_view CURRENT_SOURCE = "current_source";
        constexpr std::string_view BUCK_CONVERTER = "buck_converter";
        constexpr std::string_view BOOST_CONVERTER = "boost_converter";
        constexpr std::string_view MOTOR_DRIVER = "motor_driver";
        constexpr std::string_view HBRIDGE = "hbridge";
        constexpr std::string_view GROUND = "ground";

        // Actuators (for physics system)
        constexpr std::string_view DC_MOTOR = "dc_motor";
        constexpr std::string_view SERVO = "servo";
        constexpr std::string_view STEPPER = "stepper";
        constexpr std::string_view LINEAR_ACTUATOR = "linear_actuator";

        // Sensors
        constexpr std::string_view POTENTIOMETER_SENSOR = "potentiometer_sensor";
        constexpr std::string_view ENCODER = "encoder";
        constexpr std::string_view LIMIT_SWITCH = "limit_switch";
        constexpr std::string_view TEMPERATURE_SENSOR = "temperature_sensor";
        constexpr std::string_view CURRENT_SENSOR = "current_sensor";
        constexpr std::string_view VOLTAGE_SENSOR = "voltage_sensor";

        // Mechanical
        constexpr std::string_view GEARBOX = "gearbox";
        constexpr std::string_view JOINT = "joint";
        constexpr std::string_view LINK = "link";

        // Categories
        constexpr std::string_view CATEGORY_PASSIVE = "passive";
        constexpr std::string_view CATEGORY_SEMICONDUCTOR = "semiconductor";
        constexpr std::string_view CATEGORY_POWER = "power";
        constexpr std::string_view CATEGORY_OPTOELECTRONIC = "optoelectronic";
        constexpr std::string_view CATEGORY_ACTUATOR = "actuator";
        constexpr std::string_view CATEGORY_SENSOR = "sensor";
        constexpr std::string_view CATEGORY_MECHANICAL = "mechanical";
        constexpr std::string_view CATEGORY_LOGIC = "logic";
        constexpr std::string_view CATEGORY_COMMUNICATION = "communication";
        constexpr std::string_view CATEGORY_UI = "ui";
    }

/**
 * Convenience macros for component type checking
 */
#define IS_RESISTOR(comp) SystemConfig::is_type<Resistor>(comp)
#define IS_CAPACITOR(comp) SystemConfig::is_type<Capacitor>(comp)
#define IS_INDUCTOR(comp) SystemConfig::is_type<Inductor>(comp)
#define IS_DIODE(comp) SystemConfig::is_type<Diode>(comp)
#define IS_LED(comp) SystemConfig::is_type<LED>(comp)
#define IS_BJT(comp) SystemConfig::is_type<BJTTransistor>(comp)
#define IS_MOSFET(comp) SystemConfig::is_type<MOSFETTransistor>(comp)

/**
 * Cast macros for common component types
 */
#define CAST_RESISTOR(comp) SystemConfig::cast_as<Resistor>(comp)
#define CAST_CAPACITOR(comp) SystemConfig::cast_as<Capacitor>(comp)
#define CAST_INDUCTOR(comp) SystemConfig::cast_as<Inductor>(comp)
#define CAST_DIODE(comp) SystemConfig::cast_as<Diode>(comp)
#define CAST_LED(comp) SystemConfig::cast_as<LED>(comp)
#define CAST_BJT(comp) SystemConfig::cast_as<BJTTransistor>(comp)
#define CAST_MOSFET(comp) SystemConfig::cast_as<MOSFETTransistor>(comp)

/**
 * Component dispatcher helper
 * Eliminates if-else chains when processing components by type
 */
template<typename ResultType = void>
class ComponentDispatcher {
public:
    using Handler = std::function<ResultType(CircuitComponent*)>;

    ComponentDispatcher& handler(std::string_view type_id, Handler fn) {
        m_handlers[std::string(type_id)] = std::move(fn);
        return *this;
    }

    ComponentDispatcher& handler(std::initializer_list<std::string_view> type_ids, Handler fn) {
        for (auto type_id : type_ids) {
            m_handlers[std::string(type_id)] = fn;
        }
        return *this;
    }

    ResultType dispatch(CircuitComponent* component) const {
        if (!component) {
            if (m_default_handler) {
                return m_default_handler(component);
            }
            return ResultType();
        }

        auto it = m_handlers.find(std::string(component->type()));
        if (it != m_handlers.end()) {
            return it->second(component);
        }

        if (m_default_handler) {
            return m_default_handler(component);
        }

        return ResultType();
    }

    ComponentDispatcher& default_handler(Handler fn) {
        m_default_handler = std::move(fn);
        return *this;
    }

private:
    std::unordered_map<std::string, Handler> m_handlers;
    Handler m_default_handler;
};

} // namespace mechatron
