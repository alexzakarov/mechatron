#pragma once

#include "Types.hpp"
#include "Component.hpp"
#include <unordered_map>
#include <string>
#include <functional>

namespace mechatron {

// Forward declarations
class CircuitSimulator;
class Registry;
class CircuitPin;
class Actuator;

// Pin mapping types
enum class PinMappingType {
    // Circuit → Actuator (Voltage controls actuator)
    VoltageToActuatorInput,      // Circuit voltage (0-5V) → Actuator input (0-1)

    // Sensor → Circuit (Sensor value controls circuit pin)
    SensorToDigitalPin,          // Sensor digital state → Pin HIGH/LOW
    SensorToAnalogPin,           // Sensor analog value → Pin voltage (0-5V)

    // Circuit → Physics Direct
    VoltageToForce,              // Circuit voltage → Force magnitude (Newtons)
    VoltageToTorque,             // Circuit voltage → Torque magnitude (Nm)

    // Digital control
    DigitalToEnable,             // Pin HIGH/LOW → Enable/disable actuator
    PWMToSpeed                   // PWM duty cycle → Motor speed
};

// Pin mapping - connects circuit pin to component parameter
struct PinMapping {
    std::string circuit_pin_id;      // Format: "component_id.pin_name"
    std::string target_component_id;
    PinMappingType type;

    // Scaling factors
    float voltage_min = 0.0f;        // Minimum input voltage
    float voltage_max = 5.0f;        // Maximum input voltage
    float output_min = 0.0f;         // Minimum output value
    float output_max = 1.0f;         // Maximum output value
};

// Bridge between circuit simulation and physics/actuators
class CircuitPhysicsBridge {
public:
    CircuitPhysicsBridge();
    ~CircuitPhysicsBridge() = default;

    // Add a pin mapping
    void add_mapping(const PinMapping& mapping);

    // Remove a mapping
    void remove_mapping(const std::string& circuit_pin_id);

    // Update all mappings - call this every simulation step
    void update(double dt);

    // Set/get circuit simulator reference
    void set_circuit_simulator(class CircuitSimulator* sim) { m_circuit_sim = sim; }
    CircuitSimulator* circuit_simulator() { return m_circuit_sim; }

    // Set/get component registry for finding actuators/sensors
    void set_registry(class Registry* registry) { m_registry = registry; }
    Registry* registry() { return m_registry; }

    // Enable/disable the bridge
    void set_enabled(bool enabled) { m_enabled = enabled; }
    bool is_enabled() const { return m_enabled; }

private:
    // Process a single mapping
    void process_mapping(const PinMapping& mapping);

    // Helper functions
    float normalize_voltage(float voltage, const PinMapping& mapping) const;
    void apply_to_actuator(const std::string& component_id, float value, PinMappingType type);
    void apply_to_sensor(const std::string& component_id, float value, PinMappingType type);
    void apply_to_physics(const std::string& component_id, float value, PinMappingType type);

    // Find component pin by ID
    class CircuitPin* find_pin(const std::string& pin_id);

    CircuitSimulator* m_circuit_sim = nullptr;
    Registry* m_registry = nullptr;

    std::unordered_map<std::string, PinMapping> m_mappings;
    bool m_enabled = true;

    // Statistics
    size_t m_active_mappings = 0;
    double m_total_update_time = 0.0;
};

} // namespace mechatron
