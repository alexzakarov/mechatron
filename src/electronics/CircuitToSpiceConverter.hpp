#pragma once

#include "CircuitSimulator.hpp"
#include "NgspiceWrapper.hpp"
#include "SpiceModelLibrary.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>

namespace mechatron {

/**
 * Circuit Component to SPICE Converter
 *
 * Converts mechatron internal circuit components to SPICE netlist format.
 * Handles all component types including complex ones like motor drivers and DC-DC converters.
 */
class CircuitToSpiceConverter {
public:
    /**
     * Configuration for SPICE conversion
     */
    struct Config {
        bool use_standard_models = true;          // Include standard SPICE models
        bool include_simulation_command = true;   // Add .tran command to netlist
        double simulation_duration = 1.0;         // Default: 1 second
        double simulation_time_step = 0.001;      // Default: 1ms
        bool auto_assign_node_numbers = true;      // Automatically number nodes
        int start_node_number = 1;                 // Start numbering from this node
        bool include_ground = true;                // Add explicit ground node (0)
        std::string ground_node_name = "0";        // Ground node name
        std::string analysis_type = "tran";        // SPICE analysis command: op, tran, ac, dc, etc.
        bool verbose = false;                      // Print debug information
    };

    /**
     * Conversion result
     */
    struct Result {
        bool success;
        std::string netlist;                      // Complete SPICE netlist
        std::string error;                        // Error message if failed
        std::unordered_map<std::string, std::string> pin_to_node;  // Pin ID to node name mapping
        int total_nodes;                          // Total number of nodes
        int total_components;                     // Total number of components
    };

    CircuitToSpiceConverter();
    explicit CircuitToSpiceConverter(const Config& config);

    /**
     * Convert a list of circuit components to SPICE netlist
     * @param components List of circuit component pointers
     * @return Conversion result with netlist
     */
    Result convert(std::vector<CircuitComponent*>& components);

    /**
     * Convert a list of circuit components with wire connections
     * @param components List of circuit component pointers
     * @param wires Wire connections between components
     * @return Conversion result with netlist
     */
    Result convert_with_wires(std::vector<CircuitComponent*>& components,
                             const std::vector<Wire>& wires);

    /**
     * Set configuration
     */
    void set_config(const Config& config) { m_config = config; }

    /**
     * Get current configuration
     */
    const Config& config() const { return m_config; }

    /**
     * Get the netlist builder (for advanced customization)
     */
    NetlistBuilder& builder() { return m_builder; }

    /**
     * Reset converter state
     */
    void reset();

private:
    Config m_config;
    NetlistBuilder m_builder;
    int m_current_node_number;
    std::unordered_map<std::string, std::string> m_pin_to_node;
    std::unordered_map<std::string, std::string> m_component_models;

    // Internal helper methods
    Result convert_components(std::vector<CircuitComponent*>& components, bool reset_state);
    std::string generate_node_name(const std::string& pin_id);
    std::string get_node_for_pin(const std::string& component_id, const std::string& pin_name);
    void add_standard_models();
    void add_current_save_directives(const std::vector<CircuitComponent*>& components);

    // Component-specific conversion methods
    bool convert_resistor(const Resistor* resistor);
    bool convert_capacitor(const Capacitor* capacitor);
    bool convert_inductor(const Inductor* inductor);
    bool convert_diode(const Diode* diode);
    bool convert_led(const LED* led);
    bool convert_zener_diode(const ZenerDiode* zener);
    bool convert_bjt_transistor(const BJTTransistor* bjt);
    bool convert_mosfet_transistor(const MOSFETTransistor* mosfet);
    bool convert_motor_driver(const MotorDriver* driver);
    bool convert_buck_converter(const BuckConverter* buck);
    bool convert_boost_converter(const BoostConverter* boost);
    bool convert_dc_voltage_source(const DCVoltageSource* source);
    bool convert_ground(const Ground* ground);
    bool convert_h_bridge(const HBridge* hbridge);
    // Note: Pulse, sine sources, transformers, switches, potentiometers, op-amps
    // can be added as future enhancements if these component types are defined
    // bool_convert_pulse_voltage_source(const PulseVoltageSource* source);
    // bool_convert_sine_voltage_source(const SineVoltageSource* source);
    // bool_convert_transformer(const Transformer* transformer);
    // bool_convert_switch(const Switch* sw);
    // bool_convert_potentiometer(const Potentiometer* pot);
    // bool_convert_opamp(const OpAmp* opamp);

    // Generic component conversion (fallback)
    bool convert_generic_component(const CircuitComponent* component);

    // Utility methods
    std::string color_to_string(LED::Color color);
    std::string get_unique_model_name(const std::string& base_name);
};

/**
 * SPICE simulation mode selector
 *
 * Allows runtime selection between different simulation backends
 */
enum class SimulationMode {
    NativeMNA,      // Use built-in Modified Nodal Analysis solver
    Ngspice,        // Use ngspice for simulation (if available)
    Hybrid          // Automatically choose based on circuit complexity
};

/**
 * Extended circuit simulation result
 *
 * Extends the basic ngspice SimulationResult with additional information
 */
struct CircuitSimulationResult {
    bool success;
    std::string error;
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> node_voltages;  // time -> voltage
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> component_currents; // time -> current
    double simulation_time;  // Total simulation time
    int time_steps;         // Number of time steps
};

/**
 * Circuit Simulator with SPICE integration
 *
 * Enhanced CircuitSimulator that can use either native MNA or ngspice
 */
class CircuitSimulatorWithSpice {
public:
    explicit CircuitSimulatorWithSpice(SimulationMode mode = SimulationMode::Hybrid);

    /**
     * Set simulation mode
     */
    void set_simulation_mode(SimulationMode mode) { m_mode = mode; }

    /**
     * Get current simulation mode
     */
    SimulationMode simulation_mode() const { return m_mode; }

    /**
     * Add component to circuit
     */
    void add_component(CircuitComponent* component);

    /**
     * Add wire connection
     */
    void add_wire(const Wire& wire);

    /**
     * Run simulation for specified duration
     * @param duration Simulation duration in seconds
     * @param time_step Time step for output
     * @return Simulation result
     */
    CircuitSimulationResult simulate(double duration = 1.0, double time_step = 0.001);

    /**
     * Get component by ID
     */
    CircuitComponent* get_component(const std::string& id);

    /**
     * Get all components
     */
    const std::vector<CircuitComponent*>& components() const { return m_components; }

    /**
     * Reset simulator state
     */
    void reset();

    /**
     * Check if ngspice is available
     */
    bool is_ngspice_available() const;

private:
    SimulationMode m_mode;
    std::vector<CircuitComponent*> m_components;
    std::vector<Wire> m_wires;
    std::unique_ptr<NgspiceWrapper> m_ngspice;
    CircuitToSpiceConverter m_converter;

    // Helper methods
    bool should_use_ngspice() const;
    CircuitSimulationResult simulate_with_mna(double duration, double time_step);
    CircuitSimulationResult simulate_with_ngspice(double duration, double time_step);
};

} // namespace mechatron
