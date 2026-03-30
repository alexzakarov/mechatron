#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

// Include ngspice shared header for type definitions
// We need to define these types ourselves if the header is not available
#ifdef _WIN32
// Forward declarations for ngspice types
struct vector_info {
    char *v_name;
    int v_type;
    short v_flags;
    double *v_realdata;
    struct ngcomplex {
        double cx_real;
        double cx_imag;
    } *v_compdata;
    int v_length;
};

struct vecvalues {
    char* name;
    double creal;
    double cimag;
    bool is_scale;
    bool is_complex;
};

struct vecvaluesall {
    int veccount;      /* number of vectors in plot */
    int vecindex;      /* index of actual set of vectors */
    vecvalues** vecsa; /* values of actual set of vectors */
};

struct vecinfo {
    int number;
    char *vecname;
    bool is_real;
    void *pdvec;
    void *pdvecscale;
};

struct vecinfoall {
    char *name;
    char *title;
    char *date;
    char *type;
    int veccount;
    vecinfo **vecs;
};

typedef vector_info* pvector_info;
typedef vecvalues* pvecvalues;
typedef vecvaluesall* pvecvaluesall;
typedef vecinfoall* pvecinfoall;
#else
#include "ngspice/sharedspice.h"
#endif

namespace mechatron {

/**
 * Node voltage data from ngspice simulation
 */
struct NodeVoltage {
    std::string node;
    double voltage;
};

/**
 * Simulation result from ngspice
 */
struct SimulationResult {
    bool success;
    std::string error;
    std::vector<std::pair<double, std::vector<NodeVoltage>>> time_points;  // time -> node voltages
    std::vector<std::string> nodes;  // All node names
};

/**
 * ngspice circuit simulator wrapper
 *
 * Uses ngspice shared library (libngspice-0.dll/.so) to simulate electronic circuits.
 * Generates SPICE netlists and parses simulation results.
 */
class NgspiceWrapper {
public:
    NgspiceWrapper();
    ~NgspiceWrapper();

    /**
     * Check if ngspice is available
     */
    bool is_available();

    /**
     * Simulate circuit from netlist string
     * @param netlist SPICE netlist content
     * @param duration Simulation duration in seconds
     * @param time_step Time step for output
     * @return Simulation result
     */
    SimulationResult simulate(const std::string& netlist,
                             double duration = 1.0,
                             double time_step = 0.001);

    /**
     * Simulate circuit from netlist file
     * @param netlist_path Path to .cir file
     * @param duration Simulation duration in seconds
     * @param time_step Time step for output
     * @return Simulation result
     */
    SimulationResult simulate_file(const std::string& netlist_path,
                                   double duration = 1.0,
                                   double time_step = 0.001);

    /**
     * Generate a simple SPICE netlist from component list
     * @param components Map of component name to type and parameters
     * @param connections Wire connections
     * @return SPICE netlist string
     */
    std::string generate_netlist(
        const std::vector<std::tuple<std::string, std::string, std::unordered_map<std::string, double>>>& components,
        const std::vector<std::tuple<std::string, std::string, std::string>>& connections
    );

    /**
     * Get last error message
     */
    const std::string& error() const { return m_error; }

    /**
     * Set ngspice library path (if not in default location)
     */
    void set_library_path(const std::string& path) { m_library_path = path; }

private:
    bool load_library();
    void unload_library();
    bool init_ngspice();
    SimulationResult run_simulation(const std::vector<std::string>& netlist_lines);
    void collect_simulation_data();
    std::string get_error_message();

    std::string m_library_path;
    std::string m_error;
    void* m_library_handle;
    bool m_available;
    bool m_initialized;

    // Callback data
    std::vector<std::pair<double, std::vector<NodeVoltage>>> m_simulation_data;
    std::vector<std::string> m_node_names;
    std::string m_output_buffer;
    bool m_simulation_running;

    // Static callbacks for ngspice
    static int send_char(char* output, int id, void* user_data);
    static int send_stat(char* status, int id, void* user_data);
    static int controlled_exit(int exit_status, bool immediate, bool quit, int id, void* user_data);
    static int send_data(pvecvaluesall data, int num_vecs, int id, void* user_data);
    static int send_init_data(pvecinfoall vec_info, int id, void* user_data);
    static int bg_thread_running(bool is_running, int id, void* user_data);
};

/**
 * Helper class to build SPICE netlists programmatically
 */
class NetlistBuilder {
public:
    NetlistBuilder();

    /**
     * Set title of the netlist
     */
    void set_title(const std::string& title);

    /**
     * Add a resistor
     * @param name Component name (e.g., "R1")
     * @param node1 First node
     * @param node2 Second node
     * @param resistance Resistance in ohms
     */
    void add_resistor(const std::string& name,
                     const std::string& node1,
                     const std::string& node2,
                     double resistance);

    /**
     * Add a capacitor
     * @param name Component name (e.g., "C1")
     * @param node1 First node
     * @param node2 Second node
     * @param capacitance Capacitance in farads
     */
    void add_capacitor(const std::string& name,
                      const std::string& node1,
                      const std::string& node2,
                      double capacitance);

    /**
     * Add an inductor
     * @param name Component name (e.g., "L1")
     * @param node1 First node
     * @param node2 Second node
     * @param inductance Inductance in henries
     */
    void add_inductor(const std::string& name,
                     const std::string& node1,
                     const std::string& node2,
                     double inductance);

    /**
     * Add a voltage source
     * @param name Component name (e.g., "V1")
     * @param node_pos Positive node
     * @param node_neg Negative node
     * @param voltage Voltage in volts
     */
    void add_voltage_source(const std::string& name,
                           const std::string& node_pos,
                           const std::string& node_neg,
                           double voltage);

    /**
     * Add a DC voltage source
     * @param name Component name (e.g., "V1")
     * @param node_pos Positive node
     * @param node_neg Negative node
     * @param voltage DC voltage in volts
     */
    void add_dc_voltage(const std::string& name,
                       const std::string& node_pos,
                       const std::string& node_neg,
                       double voltage);

    /**
     * Add a pulse voltage source
     * @param name Component name
     * @param node_pos Positive node
     * @param node_neg Negative node
     * @param v_low Low voltage
     * @param v_high High voltage
     * @param delay Delay time
     * @param rise_time Rise time
     * @param fall_time Fall time
     * @param pulse_width Pulse width
     * @param period Period
     */
    void add_pulse_voltage(const std::string& name,
                          const std::string& node_pos,
                          const std::string& node_neg,
                          double v_low, double v_high,
                          double delay, double rise_time, double fall_time,
                          double pulse_width, double period);

    /**
     * Add a diode
     * @param name Component name (e.g., "D1")
     * @param node_pos Positive node (anode)
     * @param node_neg Negative node (cathode)
     * @param model Diode model name
     */
    void add_diode(const std::string& name,
                  const std::string& node_pos,
                  const std::string& node_neg,
                  const std::string& model = "D1N4148");

    /**
     * Add an LED
     * @param name Component name (e.g., "LED1")
     * @param node_pos Positive node (anode)
     * @param node_neg Negative node (cathode)
     * @param forward_voltage Forward voltage drop
     */
    void add_led(const std::string& name,
                const std::string& node_pos,
                const std::string& node_neg,
                double forward_voltage = 2.0);

    /**
     * Add a transistor (BJT)
     * @param name Component name (e.g., "Q1")
     * @param node_collector Collector node
     * @param node_base Base node
     * @param node_emitter Emitter node
     * @param model Transistor model name
     */
    void add_bjt(const std::string& name,
                const std::string& node_collector,
                const std::string& node_base,
                const std::string& node_emitter,
                const std::string& model = "NPN");

    /**
     * Add simulation command
     * @param type "tran" for transient, "ac" for AC, "dc" for DC
     * @param parameters Simulation parameters
     */
    void add_simulation(const std::string& type, const std::string& parameters);

    /**
     * Add plot command
     * @param nodes Nodes to plot
     */
    void add_plot(const std::vector<std::string>& nodes);

    /**
     * Build the netlist string
     * @return Complete SPICE netlist
     */
    std::string build() const;

    /**
     * Clear all components
     */
    void clear();

private:
    std::string m_title;
    std::vector<std::string> m_components;
    std::vector<std::string> m_models;
    std::vector<std::string> m_commands;
    int m_line_number;
};

} // namespace mechatron
