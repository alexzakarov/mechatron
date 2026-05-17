#pragma once

#include <string>
#include <vector>
#include <cstddef>
#include <memory>
#include <unordered_map>

// Include ngspice shared header for type definitions
// We need to define these types ourselves if the header is not available

// By default, ngspice support is disabled unless explicitly enabled via CMake
// This allows the codebase to compile without ngspice installed
#ifndef MECHATRON_HAVE_NGSPICE
  #define MECHATRON_HAVE_NGSPICE 0
#endif

// Include ngspice header if available
#if MECHATRON_HAVE_NGSPICE
  #include "ngspice/sharedspice.h"
#else
  // Forward declarations for ngspice types (when header is not available)
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

    // Cache: avoid re-loading circuit when netlist content is unchanged.
    size_t m_loaded_netlist_hash = 0;
    bool m_has_loaded_netlist = false;

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
     * Add a MOSFET (NMOS/PMOS)
     * @param name Component name (e.g., "M1")
     * @param node_drain Drain node
     * @param node_gate Gate node
     * @param node_source Source node
     * @param node_bulk Bulk/substrate node (can be same as source for discrete devices)
     * @param model MOSFET model name
     * @param length Channel length in meters (for BSIM)
     * @param width Channel width in meters (for BSIM)
     */
    void add_mosfet(const std::string& name,
                   const std::string& node_drain,
                   const std::string& node_gate,
                   const std::string& node_source,
                   const std::string& node_bulk,
                   const std::string& model = "NMOS",
                   double length = 1e-6,
                   double width = 10e-6);

    /**
     * Add a JFET
     * @param name Component name (e.g., "J1")
     * @param node_drain Drain node
     * @param node_gate Gate node
     * @param node_source Source node
     * @param model JFET model name
     */
    void add_jfet(const std::string& name,
                 const std::string& node_drain,
                 const std::string& node_gate,
                 const std::string& node_source,
                 const std::string& model = "NJF");

    /**
     * Add an op-amp (using macromodel)
     * @param name Component name (e.g., "XU1")
     * @param node_non_inv Non-inverting input
     * @param node_inv Inverting input
     * @param node_vcc Positive supply
     * @param node_vee Negative supply
     * @param node_out Output
     * @param model Op-amp model name (e.g., "UA741")
     */
    void add_opamp(const std::string& name,
                  const std::string& node_non_inv,
                  const std::string& node_inv,
                  const std::string& node_vcc,
                  const std::string& node_vee,
                  const std::string& node_out,
                  const std::string& model = "UA741");

    /**
     * Add a DC current source
     * @param name Component name (e.g., "I1")
     * @param node_pos Positive node (current flows out)
     * @param node_neg Negative node (current flows in)
     * @param current Current in amps
     */
    void add_dc_current(const std::string& name,
                       const std::string& node_pos,
                       const std::string& node_neg,
                       double current);

    /**
     * Add a sinusoidal voltage source
     * @param name Component name
     * @param node_pos Positive node
     * @param node_neg Negative node
     * @param offset DC offset voltage
     * @param amplitude AC amplitude voltage
     * @param frequency Frequency in Hz
     */
    void add_sine_voltage(const std::string& name,
                         const std::string& node_pos,
                         const std::string& node_neg,
                         double offset,
                         double amplitude,
                         double frequency);

    /**
     * Add a pulse current source
     * @param name Component name
     * @param node_pos Positive node
     * @param node_neg Negative node
     * @param i_low Low current
     * @param i_high High current
     * @param delay Delay time
     * @param rise_time Rise time
     * @param fall_time Fall time
     * @param pulse_width Pulse width
     * @param period Period
     */
    void add_pulse_current(const std::string& name,
                          const std::string& node_pos,
                          const std::string& node_neg,
                          double i_low, double i_high,
                          double delay, double rise_time, double fall_time,
                          double pulse_width, double period);

    /**
     * Add a transformer
     * @param name Component name (e.g., "K1")
     * @param inductor1 Primary inductor name
     * @param inductor2 Secondary inductor name
     * @param coupling_coefficient Coupling coefficient (0-1)
     */
    void add_transformer(const std::string& name,
                        const std::string& inductor1,
                        const std::string& inductor2,
                        double coupling_coefficient = 0.99);

    /**
     * Add a voltage-controlled switch
     * @param name Component name (e.g., "S1")
     * @param node_pos Positive node of switch
     * @param node_neg Negative node of switch
     * @param node_ctrl_pos Positive control node
     * @param node_ctrl_neg Negative control node
     * @param model Switch model name
     */
    void add_vswitch(const std::string& name,
                    const std::string& node_pos,
                    const std::string& node_neg,
                    const std::string& node_ctrl_pos,
                    const std::string& node_ctrl_neg,
                    const std::string& model = "SWITCH");

    /**
     * Add a current-controlled switch
     * @param name Component name (e.g., "W1")
     * @param node_pos Positive node of switch
     * @param node_neg Negative node of switch
     * @param controlling_source Voltage source through which control current flows
     * @param model Switch model name
     */
    void add_cswitch(const std::string& name,
                    const std::string& node_pos,
                    const std::string& node_neg,
                    const std::string& controlling_source,
                    const std::string& model = "CSWITCH");

    /**
     * Add a resistor with temperature coefficient
     * @param name Component name
     * @param node1 First node
     * @param node2 Second node
     * @param resistance Resistance at nominal temperature
     * @param temp_coeff Temperature coefficient (ppm/°C or 1/°C)
     * @param nominal_temp Nominal temperature in °C
     */
    void add_resistor_tc(const std::string& name,
                        const std::string& node1,
                        const std::string& node2,
                        double resistance,
                        double temp_coeff = 0.0,
                        double nominal_temp = 27.0);

    /**
     * Add a capacitor with initial voltage
     * @param name Component name
     * @param node1 First node
     * @param node2 Second node
     * @param capacitance Capacitance in farads
     * @param initial_voltage Initial voltage across capacitor
     */
    void add_capacitor_ic(const std::string& name,
                         const std::string& node1,
                         const std::string& node2,
                         double capacitance,
                         double initial_voltage = 0.0);

    /**
     * Add an inductor with initial current
     * @param name Component name
     * @param node1 First node
     * @param node2 Second node
     * @param inductance Inductance in henries
     * @param initial_current Initial current through inductor
     */
    void add_inductor_ic(const std::string& name,
                        const std::string& node1,
                        const std::string& node2,
                        double inductance,
                        double initial_current = 0.0);

    /**
     * Add a subcircuit instance (for complex components like op-amps, ICs)
     * @param name Instance name (e.g., "XU1")
     * @param subcircuit_name Subcircuit definition name
     * @param nodes Connection nodes
     * @param parameters Optional parameters
     */
    void add_subcircuit(const std::string& name,
                       const std::string& subcircuit_name,
                       const std::vector<std::string>& nodes,
                       const std::vector<std::pair<std::string, double>>& parameters = {});

    /**
     * Add a motor driver H-bridge (composite subcircuit)
     * @param name Instance name
     * @param node_vcc Power supply
     * @param node_gnd Ground
     * @param node_in1 Input 1 (PWM A)
     * @param node_in2 Input 2 (PWM B)
     * @param node_in3 Input 3 (PWM C)
     * @param node_in4 Input 4 (PWM D)
     * @param node_out1 Motor terminal 1
     * @param node_out2 Motor terminal 2
     * @param mosfet_model MOSFET model to use
     */
    void add_motor_driver(const std::string& name,
                         const std::string& node_vcc,
                         const std::string& node_gnd,
                         const std::string& node_in1,
                         const std::string& node_in2,
                         const std::string& node_in3,
                         const std::string& node_in4,
                         const std::string& node_out1,
                         const std::string& node_out2,
                         const std::string& mosfet_model = "NMOS_DRV");

    void add_h_bridge(const std::string& name,
                     const std::string& node_vcc,
                     const std::string& node_gnd,
                     const std::string& node_in1,
                     const std::string& node_in2,
                     const std::string& node_en,
                     const std::string& node_out1,
                     const std::string& node_out2);

    /**
     * Add a buck converter (composite subcircuit with internal PWM)
     * @param name Instance name
     * @param node_vin Input voltage
     * @param node_gnd Ground
     * @param node_vout Output voltage
     * @param inductance Output inductor in henries
     * @param capacitance Output capacitor in farads
     * @param switching_freq Switching frequency in Hz
     * @param mosfet_model MOSFET model to use
     * Note: PWM is generated internally at 50% duty cycle
     */
    void add_buck_converter(const std::string& name,
                           const std::string& node_vin,
                           const std::string& node_gnd,
                           const std::string& node_vout,
                           double inductance,
                           double capacitance,
                           double switching_freq,
                           const std::string& mosfet_model = "NMOS_PWR",
                           double duty_cycle = 0.5);

    /**
     * Add a boost converter (composite subcircuit with internal PWM)
     * @param name Instance name
     * @param node_vin Input voltage
     * @param node_gnd Ground
     * @param node_vout Output voltage
     * @param inductance Input inductor in henries
     * @param capacitance Output capacitor in farads
     * @param switching_freq Switching frequency in Hz
     * @param mosfet_model MOSFET model to use
     * Note: PWM is generated internally at 50% duty cycle
     */
    void add_boost_converter(const std::string& name,
                           const std::string& node_vin,
                           const std::string& node_gnd,
                           const std::string& node_vout,
                           double inductance,
                           double capacitance,
                           double switching_freq,
                           const std::string& mosfet_model = "NMOS_PWR",
                           double duty_cycle = 0.5);

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
     * Add a model or subcircuit definition directly
     * @param model_def Complete .model or .subckt statement
     */
    void add_model(const std::string& model_def);

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
