#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace mechatron {

/**
 * SPICE Model Library
 *
 * Provides realistic SPICE model parameters for various electronic components.
 * Models are based on typical semiconductor parameters and industry standards.
 */
class SpiceModelLibrary {
public:
    /**
     * Model categories
     */
    enum class ModelType {
        Diode,
        BJT_NPN,
        BJT_PNP,
        NMOS,
        PMOS,
        NJFET,
        PJFET,
        OpAmp,
        LED
    };

    /**
     * Get a SPICE .model statement for a diode
     * @param model_name Name for the model
     * @param diode_type Type of diode (signal, rectifier, schottky, zener)
     * @return Complete .model statement
     */
    static std::string get_diode_model(const std::string& model_name,
                                      const std::string& diode_type = "1N4148");

    /**
     * Get a SPICE .model statement for a BJT
     * @param model_name Name for the model
     * @param transistor_type Specific transistor (2N2222, 2N3904, BC547, etc.)
     * @param is_npn true for NPN, false for PNP
     * @return Complete .model statement
     */
    static std::string get_bjt_model(const std::string& model_name,
                                     const std::string& transistor_type = "2N2222",
                                     bool is_npn = true);

    /**
     * Get a SPICE .model statement for a MOSFET
     * @param model_name Name for the model
     * @param mosfet_type Specific MOSFET type (power, logic, etc.)
     * @param is_nmos true for NMOS, false for PMOS
     * @return Complete .model statement
     */
    static std::string get_mosfet_model(const std::string& model_name,
                                       const std::string& mosfet_type = "IRF540",
                                       bool is_nmos = true);

    /**
     * Get a SPICE .model statement for a JFET
     * @param model_name Name for the model
     * @param jfet_type Specific JFET type (J310, 2N3819, etc.)
     * @param is_njfet true for N-channel, false for P-channel
     * @return Complete .model statement
     */
    static std::string get_jfet_model(const std::string& model_name,
                                     const std::string& jfet_type = "2N3819",
                                     bool is_njfet = true);

    /**
     * Get a SPICE .model statement for an LED
     * @param model_name Name for the model
     * @param color LED color (red, green, blue, white, yellow, etc.)
     * @return Complete .model statement
     */
    static std::string get_led_model(const std::string& model_name,
                                    const std::string& color = "red");

    /**
     * Get SPICE subcircuit definition for an op-amp
     * @param model_name Name for the subcircuit
     * @param opamp_type Op-amp type (UA741, LM358, TL081, etc.)
     * @return Complete .subckt definition
     */
    static std::string get_opamp_subcircuit(const std::string& model_name,
                                          const std::string& opamp_type = "UA741");

    /**
     * Get voltage-controlled switch model
     * @param model_name Name for the model
     * @param r_on On resistance (ohms)
     * @param r_off Off resistance (ohms)
     * @param v_threshold Threshold voltage (volts)
     * @param v_hysteresis Hysteresis voltage (volts)
     * @return Complete .model statement
     */
    static std::string get_vswitch_model(const std::string& model_name,
                                        double r_on = 0.01,
                                        double r_off = 1e9,
                                        double v_threshold = 0.5,
                                        double v_hysteresis = 0.0);

    /**
     * Get current-controlled switch model
     * @param model_name Name for the model
     * @param r_on On resistance (ohms)
     * @param r_off Off resistance (ohms)
     * @param i_threshold Threshold current (amps)
     * @param i_hysteresis Hysteresis current (amps)
     * @return Complete .model statement
     */
    static std::string get_cswitch_model(const std::string& model_name,
                                        double r_on = 0.01,
                                        double r_off = 1e9,
                                        double i_threshold = 0.1,
                                        double i_hysteresis = 0.0);

    /**
     * Get motor driver MOSFET model (optimized for H-bridge applications)
     * @param model_name Name for the model
     * @param is_nmos true for NMOS, false for PMOS
     * @param v_ds_rating Drain-source voltage rating (V)
     * @param r_ds_on On-resistance (milliohms)
     * @return Complete .model statement
     */
    static std::string get_motor_mosfet_model(const std::string& model_name,
                                             bool is_nmos = true,
                                             double v_ds_rating = 30.0,
                                             double r_ds_on = 10.0);

    /**
     * Get power MOSFET model for DC-DC converters
     * @param model_name Name for the model
     * @param is_nmos true for NMOS, false for PMOS
     * @param v_ds_rating Drain-source voltage rating (V)
     * @param r_ds_on On-resistance (milliohms)
     * @param q_gate Gate charge (nC)
     * @return Complete .model statement
     */
    static std::string get_power_mosfet_model(const std::string& model_name,
                                             bool is_nmos = true,
                                             double v_ds_rating = 60.0,
                                             double r_ds_on = 5.0,
                                             double q_gate = 20.0);

    /**
     * Get all standard models as a vector of strings
     * Useful for initializing a netlist with common models
     * @return Vector of .model and .subckt statements
     */
    static std::vector<std::string> get_standard_models();

    /**
     * Get model library version
     */
    static const char* version() { return "1.0.0"; }

private:
    // Diode model parameters
    struct DiodeParams {
        double is;      // Saturation current (A)
        double n;       // Emission coefficient
        double rs;      // Series resistance (ohms)
        double cj0;     // Zero-bias junction capacitance (F)
        double vj;      // Junction potential (V)
        double bv;      // Reverse breakdown voltage (V)
        double ibv;     // Current at breakdown voltage (A)
        double tt;      // Transit time (s)
    };

    // BJT Gummel-Poon model parameters
    struct BJTParams {
        double is;      // Transport saturation current (A)
        double bf;      // Ideal maximum forward beta
        double nf;      // Forward current emission coefficient
        double vaf;     // Forward Early voltage (V)
        double ikf;     // Corner for forward beta high-current roll-off (A)
        double ise;     // B-E leakage saturation current (A)
        double ne;      // B-E leakage emission coefficient
        double br;      // Ideal maximum reverse beta
        double nr;      // Reverse current emission coefficient
        double var;     // Reverse Early voltage (V)
        double cje;     // B-E zero-bias depletion capacitance (F)
        double vje;     // B-E built-in potential (V)
        double mje;     // B-E grading coefficient
        double cjc;     // B-C zero-bias depletion capacitance (F)
        double vjc;     // B-C built-in potential (V)
        double mjc;     // B-C grading coefficient
        double tf;      // Ideal forward transit time (s)
        double tr;      // Ideal reverse transit time (s)
    };

    // MOSFET BSIM model parameters (simplified)
    struct MOSFETParams {
        double vto;     // Zero-bias threshold voltage (V)
        double kp;      // Transconductance (A/V²)
        double lambda;  // Channel-length modulation (1/V)
        double rd;      // Drain ohmic resistance (ohms)
        double rs;      // Source ohmic resistance (ohms)
        double cbd;     // B-D junction capacitance (F)
        double cbs;     // B-S junction capacitance (F)
        double is;      // Bulk junction saturation current (A)
        double pb;      // Bulk junction potential (V)
        double cgso;    // Gate-source overlap capacitance per meter (F/m)
        double cgdo;    // Gate-drain overlap capacitance per meter (F/m)
        double tox;     // Oxide thickness (m)
        double nsub;    // Substrate doping (1/cm³)
    };

    // Helper functions to get predefined parameters
    static DiodeParams get_diode_params(const std::string& diode_type);
    static BJTParams get_bjt_params(const std::string& transistor_type);
    static MOSFETParams get_mosfet_params(const std::string& mosfet_type);
};

/**
 * SPICE parameter mapping utilities
 *
 * Maps C++ component parameters to SPICE model parameters
 */
class SpiceParameterMapper {
public:
    /**
     * Convert resistance value to SPICE format (with metric prefixes)
     */
    static std::string format_resistance(double ohms);

    /**
     * Convert capacitance value to SPICE format (with metric prefixes)
     */
    static std::string format_capacitance(double farads);

    /**
     * Convert inductance value to SPICE format (with metric prefixes)
     */
    static std::string format_inductance(double henries);

    /**
     * Convert voltage value to SPICE format (with metric prefixes)
     */
    static std::string format_voltage(double volts);

    /**
     * Convert current value to SPICE format (with metric prefixes)
     */
    static std::string format_current(double amps);

    /**
     * Convert frequency value to SPICE format (with metric prefixes)
     */
    static std::string format_frequency(double hz);

    /**
     * Convert time value to SPICE format (with metric prefixes)
     */
    static std::string format_time(double seconds);

private:
    static std::string format_value(double value, const char* units);
};

} // namespace mechatron
