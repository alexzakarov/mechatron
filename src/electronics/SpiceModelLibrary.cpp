#include "SpiceModelLibrary.hpp"
#include <sstream>
#include <cmath>
#include <spdlog/spdlog.h>

namespace mechatron {

// ============================================================================
// SpiceModelLibrary
// ============================================================================

std::string SpiceModelLibrary::get_diode_model(const std::string& model_name,
                                              const std::string& diode_type) {
    DiodeParams params = get_diode_params(diode_type);

    std::stringstream ss;
    ss << ".model " << model_name << " D("
       << "Is=" << SpiceParameterMapper::format_current(params.is) << " "
       << "N=" << params.n << " "
       << "Rs=" << SpiceParameterMapper::format_resistance(params.rs) << " "
       << "Cjo=" << SpiceParameterMapper::format_capacitance(params.cj0) << " "
       << "Vj=" << params.vj << " "
       << "Bv=" << params.bv << " "
       << "Ibv=" << SpiceParameterMapper::format_current(params.ibv) << " "
       << "Tt=" << SpiceParameterMapper::format_time(params.tt) << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_bjt_model(const std::string& model_name,
                                             const std::string& transistor_type,
                                             bool is_npn) {
    BJTParams params = get_bjt_params(transistor_type);

    std::string type = is_npn ? "NPN" : "PNP";

    std::stringstream ss;
    ss << ".model " << model_name << " " << type << "("
       << "Is=" << SpiceParameterMapper::format_current(params.is) << " "
       << "Bf=" << params.bf << " "
       << "Nf=" << params.nf << " "
       << "Vaf=" << params.vaf << " "
       << "Ikf=" << SpiceParameterMapper::format_current(params.ikf) << " "
       << "Ise=" << SpiceParameterMapper::format_current(params.ise) << " "
       << "Ne=" << params.ne << " "
       << "Br=" << params.br << " "
       << "Nr=" << params.nr << " "
       << "Var=" << params.var << " "
       << "Cje=" << SpiceParameterMapper::format_capacitance(params.cje) << " "
       << "Vje=" << params.vje << " "
       << "Mje=" << params.mje << " "
       << "Cjc=" << SpiceParameterMapper::format_capacitance(params.cjc) << " "
       << "Vjc=" << params.vjc << " "
       << "Mjc=" << params.mjc << " "
       << "Tf=" << SpiceParameterMapper::format_time(params.tf) << " "
       << "Tr=" << SpiceParameterMapper::format_time(params.tr) << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_mosfet_model(const std::string& model_name,
                                                const std::string& mosfet_type,
                                                bool is_nmos) {
    MOSFETParams params = get_mosfet_params(mosfet_type);

    std::string type = is_nmos ? "NMOS" : "PMOS";

    std::stringstream ss;
    ss << ".model " << model_name << " " << type << "("
       << "Vto=" << params.vto << " "
       << "Kp=" << params.kp << " "
       << "Lambda=" << params.lambda << " "
       << "Rd=" << SpiceParameterMapper::format_resistance(params.rd) << " "
       << "Rs=" << SpiceParameterMapper::format_resistance(params.rs) << " "
       << "Cbd=" << SpiceParameterMapper::format_capacitance(params.cbd) << " "
       << "Cbs=" << SpiceParameterMapper::format_capacitance(params.cbs) << " "
       << "Is=" << SpiceParameterMapper::format_current(params.is) << " "
       << "Pb=" << params.pb << " "
       << "Cgso=" << SpiceParameterMapper::format_capacitance(params.cgso) << " "
       << "Cgdo=" << SpiceParameterMapper::format_capacitance(params.cgdo) << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_jfet_model(const std::string& model_name,
                                             const std::string& jfet_type,
                                             bool is_njfet) {
    // Simplified JFET model parameters
    double vto = -2.0;      // Pinch-off voltage (V)
    double beta = 1e-3;     // Transconductance parameter (A/V²)
    double lambda = 0.01;   // Channel-length modulation
    double rd = 100.0;      // Drain resistance
    double rs = 100.0;      // Source resistance
    double cgs = 5e-12;     // G-S capacitance
    double cgd = 2e-12;     // G-D capacitance
    double is = 1e-14;      // Gate junction saturation current
    double pb = 0.8;        // Gate junction potential

    // Adjust for specific JFET types
    if (jfet_type == "J310" || jfet_type == "2N5457") {
        vto = -4.0;
        beta = 3e-4;
    } else if (jfet_type == "2N3819") {
        vto = -3.0;
        beta = 1.2e-3;
    }

    if (!is_njfet) {
        vto = 2.0;  // Positive for P-channel
    }

    std::string type = is_njfet ? "NJF" : "PJF";

    std::stringstream ss;
    ss << ".model " << model_name << " " << type << "("
       << "Vto=" << vto << " "
       << "Beta=" << beta << " "
       << "Lambda=" << lambda << " "
       << "Rd=" << rd << " "
       << "Rs=" << rs << " "
       << "Cgs=" << SpiceParameterMapper::format_capacitance(cgs) << " "
       << "Cgd=" << SpiceParameterMapper::format_capacitance(cgd) << " "
       << "Is=" << SpiceParameterMapper::format_current(is) << " "
       << "Pb=" << pb << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_led_model(const std::string& model_name,
                                            const std::string& color) {
    // LED forward voltages by color (typical values at 20mA)
    std::unordered_map<std::string, double> led_voltages = {
        {"red", 2.0},
        {"green", 2.2},
        {"yellow", 2.1},
        {"orange", 2.0},
        {"blue", 3.3},
        {"white", 3.3},
        {"uv", 3.6},
        {"ir", 1.2},
        {"pink", 3.1},
        {"amber", 2.1}
    };

    // LED capacitance (inverse to forward voltage roughly)
    std::unordered_map<std::string, double> led_cap = {
        {"red", 50e-12},
        {"green", 40e-12},
        {"yellow", 45e-12},
        {"orange", 50e-12},
        {"blue", 30e-12},
        {"white", 30e-12},
        {"uv", 25e-12},
        {"ir", 80e-12},
        {"pink", 35e-12},
        {"amber", 45e-12}
    };

    double vf = 2.0;  // Default
    double cj0 = 50e-12;  // Default

    auto it_v = led_voltages.find(color);
    if (it_v != led_voltages.end()) {
        vf = it_v->second;
    }

    auto it_c = led_cap.find(color);
    if (it_c != led_cap.end()) {
        cj0 = it_c->second;
    }

    // LED model parameters
    double is = 1e-14;      // Very small saturation current
    double n = 2.0;         // Emission coefficient for LEDs
    double rs = 1.0;        // Series resistance
    double tt = 10e-9;      // Transit time

    std::stringstream ss;
    ss << ".model " << model_name << "_led D("
       << "Is=" << SpiceParameterMapper::format_current(is) << " "
       << "N=" << n << " "
       << "Rs=" << rs << " "
       << "Cjo=" << SpiceParameterMapper::format_capacitance(cj0) << " "
       << "Vj=" << vf << " "
       << "Tt=" << SpiceParameterMapper::format_time(tt) << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_opamp_subcircuit(const std::string& model_name,
                                                   const std::string& opamp_type) {
    // Simplified op-amp macromodel based on Boyle model
    // This is a basic implementation that works for most general-purpose applications

    std::stringstream ss;

    if (opamp_type == "UA741" || opamp_type == "LM741") {
        // Classic uA741 macromodel
        ss << ".subckt " << model_name << " 1 2 3 4 5\n";  // + - V+ V- OUT
        ss << "* Input stage\n";
        ss << "R1 1 6 100k\n";
        ss << "R2 2 7 100k\n";
        ss << "C1 6 7 30pF\n";
        ss << "G1 4 8 6 7 1u\n";  // VCCS
        ss << "R3 8 4 100k\n";
        ss << "* Intermediate stage\n";
        ss << "C2 8 9 100pF\n";
        ss << "G2 4 9 8 4 100u\n";
        ss << "* Output stage\n";
        ss << "RO 9 5 100\n";
        ss << ".ends\n";
    } else if (opamp_type == "LM358") {
        // LM358 (dual op-amp, similar topology)
        ss << ".subckt " << model_name << " 1 2 3 4 5\n";
        ss << "R1 1 6 100k\n";
        ss << "R2 2 7 100k\n";
        ss << "C1 6 7 30pF\n";
        ss << "G1 4 8 6 7 1u\n";
        ss << "R3 8 4 100k\n";
        ss << "C2 8 9 100pF\n";
        ss << "G2 4 9 8 4 100u\n";
        ss << "RO 9 5 50\n";
        ss << ".ends\n";
    } else if (opamp_type == "TL081" || opamp_type == "TL071") {
        // JFET-input op-amp (higher input impedance)
        ss << ".subckt " << model_name << " 1 2 3 4 5\n";
        ss << "* JFET input stage\n";
        ss << "J1 1 10 4 JFET_INPUT\n";
        ss << "J2 2 11 4 JFET_INPUT\n";
        ss << "R1 10 3 10k\n";
        ss << "R2 11 3 10k\n";
        ss << "C1 10 11 10pF\n";
        ss << "G1 4 12 10 11 10u\n";
        ss << "R3 12 4 100k\n";
        ss << "C2 12 13 50pF\n";
        ss << "G2 4 13 12 4 100u\n";
        ss << "RO 13 5 50\n";
        ss << ".model JFET_INPUT NJF(Vto=-2.0 Beta=1e-3)\n";
        ss << ".ends\n";
    } else {
        // Generic op-amp macromodel
        ss << ".subckt " << model_name << " 1 2 3 4 5\n";
        ss << "Rin 1 2 1e9\n";
        ss << "G1 0 6 1 2 1u\n";
        ss << "R1 6 0 1e6\n";
        ss << "C1 6 0 100pF\n";
        ss << "E1 4 7 6 0 1e5\n";
        ss << "RO 7 5 100\n";
        ss << ".ends\n";
    }

    std::string result = ss.str();
    spdlog::debug("[SpiceModelLibrary] Generated opamp model ({} chars, contains .ends: {})",
                  result.length(), result.find(".ends") != std::string::npos);
    return result;
}

std::string SpiceModelLibrary::get_vswitch_model(const std::string& model_name,
                                                double r_on,
                                                double r_off,
                                                double v_threshold,
                                                double v_hysteresis) {
    std::stringstream ss;
    ss << ".model " << model_name << " SW("
       << "Ron=" << r_on << " "
       << "Roff=" << r_off << " "
       << "Vt=" << v_threshold << " "
       << "Vh=" << v_hysteresis << ")";
    return ss.str();
}

std::string SpiceModelLibrary::get_cswitch_model(const std::string& model_name,
                                                double r_on,
                                                double r_off,
                                                double i_threshold,
                                                double i_hysteresis) {
    std::stringstream ss;
    ss << ".model " << model_name << " CSW("
       << "Ron=" << r_on << " "
       << "Roff=" << r_off << " "
       << "It=" << i_threshold << " "
       << "Ih=" << i_hysteresis << ")";
    return ss.str();
}

std::string SpiceModelLibrary::get_motor_mosfet_model(const std::string& model_name,
                                                     bool is_nmos,
                                                     double v_ds_rating,
                                                     double r_ds_on) {
    // Motor driver MOSFETs are optimized for:
    // - Low Rds(on)
    // - High current capability
    // - Fast switching
    // - Robust body diode

    double vto = is_nmos ? 2.0 : -2.0;
    double kp = is_nmos ? 20.0 : 10.0;
    double lambda = 0.01;
    double rd = r_ds_on / 1000.0;  // Convert mOhm to Ohm
    double rs = rd * 0.5;
    double cbd = 1e-9;
    double cbs = 1e-9;
    double is = 1e-12;
    double pb = 0.8;
    double cgso = 1e-9;
    double cgdo = 1e-9;

    std::string type = is_nmos ? "NMOS" : "PMOS";

    std::stringstream ss;
    ss << ".model " << model_name << " " << type << "("
       << "Vto=" << vto << " "
       << "Kp=" << kp << " "
       << "Lambda=" << lambda << " "
       << "Rd=" << SpiceParameterMapper::format_resistance(rd) << " "
       << "Rs=" << SpiceParameterMapper::format_resistance(rs) << " "
       << "Cbd=" << SpiceParameterMapper::format_capacitance(cbd) << " "
       << "Cbs=" << SpiceParameterMapper::format_capacitance(cbs) << " "
       << "Is=" << SpiceParameterMapper::format_current(is) << " "
       << "Pb=" << pb << " "
       << "Cgso=" << SpiceParameterMapper::format_capacitance(cgso) << " "
       << "Cgdo=" << SpiceParameterMapper::format_capacitance(cgdo) << ")";

    return ss.str();
}

std::string SpiceModelLibrary::get_power_mosfet_model(const std::string& model_name,
                                                     bool is_nmos,
                                                     double v_ds_rating,
                                                     double r_ds_on,
                                                     double q_gate) {
    // Power MOSFETs for DC-DC converters are optimized for:
    // - Very low Rds(on)
    // - Low gate charge (fast switching)
    // - Low gate-source threshold
    // - High dv/dt capability

    double vto = is_nmos ? 1.5 : -1.5;  // Lower threshold for logic-level gate drive
    double kp = is_nmos ? 30.0 : 15.0;
    double lambda = 0.005;
    double rd = r_ds_on / 1000.0;  // Convert mOhm to Ohm
    double rs = rd * 0.5;
    double cbd = 500e-12;  // Higher capacitance for power devices
    double cbs = 500e-12;
    double is = 1e-12;
    double pb = 0.8;

    // Gate capacitance based on gate charge
    // Qg = Cgs * Vgs roughly
    double c_oss = q_gate / 5.0 * 1e-9;  // Estimate output capacitance
    double cgso = c_oss * 0.5;
    double cgdo = c_oss * 0.3;

    std::string type = is_nmos ? "NMOS" : "PMOS";

    std::stringstream ss;
    ss << ".model " << model_name << " " << type << "("
       << "Vto=" << vto << " "
       << "Kp=" << kp << " "
       << "Lambda=" << lambda << " "
       << "Rd=" << SpiceParameterMapper::format_resistance(rd) << " "
       << "Rs=" << SpiceParameterMapper::format_resistance(rs) << " "
       << "Cbd=" << SpiceParameterMapper::format_capacitance(cbd) << " "
       << "Cbs=" << SpiceParameterMapper::format_capacitance(cbs) << " "
       << "Is=" << SpiceParameterMapper::format_current(is) << " "
       << "Pb=" << pb << " "
       << "Cgso=" << SpiceParameterMapper::format_capacitance(cgso) << " "
       << "Cgdo=" << SpiceParameterMapper::format_capacitance(cgdo) << ")";

    return ss.str();
}

std::vector<std::string> SpiceModelLibrary::get_standard_models() {
    std::vector<std::string> models;

    // Standard diodes
    models.push_back(get_diode_model("D1N4148", "1N4148"));
    models.push_back(get_diode_model("D1N4007", "1N4007"));
    models.push_back(get_diode_model("D1N5819", "1N5819"));

    // Standard BJTs
    models.push_back(get_bjt_model("Q2N2222", "2N2222", true));
    models.push_back(get_bjt_model("Q2N3904", "2N3904", true));
    models.push_back(get_bjt_model("Q2N3906", "2N3906", false));
    models.push_back(get_bjt_model("QBC547", "BC547", true));

    // Standard MOSFETs
    models.push_back(get_mosfet_model("IRF540", "IRF540", true));
    models.push_back(get_mosfet_model("IRF9540", "IRF9540", false));
    models.push_back(get_mosfet_model("LOGIC_NMOS", "logic_nmos", true));
    models.push_back(get_mosfet_model("LOGIC_PMOS", "logic_pmos", false));

    // Standard LEDs
    models.push_back(get_led_model("LED_RED", "red"));
    models.push_back(get_led_model("LED_GREEN", "green"));
    models.push_back(get_led_model("LED_BLUE", "blue"));
    models.push_back(get_led_model("LED_WHITE", "white"));

    // Standard op-amps
    models.push_back(get_opamp_subcircuit("UA741", "UA741"));
    models.push_back(get_opamp_subcircuit("LM358", "LM358"));

    // Switches
    models.push_back(get_vswitch_model("SWITCH", 0.01, 1e9, 0.5, 0.0));
    models.push_back(get_cswitch_model("CSWITCH", 0.01, 1e9, 0.1, 0.0));

    // Motor driver MOSFETs
    models.push_back(get_motor_mosfet_model("NMOS_DRV", true, 30.0, 10.0));
    models.push_back(get_motor_mosfet_model("PMOS_DRV", false, 30.0, 15.0));

    // Power MOSFETs
    models.push_back(get_power_mosfet_model("NMOS_PWR", true, 60.0, 5.0, 20.0));
    models.push_back(get_power_mosfet_model("PMOS_PWR", false, 60.0, 8.0, 25.0));

    return models;
}

// ============================================================================
// Private helper functions
// ============================================================================

SpiceModelLibrary::DiodeParams SpiceModelLibrary::get_diode_params(const std::string& diode_type) {
    DiodeParams params;

    if (diode_type == "1N4148" || diode_type == "signal") {
        // Small signal switching diode
        params.is = 2.682e-9;
        params.n = 1.836;
        params.rs = 0.5664;
        params.cj0 = 4e-12;
        params.vj = 0.5;
        params.bv = 75.0;
        params.ibv = 1e-4;
        params.tt = 11.54e-9;
    } else if (diode_type == "1N4007" || diode_type == "rectifier") {
        // 1A rectifier diode
        params.is = 7.03e-15;
        params.n = 1.55;
        params.rs = 0.042;
        params.cj0 = 25e-12;
        params.vj = 0.5;
        params.bv = 1000.0;
        params.ibv = 1e-5;
        params.tt = 8.97e-6;
    } else if (diode_type == "1N5819" || diode_type == "schottky") {
        // Schottky barrier diode
        params.is = 2.2e-7;
        params.n = 1.05;
        params.rs = 0.1;
        params.cj0 = 200e-12;
        params.vj = 0.4;
        params.bv = 40.0;
        params.ibv = 1e-3;
        params.tt = 1e-9;
    } else if (diode_type == "1N4733" || diode_type == "zener_5v1") {
        // 5.1V Zener diode
        params.is = 1e-10;
        params.n = 2.0;
        params.rs = 10.0;
        params.cj0 = 100e-12;
        params.vj = 0.75;
        params.bv = 5.1;
        params.ibv = 1e-3;
        params.tt = 100e-9;
    } else {
        // Default diode parameters
        params.is = 1e-14;
        params.n = 1.5;
        params.rs = 1.0;
        params.cj0 = 10e-12;
        params.vj = 0.7;
        params.bv = 100.0;
        params.ibv = 1e-4;
        params.tt = 10e-9;
    }

    return params;
}

SpiceModelLibrary::BJTParams SpiceModelLibrary::get_bjt_params(const std::string& transistor_type) {
    BJTParams params;

    if (transistor_type == "2N2222" || transistor_type == "2N3904") {
        // General purpose NPN
        params.is = 1e-14;
        params.bf = 200.0;
        params.nf = 1.0;
        params.vaf = 100.0;
        params.ikf = 0.1;
        params.ise = 1e-14;
        params.ne = 1.5;
        params.br = 5.0;
        params.nr = 1.0;
        params.var = 100.0;
        params.cje = 20e-12;
        params.vje = 0.75;
        params.mje = 0.33;
        params.cjc = 10e-12;
        params.vjc = 0.75;
        params.mjc = 0.33;
        params.tf = 0.3e-9;
        params.tr = 10e-9;
    } else if (transistor_type == "2N3906") {
        // General purpose PNP
        params.is = 1e-14;
        params.bf = 200.0;
        params.nf = 1.0;
        params.vaf = 100.0;
        params.ikf = 0.1;
        params.ise = 1e-14;
        params.ne = 1.5;
        params.br = 5.0;
        params.nr = 1.0;
        params.var = 100.0;
        params.cje = 20e-12;
        params.vje = 0.75;
        params.mje = 0.33;
        params.cjc = 10e-12;
        params.vjc = 0.75;
        params.mjc = 0.33;
        params.tf = 0.3e-9;
        params.tr = 10e-9;
    } else if (transistor_type == "BC547") {
        // Low noise NPN
        params.is = 1e-14;
        params.bf = 300.0;
        params.nf = 1.0;
        params.vaf = 120.0;
        params.ikf = 0.05;
        params.ise = 5e-15;
        params.ne = 1.4;
        params.br = 4.0;
        params.nr = 1.0;
        params.var = 60.0;
        params.cje = 15e-12;
        params.vje = 0.7;
        params.mje = 0.33;
        params.cjc = 5e-12;
        params.vjc = 0.7;
        params.mjc = 0.33;
        params.tf = 0.2e-9;
        params.tr = 15e-9;
    } else {
        // Default BJT parameters
        params.is = 1e-16;
        params.bf = 100.0;
        params.nf = 1.0;
        params.vaf = 100.0;
        params.ikf = 0.1;
        params.ise = 1e-14;
        params.ne = 1.5;
        params.br = 1.0;
        params.nr = 1.0;
        params.var = 50.0;
        params.cje = 10e-12;
        params.vje = 0.75;
        params.mje = 0.33;
        params.cjc = 5e-12;
        params.vjc = 0.75;
        params.mjc = 0.33;
        params.tf = 0.5e-9;
        params.tr = 20e-9;
    }

    return params;
}

SpiceModelLibrary::MOSFETParams SpiceModelLibrary::get_mosfet_params(const std::string& mosfet_type) {
    MOSFETParams params;

    if (mosfet_type == "IRF540" || mosfet_type == "power_nmos") {
        // N-channel power MOSFET
        params.vto = 4.0;
        params.kp = 20.0;
        params.lambda = 0.01;
        params.rd = 0.05;  // 50 mOhm
        params.rs = 0.02;
        params.cbd = 1e-9;
        params.cbs = 1e-9;
        params.is = 1e-12;
        params.pb = 0.8;
        params.cgso = 1e-9;
        params.cgdo = 1e-9;
    } else if (mosfet_type == "IRF9540" || mosfet_type == "power_pmos") {
        // P-channel power MOSFET
        params.vto = -4.0;
        params.kp = 10.0;
        params.lambda = 0.01;
        params.rd = 0.15;  // 150 mOhm
        params.rs = 0.05;
        params.cbd = 1.5e-9;
        params.cbs = 1.5e-9;
        params.is = 1e-12;
        params.pb = 0.8;
        params.cgso = 1.5e-9;
        params.cgdo = 1.5e-9;
    } else if (mosfet_type == "2N7000" || mosfet_type == "logic_nmos") {
        // Small signal NMOS
        params.vto = 2.0;
        params.kp = 0.1;
        params.lambda = 0.02;
        params.rd = 1.0;
        params.rs = 1.0;
        params.cbd = 50e-12;
        params.cbs = 50e-12;
        params.is = 1e-14;
        params.pb = 0.8;
        params.cgso = 50e-12;
        params.cgdo = 50e-12;
    } else if (mosfet_type == "logic_pmos") {
        params.vto = -2.0;
        params.kp = 0.08;
        params.lambda = 0.02;
        params.rd = 1.5;
        params.rs = 1.5;
        params.cbd = 60e-12;
        params.cbs = 60e-12;
        params.is = 1e-14;
        params.pb = 0.8;
        params.cgso = 60e-12;
        params.cgdo = 60e-12;
    } else {
        // Default MOSFET parameters
        params.vto = (mosfet_type.find("pmos") != std::string::npos) ? -2.0 : 2.0;
        params.kp = 1.0;
        params.lambda = 0.01;
        params.rd = 1.0;
        params.rs = 1.0;
        params.cbd = 100e-12;
        params.cbs = 100e-12;
        params.is = 1e-14;
        params.pb = 0.8;
        params.cgso = 100e-12;
        params.cgdo = 100e-12;
    }

    return params;
}

// ============================================================================
// SpiceParameterMapper
// ============================================================================

std::string SpiceParameterMapper::format_resistance(double ohms) {
    return format_value(ohms, "");
}

std::string SpiceParameterMapper::format_capacitance(double farads) {
    return format_value(farads, "F");
}

std::string SpiceParameterMapper::format_inductance(double henries) {
    return format_value(henries, "H");
}

std::string SpiceParameterMapper::format_voltage(double volts) {
    return format_value(volts, "");
}

std::string SpiceParameterMapper::format_current(double amps) {
    return format_value(amps, "A");
}

std::string SpiceParameterMapper::format_frequency(double hz) {
    return format_value(hz, "Hz");
}

std::string SpiceParameterMapper::format_time(double seconds) {
    return format_value(seconds, "s");
}

std::string SpiceParameterMapper::format_value(double value, const char* units) {
    if (std::abs(value) == 0.0) {
        return "0";
    }

    std::stringstream ss;

    // Handle metric prefixes
    if (std::abs(value) >= 1e12) {
        ss << (value / 1e12) << "T" << units;
    } else if (std::abs(value) >= 1e9) {
        ss << (value / 1e9) << "G" << units;
    } else if (std::abs(value) >= 1e6) {
        ss << (value / 1e6) << "Meg" << units;
    } else if (std::abs(value) >= 1e3) {
        ss << (value / 1e3) << "k" << units;
    } else if (std::abs(value) >= 1.0) {
        ss << value << units;
    } else if (std::abs(value) >= 1e-3) {
        ss << (value * 1e3) << "m" << units;
    } else if (std::abs(value) >= 1e-6) {
        ss << (value * 1e6) << "u" << units;
    } else if (std::abs(value) >= 1e-9) {
        ss << (value * 1e9) << "n" << units;
    } else if (std::abs(value) >= 1e-12) {
        ss << (value * 1e12) << "p" << units;
    } else if (std::abs(value) >= 1e-15) {
        ss << (value * 1e15) << "f" << units;
    } else {
        // Very small values, use scientific notation
        ss << value << units;
    }

    return ss.str();
}

} // namespace mechatron
