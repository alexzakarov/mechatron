#include "NgspiceWrapper.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <mutex>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

// Include ngspice shared header
#ifdef _WIN32
// Path to ngspice shared library - will be set dynamically
#else
#include "ngspice/sharedspice.h"
#endif

namespace mechatron {

namespace fs = std::filesystem;

// ============================================================================
// ngspice shared library function pointers
// ============================================================================

typedef int (*ngSpice_Init_Fn)(
    int(*printfcn)(char*, int, void*),
    int(*statfcn)(char*, int, void*),
    int(*ngexit)(int, bool, bool, int, void*),
    int(*sdata)(pvecvaluesall, int, int, void*),
    int(*sinitdata)(pvecinfoall, int, void*),
    int(*bgtrun)(bool, int, void*),
    void* user_data
);

typedef int (*ngSpice_Command_Fn)(char*);
typedef void* (*ngGet_Vec_Info_Fn)(char*);
typedef char* (*ngSpice_CurPlot_Fn)();
typedef char** (*ngSpice_AllPlots_Fn)();
typedef char** (*ngSpice_AllVecs_Fn)(char*);
typedef void (*ngSpice_Circ_Fn)(char**);
typedef int (*ngSpice_running_Fn)();

struct NgspiceFunctions {
    ngSpice_Init_Fn init = nullptr;
    ngSpice_Command_Fn command = nullptr;
    ngGet_Vec_Info_Fn get_vec_info = nullptr;
    ngSpice_CurPlot_Fn cur_plot = nullptr;
    ngSpice_AllPlots_Fn all_plots = nullptr;
    ngSpice_AllVecs_Fn all_vecs = nullptr;
    ngSpice_Circ_Fn circ = nullptr;
    ngSpice_running_Fn running = nullptr;
};

// Global function pointers and mutex for thread safety
static std::mutex g_ngspice_mutex;
static NgspiceFunctions g_ngspice;
static int g_ngspice_ref_count = 0;

// ============================================================================
// NgspiceWrapper
// ============================================================================

NgspiceWrapper::NgspiceWrapper()
    : m_library_handle(nullptr)
    , m_available(false)
    , m_initialized(false)
    , m_simulation_running(false)
{
    std::lock_guard<std::mutex> lock(g_ngspice_mutex);

    // First user loads the library
    if (g_ngspice_ref_count == 0) {
        m_available = load_library();
        if (m_available) {
            m_available = init_ngspice();
        }
    } else {
        m_available = true;
        m_initialized = true;
    }

    if (m_available) {
        g_ngspice_ref_count++;
    }
}

NgspiceWrapper::~NgspiceWrapper() {
    std::lock_guard<std::mutex> lock(g_ngspice_mutex);

    g_ngspice_ref_count--;

    // Last user unloads the library
    if (g_ngspice_ref_count <= 0) {
        unload_library();
        g_ngspice_ref_count = 0;
    }
}

bool NgspiceWrapper::load_library() {
#ifdef _WIN32
    // Try to load libngspice-0.dll
    std::vector<std::string> dll_paths = {
        m_library_path,
        "C:\\Users\\Muhammed\\Downloads\\ngspice-38_dll_64\\Spice64_dll\\dll-mingw\\libngspice-0.dll",
        "libngspice-0.dll",
        "./libngspice-0.dll",
        "./ngspice.dll"
    };

    for (const auto& path : dll_paths) {
        if (!path.empty()) {
            m_library_handle = LoadLibraryA(path.c_str());
            if (m_library_handle) {
                spdlog::info("Loaded ngspice library from: {}", path);
                break;
            }
        }
    }

    if (!m_library_handle) {
        m_error = "Failed to load libngspice-0.dll. Please install ngspice or set correct library path.";
        spdlog::error(m_error);
        return false;
    }

    // Get function pointers
    g_ngspice.init = (ngSpice_Init_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_Init");
    g_ngspice.command = (ngSpice_Command_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_Command");
    g_ngspice.get_vec_info = (ngGet_Vec_Info_Fn)GetProcAddress((HMODULE)m_library_handle, "ngGet_Vec_Info");
    g_ngspice.cur_plot = (ngSpice_CurPlot_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_CurPlot");
    g_ngspice.all_plots = (ngSpice_AllPlots_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_AllPlots");
    g_ngspice.all_vecs = (ngSpice_AllVecs_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_AllVecs");
    g_ngspice.circ = (ngSpice_Circ_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_Circ");
    g_ngspice.running = (ngSpice_running_Fn)GetProcAddress((HMODULE)m_library_handle, "ngSpice_running");

#else
    // Linux/macOS
    std::vector<std::string> so_paths = {
        m_library_path.empty() ? "libngspice.so.0" : m_library_path,
        "libngspice.so",
        "/usr/lib/libngspice.so.0",
        "/usr/local/lib/libngspice.so.0"
    };

    for (const auto& path : so_paths) {
        m_library_handle = dlopen(path.c_str(), RTLD_LAZY);
        if (m_library_handle) {
            spdlog::info("Loaded ngspice library from: {}", path);
            break;
        }
    }

    if (!m_library_handle) {
        m_error = "Failed to load libngspice: " + std::string(dlerror());
        spdlog::error(m_error);
        return false;
    }

    g_ngspice.init = (ngSpice_Init_Fn)dlsym(m_library_handle, "ngSpice_Init");
    g_ngspice.command = (ngSpice_Command_Fn)dlsym(m_library_handle, "ngSpice_Command");
    g_ngspice.get_vec_info = (ngGet_Vec_Info_Fn)dlsym(m_library_handle, "ngGet_Vec_Info");
    g_ngspice.cur_plot = (ngSpice_CurPlot_Fn)dlsym(m_library_handle, "ngSpice_CurPlot");
    g_ngspice.all_plots = (ngSpice_AllPlots_Fn)dlsym(m_library_handle, "ngSpice_AllPlots");
    g_ngspice.all_vecs = (ngSpice_AllVecs_Fn)dlsym(m_library_handle, "ngSpice_AllVecs");
    g_ngspice.circ = (ngSpice_Circ_Fn)dlsym(m_library_handle, "ngSpice_Circ");
    g_ngspice.running = (ngSpice_running_Fn)dlsym(m_library_handle, "ngSpice_running");
#endif

    // Verify all required functions
    if (!g_ngspice.init || !g_ngspice.command) {
        m_error = "Failed to get ngspice function pointers";
        spdlog::error(m_error);
        unload_library();
        return false;
    }

    return true;
}

void NgspiceWrapper::unload_library() {
    if (m_library_handle) {
#ifdef _WIN32
        FreeLibrary((HMODULE)m_library_handle);
#else
        dlclose(m_library_handle);
#endif
        m_library_handle = nullptr;

        // Reset function pointers
        g_ngspice = {};
    }
}

bool NgspiceWrapper::init_ngspice() {
    if (!g_ngspice.init) {
        return false;
    }

    // Initialize ngspice with callbacks
    int result = g_ngspice.init(
        send_char,
        send_stat,
        controlled_exit,
        send_data,
        send_init_data,
        bg_thread_running,
        this
    );

    if (result != 0) {
        m_error = "Failed to initialize ngspice";
        spdlog::error(m_error);
        return false;
    }

    // Set to SPICE3 compatibility mode
    g_ngspice.command(const_cast<char*>("set ngbehavior=spice3"));

    // Disable output scrolling for easier parsing
    g_ngspice.command(const_cast<char*>("set noaskquit"));
    g_ngspice.command(const_cast<char*>("set nobanner"));

    m_initialized = true;
    spdlog::info("ngspice initialized successfully");
    return true;
}

bool NgspiceWrapper::is_available() {
    return m_available && m_initialized;
}

SimulationResult NgspiceWrapper::simulate(const std::string& netlist,
                                          double duration,
                                          double time_step) {
    if (!m_available || !m_initialized) {
        SimulationResult result;
        result.success = false;
        result.error = "ngspice is not available or not initialized";
        return result;
    }

    // Clear previous simulation data
    m_simulation_data.clear();
    m_node_names.clear();
    m_output_buffer.clear();
    m_simulation_running = true;

    // Split netlist into lines
    std::vector<std::string> lines;
    std::istringstream iss(netlist);
    std::string line;
    while (std::getline(iss, line)) {
        if (!line.empty() && line[0] != '*') {  // Skip comments and empty lines
            lines.push_back(line);
        }
    }

    // Add null terminator for ngSpice_Circ
    lines.push_back("");

    spdlog::info("Simulating circuit with {} lines", lines.size() - 1);

    // Run simulation
    SimulationResult result = run_simulation(lines);

    m_simulation_running = false;
    return result;
}

SimulationResult NgspiceWrapper::simulate_file(const std::string& netlist_path,
                                               double duration,
                                               double time_step) {
    // Read file content
    std::ifstream file(netlist_path);
    if (!file.is_open()) {
        SimulationResult result;
        result.success = false;
        result.error = "Failed to open netlist file: " + netlist_path;
        return result;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return simulate(buffer.str(), duration, time_step);
}

SimulationResult NgspiceWrapper::run_simulation(const std::vector<std::string>& netlist_lines) {
    SimulationResult result;
    result.success = false;

    // Clear previous data
    m_simulation_data.clear();
    m_node_names.clear();
    m_output_buffer.clear();

    // Separate netlist into circuit lines and simulation command
    std::vector<std::string> circuit_lines;
    std::string tran_cmd;

    for (const auto& line : netlist_lines) {
        std::string trimmed = line;
        // Trim whitespace
        size_t start = trimmed.find_first_not_of(" \t\r\n");
        if (start != std::string::npos) {
            trimmed = trimmed.substr(start);
            size_t end = trimmed.find_last_not_of(" \t\r\n");
            if (end != std::string::npos) {
                trimmed = trimmed.substr(0, end + 1);
            }
        }

        if (trimmed.empty() || trimmed[0] == '*') {
            continue;  // Skip empty lines and comments
        }

        if (trimmed.find(".end") == 0 || trimmed.find(".END") == 0) {
            continue;  // Skip .end
        }

        // Extract .tran command, but keep .model and other dot commands
        if (trimmed.find(".tran") == 0 || trimmed.find(".TRAN") == 0) {
            tran_cmd = trimmed;
            continue;  // Don't add .tran to circuit
        }

        circuit_lines.push_back(trimmed);
    }

    // Build full netlist string
    std::string netlist_str;
    for (const auto& line : circuit_lines) {
        if (!line.empty()) {
            netlist_str += line + "\n";
        }
    }
    netlist_str += ".end\n";

    // Write to temporary file
    std::string temp_file = "temp_circuit.cir";
    std::ofstream ofs(temp_file);
    ofs << netlist_str;
    ofs.close();

    spdlog::info("Wrote netlist to {}", temp_file);

    // Load circuit using source command
    std::string source_cmd = "source " + temp_file;
    spdlog::info("Loading circuit: {}", source_cmd);
    g_ngspice.command(const_cast<char*>(source_cmd.c_str()));
    spdlog::info("Circuit loaded, output: {}", m_output_buffer);

    // Run the simulation command
    if (!tran_cmd.empty()) {
        // Remove the leading dot from .tran and use as command
        std::string cmd = tran_cmd;
        if (cmd.find(".tran") == 0) {
            cmd = "tran" + cmd.substr(5);
        } else if (cmd.find(".TRAN") == 0) {
            cmd = "tran" + cmd.substr(5);
        }
        spdlog::info("Running simulation: {}", cmd);
        g_ngspice.command(const_cast<char*>(cmd.c_str()));
    } else {
        // Default transient analysis if no .tran specified
        spdlog::info("No .tran command found, running default");
        g_ngspice.command(const_cast<char*>("tran 10u 10m"));
    }

    // Wait for simulation to complete (with timeout)
    int timeout = 60;  // 60 seconds for complex circuits
    int elapsed = 0;
    int checks_since_last_data = 0;

    while (elapsed < timeout * 10) {
        // Check both ngSpice_running() and our m_simulation_running flag
        int running = 0;
        if (g_ngspice.running) {
            running = g_ngspice.running();
        }

        // Check if simulation is complete (both should indicate not running)
        if (running == 0 && !m_simulation_running) {
            spdlog::info("Simulation completed after {}s", elapsed / 10.0);
            break;
        }

#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
#endif
        elapsed++;
        checks_since_last_data++;

        // Log progress periodically
        if (elapsed % 50 == 0) {
            spdlog::debug("Waiting... running={}, bg_running={}, elapsed={}s",
                          running, m_simulation_running, elapsed / 10);
        }

        // If we've collected data and simulation seems stuck, check if data has stopped arriving
        if (elapsed > 20 && !m_simulation_data.empty()) {
            if (checks_since_last_data > 50) {  // No new data for 5 seconds
                spdlog::info("No new data for 5s, assuming simulation complete");
                break;
            }
        }
    }

    if (elapsed >= timeout * 10) {
        m_error = "Simulation timeout after " + std::to_string(timeout) + " seconds";
        result.error = m_error;
        spdlog::error("{}", m_error);
        spdlog::error("ngspice output: {}", m_output_buffer);
        spdlog::error("Collected {} data points before timeout", m_simulation_data.size());
        // Don't return immediately - try to use what we have
    }

    // Give ngspice time to finalize
#ifdef _WIN32
    Sleep(200);
#else
    usleep(200000);
#endif

    // If we have data from callbacks, use it. Otherwise try ngGet_Vec_Info.
    if (!m_simulation_data.empty()) {
        result.time_points = m_simulation_data;
        result.nodes = m_node_names;
        result.success = true;
        spdlog::info("Using {} time points collected from callbacks", m_simulation_data.size());
    } else {
        // Fallback to collecting data via ngGet_Vec_Info
        collect_simulation_data();
        if (!m_simulation_data.empty()) {
            result.time_points = m_simulation_data;
            result.nodes = m_node_names;
            result.success = true;
        } else {
            result.error = "No simulation data collected. Output: " + m_output_buffer;
            spdlog::warn("No data collected");
        }
    }

    return result;
}

void NgspiceWrapper::collect_simulation_data() {
    if (!g_ngspice.all_vecs) {
        spdlog::warn("all_vecs function not available");
        return;
    }

    // Get current plot name
    char* cur_plot = nullptr;
    if (g_ngspice.cur_plot) {
        cur_plot = g_ngspice.cur_plot();
    }

    if (!cur_plot) {
        spdlog::warn("No current plot");
        return;
    }

    spdlog::debug("Current plot: {}", cur_plot);

    // Get all vectors in current plot
    char** all_vecs = g_ngspice.all_vecs(cur_plot);
    if (!all_vecs) {
        spdlog::warn("No vectors in plot");
        return;
    }

    // Collect vector names
    std::vector<std::string> vec_names;
    for (int i = 0; all_vecs[i] != nullptr; i++) {
        vec_names.push_back(all_vecs[i]);
        spdlog::debug("Vector: {}", all_vecs[i]);
    }

    if (vec_names.empty()) {
        spdlog::warn("No vector names found");
        return;
    }

    // Find time vector (usually "time")
    std::string time_name = "time";
    pvector_info time_vec = nullptr;

    for (const auto& name : vec_names) {
        if (name == "time") {
            time_vec = (pvector_info)g_ngspice.get_vec_info(const_cast<char*>(name.c_str()));
            break;
        }
    }

    if (!time_vec) {
        spdlog::warn("Time vector not found");
        return;
    }

    // Collect data points
    m_node_names = vec_names;
    int num_points = time_vec->v_length;

    spdlog::debug("Time vector has {} points", num_points);

    for (int i = 0; i < num_points; i++) {
        double t = time_vec->v_realdata[i];
        std::vector<NodeVoltage> voltages;

        // Get all node voltages at this time point
        for (const auto& name : vec_names) {
            if (name == "time") continue;  // Skip time vector itself

            pvector_info vec = (pvector_info)g_ngspice.get_vec_info(const_cast<char*>(name.c_str()));
            if (vec && vec->v_realdata && i < vec->v_length) {
                NodeVoltage nv;
                nv.node = name;
                nv.voltage = vec->v_realdata[i];
                voltages.push_back(nv);
            }
        }

        m_simulation_data.push_back({t, voltages});
    }

    spdlog::info("Collected {} time points with {} vectors each",
                 m_simulation_data.size(), m_node_names.size());
}

std::string NgspiceWrapper::generate_netlist(
    const std::vector<std::tuple<std::string, std::string, std::unordered_map<std::string, double>>>& components,
    const std::vector<std::tuple<std::string, std::string, std::string>>& connections
) {
    NetlistBuilder builder;
    builder.set_title("Generated Netlist");

    // Simple netlist generation - extend as needed
    for (const auto& [name, type, params] : components) {
        if (type == "resistor" || type == "R") {
            double r = params.count("resistance") ? params.at("resistance") : 1000.0;
            builder.add_resistor(name, "1", "2", r);
        } else if (type == "capacitor" || type == "C") {
            double c = params.count("capacitance") ? params.at("capacitance") : 1e-6;
            builder.add_capacitor(name, "1", "2", c);
        } else if (type == "voltage_source" || type == "V") {
            double v = params.count("voltage") ? params.at("voltage") : 5.0;
            builder.add_dc_voltage(name, "1", "0", v);
        }
    }

    builder.add_simulation("tran", "10m " + std::to_string(1.0));  // 1 second transient

    return builder.build();
}

// ============================================================================
// Callback functions for ngspice
// ============================================================================

int NgspiceWrapper::send_char(char* output, int id, void* user_data) {
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper) {
        wrapper->m_output_buffer += output;
        spdlog::debug("[ngspice] {}", output);
    }
    return 0;
}

int NgspiceWrapper::send_stat(char* status, int id, void* user_data) {
    spdlog::debug("[ngspice status] {}", status);
    return 0;
}

int NgspiceWrapper::controlled_exit(int exit_status, bool immediate, bool quit, int id, void* user_data) {
    spdlog::info("[ngspice] Exit requested: status={}, immediate={}, quit={}", exit_status, immediate, quit);
    return 0;
}

int NgspiceWrapper::send_data(pvecvaluesall data, int num_vecs, int id, void* user_data) {
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (!wrapper || !data) {
        return 0;
    }

    // data is vecvaluesall*, containing actual simulation data points
    // vecsa is an array of vecvalues*, one per vector
    // Each vecvalues contains: name, creal, cimag, is_scale, is_complex
    spdlog::debug("send_data called: veccount={}, vecindex={}", data->veccount, data->vecindex);

    // Find time vector and extract time point
    double time_val = 0.0;
    std::vector<NodeVoltage> voltages;

    for (int i = 0; i < data->veccount; i++) {
        pvecvalues vv = data->vecsa[i];
        if (!vv) continue;

        std::string vec_name = vv->name;
        double value = vv->creal;

        if (vv->is_scale) {
            // This is the time/scale vector
            time_val = value;
        } else {
            // This is a voltage/current value
            // Remove 'v(' prefix and ')' suffix from node names if present
            std::string node_name = vec_name;
            size_t pos = node_name.find("v(");
            if (pos == 0) {
                node_name = node_name.substr(2);
                pos = node_name.find(")");
                if (pos != std::string::npos) {
                    node_name = node_name.substr(0, pos);
                }
            }

            NodeVoltage nv;
            nv.node = node_name;
            nv.voltage = value;
            voltages.push_back(nv);
        }

        // Collect node names on first callback
        if (data->vecindex == 0 && !vec_name.empty()) {
            if (std::find(wrapper->m_node_names.begin(), wrapper->m_node_names.end(), vec_name) == wrapper->m_node_names.end()) {
                wrapper->m_node_names.push_back(vec_name);
            }
        }
    }

    // Add this time point to simulation data
    if (!voltages.empty() || data->veccount <= 1) {
        wrapper->m_simulation_data.push_back({time_val, voltages});
    }

    spdlog::debug("Collected time point {}: {} voltages", time_val, voltages.size());

    return 0;
}

int NgspiceWrapper::send_init_data(pvecinfoall vec_info, int id, void* user_data) {
    spdlog::debug("[ngspice] Init data received: plot={}, veccount={}",
                  vec_info ? vec_info->name : "null", vec_info ? vec_info->veccount : 0);

    // Store vector info for later data collection
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper && vec_info) {
        spdlog::debug("Vectors in plot:");
        for (int i = 0; i < vec_info->veccount; i++) {
            if (vec_info->vecs && vec_info->vecs[i]) {
                spdlog::debug("  {}: {}", i, vec_info->vecs[i]->vecname);
            }
        }
    }
    return 0;
}

int NgspiceWrapper::bg_thread_running(bool is_running, int id, void* user_data) {
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper) {
        wrapper->m_simulation_running = is_running;
        spdlog::debug("[ngspice] Background thread running: {}", is_running);
    }
    return 0;
}

std::string NgspiceWrapper::get_error_message() {
    return m_output_buffer;
}

// ============================================================================
// NetlistBuilder
// ============================================================================

NetlistBuilder::NetlistBuilder()
    : m_line_number(1)
{
    set_title("Circuit Simulation");
}

void NetlistBuilder::set_title(const std::string& title) {
    m_title = title;
}

void NetlistBuilder::add_resistor(const std::string& name,
                                  const std::string& node1,
                                  const std::string& node2,
                                  double resistance) {
    m_components.push_back(name + " " + node1 + " " + node2 + " " +
                          std::to_string(resistance));
}

void NetlistBuilder::add_capacitor(const std::string& name,
                                   const std::string& node1,
                                   const std::string& node2,
                                   double capacitance) {
    m_components.push_back(name + " " + node1 + " " + node2 + " " +
                          std::to_string(capacitance));
}

void NetlistBuilder::add_inductor(const std::string& name,
                                  const std::string& node1,
                                  const std::string& node2,
                                  double inductance) {
    m_components.push_back(name + " " + node1 + " " + node2 + " " +
                          std::to_string(inductance));
}

void NetlistBuilder::add_voltage_source(const std::string& name,
                                       const std::string& node_pos,
                                       const std::string& node_neg,
                                       double voltage) {
    add_dc_voltage(name, node_pos, node_neg, voltage);
}

void NetlistBuilder::add_dc_voltage(const std::string& name,
                                   const std::string& node_pos,
                                   const std::string& node_neg,
                                   double voltage) {
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " +
                          std::to_string(voltage));
}

void NetlistBuilder::add_pulse_voltage(const std::string& name,
                                      const std::string& node_pos,
                                      const std::string& node_neg,
                                      double v_low, double v_high,
                                      double delay, double rise_time,
                                      double fall_time, double pulse_width,
                                      double period) {
    std::stringstream ss;
    ss << name << " " << node_pos << " " << node_neg << " DC 0 PULSE("
       << v_low << " " << v_high << " "
       << delay << " " << rise_time << " " << fall_time << " "
       << pulse_width << " " << period << ")";
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_diode(const std::string& name,
                              const std::string& node_pos,
                              const std::string& node_neg,
                              const std::string& model) {
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " + model);
}

void NetlistBuilder::add_led(const std::string& name,
                            const std::string& node_pos,
                            const std::string& node_neg,
                            double forward_voltage) {
    // Add LED model
    m_models.push_back(".model " + name + "_model D(Is=1e-14 Rs=1 N=1.8 Vj=" +
                      std::to_string(forward_voltage) + ")");
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " +
                          name + "_model");
}

void NetlistBuilder::add_bjt(const std::string& name,
                            const std::string& node_collector,
                            const std::string& node_base,
                            const std::string& node_emitter,
                            const std::string& model) {
    m_components.push_back(name + " " + node_collector + " " + node_base + " " +
                          node_emitter + " " + model);
}

void NetlistBuilder::add_simulation(const std::string& type,
                                   const std::string& parameters) {
    m_commands.push_back("." + type + " " + parameters);
}

void NetlistBuilder::add_plot(const std::vector<std::string>& nodes) {
    std::string cmd = ".plot";
    for (const auto& node : nodes) {
        cmd += " " + node;
    }
    m_commands.push_back(cmd);
}

std::string NetlistBuilder::build() const {
    std::stringstream ss;

    // Title
    ss << "* " << m_title << "\n";

    // Models
    for (const auto& model : m_models) {
        ss << model << "\n";
    }

    // Components
    for (const auto& comp : m_components) {
        ss << comp << "\n";
    }

    // Commands
    for (const auto& cmd : m_commands) {
        ss << cmd << "\n";
    }

    // End
    ss << ".end\n";

    return ss.str();
}

void NetlistBuilder::clear() {
    m_title.clear();
    m_components.clear();
    m_models.clear();
    m_commands.clear();
    m_line_number = 1;
}

} // namespace mechatron
