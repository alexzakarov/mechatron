#include "NgspiceWrapper.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <mutex>
#include <cstring>
#include <algorithm>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>  // For usleep
#endif

// Include ngspice shared header if available
#if MECHATRON_HAVE_NGSPICE
  #include "ngspice/sharedspice.h"
#else
  // When ngspice is not available, stub implementations will be used
  #pragma message("Ngspice support disabled - building without ngspice integration")
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
#if MECHATRON_HAVE_NGSPICE
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
#else
    m_available = false;
    m_initialized = false;
    m_error = "ngspice support was not enabled during build. Recompile with MECHATRON_HAVE_NGSPICE=1 to enable ngspice integration.";
    spdlog::warn(m_error);
#endif
}

NgspiceWrapper::~NgspiceWrapper() {
#if MECHATRON_HAVE_NGSPICE
    std::lock_guard<std::mutex> lock(g_ngspice_mutex);

    g_ngspice_ref_count--;

    // Last user unloads the library
    if (g_ngspice_ref_count <= 0) {
        unload_library();
        g_ngspice_ref_count = 0;
    }
#endif
}

bool NgspiceWrapper::load_library() {
#if MECHATRON_HAVE_NGSPICE
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
                spdlog::debug("Loaded ngspice library from: {}", path);
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
#ifdef __APPLE__
        , "/opt/homebrew/lib/libngspice.dylib"
        , "/usr/local/lib/libngspice.dylib"
        , "libngspice.dylib"
#endif
    };

    for (const auto& path : so_paths) {
        m_library_handle = dlopen(path.c_str(), RTLD_LAZY);
        if (m_library_handle) {
            spdlog::debug("Loaded ngspice library from: {}", path);
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
    if (!g_ngspice.init || !g_ngspice.command || !g_ngspice.circ) {
        m_error = "Failed to get ngspice function pointers";
        spdlog::error(m_error);
        unload_library();
        return false;
    }

    return true;
#else
    m_error = "ngspice support not enabled during build";
    return false;
#endif
}

void NgspiceWrapper::unload_library() {
#if MECHATRON_HAVE_NGSPICE
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
#endif
}

bool NgspiceWrapper::init_ngspice() {
#if MECHATRON_HAVE_NGSPICE
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
    spdlog::debug("ngspice initialized successfully");
    return true;
#else
    return false;
#endif
}

bool NgspiceWrapper::is_available() {
    return m_available && m_initialized;
}

SimulationResult NgspiceWrapper::simulate(const std::string& netlist,
                                          double duration,
                                          double time_step) {
#if MECHATRON_HAVE_NGSPICE
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

    spdlog::trace("Simulating circuit with {} lines", lines.size() - 1);

    // Run simulation
    SimulationResult result = run_simulation(lines);

    m_simulation_running = false;
    return result;
#else
    SimulationResult result;
    result.success = false;
    result.error = "ngspice support not enabled during build. Recompile with MECHATRON_HAVE_NGSPICE=1.";
    spdlog::error(result.error);
    return result;
#endif
}

SimulationResult NgspiceWrapper::simulate_file(const std::string& netlist_path,
                                               double duration,
                                               double time_step) {
#if MECHATRON_HAVE_NGSPICE
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
#else
    SimulationResult result;
    result.success = false;
    result.error = "ngspice support not enabled during build";
    spdlog::error(result.error);
    return result;
#endif
}

SimulationResult NgspiceWrapper::run_simulation(const std::vector<std::string>& netlist_lines) {
#if MECHATRON_HAVE_NGSPICE
    SimulationResult result;
    result.success = false;

    // Clear previous data
    m_simulation_data.clear();
    m_node_names.clear();
    m_output_buffer.clear();

    // Separate netlist into circuit lines and simulation command
    std::vector<std::string> circuit_lines;
    std::string analysis_cmd;

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
            // Only skip if it's ".end" (not ".ends")
            size_t len = trimmed.length();
            if (len == 4 ||  // Exactly ".end"
                (len >= 5 && (trimmed[4] == ' ' || trimmed[4] == '\t' || trimmed[4] == '\0'))) {  // ".end " with space/param
                continue;  // Skip .end (but not .ends)
            }
        }

        // Extract analysis commands, but keep .model/.subckt and other dot commands.
        if (trimmed.find(".tran") == 0 || trimmed.find(".TRAN") == 0 ||
            trimmed.find(".op") == 0 || trimmed.find(".OP") == 0 ||
            trimmed.find(".dc") == 0 || trimmed.find(".DC") == 0 ||
            trimmed.find(".ac") == 0 || trimmed.find(".AC") == 0) {
            analysis_cmd = trimmed;
            continue;  // Don't add .tran to circuit
        }

        circuit_lines.push_back(trimmed);
    }

    // Prepare ngSpice_Circ input (circuit only; analysis is issued via ngSpice_Command).
    std::vector<std::string> circ_lines = circuit_lines;
    circ_lines.push_back(".end");
    circ_lines.push_back("");

    // Hash circuit portion to avoid re-loading it when unchanged.
    std::string hash_basis;
    hash_basis.reserve(4096);
    for (const auto& l : circ_lines) {
        hash_basis.append(l);
        hash_basis.push_back('\n');
    }
    const size_t net_hash = std::hash<std::string>{}(hash_basis);

    const bool need_reload = (!m_has_loaded_netlist) || (net_hash != m_loaded_netlist_hash);
    if (need_reload) {
        g_ngspice.command(const_cast<char*>("reset"));

        std::vector<char*> cstrs;
        cstrs.reserve(circ_lines.size() + 1);
        for (auto& s : circ_lines) {
            cstrs.push_back(const_cast<char*>(s.c_str()));
        }
        cstrs.push_back(nullptr);

        if (spdlog::should_log(spdlog::level::trace)) {
            spdlog::trace("[NGSPICE] Loading circuit ({} lines)", circ_lines.size() - 1);
        }
        g_ngspice.circ(cstrs.data());

        m_loaded_netlist_hash = net_hash;
        m_has_loaded_netlist = true;
    }

    // If ngspice reported a load error through stdout, abort early.
    if (m_output_buffer.find("Error") != std::string::npos ||
        m_output_buffer.find("ERROR") != std::string::npos) {
        spdlog::error("[Ngspice] Failed to load circuit: {}", m_output_buffer);
        m_simulation_running = false;
        result.success = false;
        result.error = "Failed to load circuit: " + m_output_buffer;
        return result;
    }

    // Run the simulation command
    if (!analysis_cmd.empty()) {
        // Remove the leading dot and use as an interactive ngspice command.
        std::string cmd = analysis_cmd.substr(1);
        spdlog::debug("Running simulation: {}", cmd);
        g_ngspice.command(const_cast<char*>(cmd.c_str()));
    } else {
        // Require explicit analysis specification
        spdlog::error("[Ngspice] No analysis command (.tran, .ac, .dc, etc.) found in netlist");
        spdlog::error("[Ngspice] Please specify the analysis type explicitly (e.g., '.tran 10u 10m' for transient analysis)");
        m_simulation_running = false;
        result.success = false;
        result.error = "No analysis command found in netlist. Please specify .tran, .ac, .dc, or other analysis type.";
        return result;
    }

    // Wait for completion only when ngspice reports background running.
    // .op completes synchronously; avoid polling sleeps for it.
    const bool is_op = (analysis_cmd.rfind(".op", 0) == 0) || (analysis_cmd.rfind(".OP", 0) == 0);
    if (!is_op && g_ngspice.running && g_ngspice.running() != 0) {
        const int timeout_s = 10;
        int spins = 0;
        while (g_ngspice.running() != 0 && spins < timeout_s * 10) {
#ifdef _WIN32
            Sleep(100);
#else
            usleep(100000);
#endif
            spins++;
        }
        if (spins >= timeout_s * 10) {
            spdlog::warn("[Ngspice] Still running after {}s, continuing with collected data", timeout_s);
        }
    }

    // Give ngspice time to finalize only when callbacks did not provide data.
    if (m_simulation_data.empty()) {
#ifdef _WIN32
        Sleep(200);
#else
        usleep(200000);
#endif
    }

    // If we have data from callbacks, use it. Otherwise try ngGet_Vec_Info.
    if (!m_simulation_data.empty()) {
        result.time_points = m_simulation_data;
        result.nodes = m_node_names;
        result.success = true;

        if (spdlog::should_log(spdlog::level::trace)) {
            spdlog::trace("[NGSPICE] Returning {} time points (callbacks)", m_simulation_data.size());
        }
    } else {
        // No data collected - check for simulation errors
        if (m_output_buffer.find("singular matrix") != std::string::npos ||
            m_output_buffer.find("Error") != std::string::npos ||
            m_output_buffer.find("failed") != std::string::npos) {
            result.success = false;
            result.error = "Simulation failed: " + m_output_buffer;
            spdlog::warn("[NGSPICE] Simulation failed - circuit may be incomplete or invalid");
        } else {
            // Fallback to collecting data via ngGet_Vec_Info
            spdlog::warn("[Ngspice] No data from callbacks, attempting fallback data collection");
            collect_simulation_data();
            if (!m_simulation_data.empty()) {
                result.time_points = m_simulation_data;
                result.nodes = m_node_names;
                result.success = true;
                spdlog::debug("Fallback collection successful: {} time points", m_simulation_data.size());
            } else {
                result.error = "No simulation data collected. Output: " + m_output_buffer;
                spdlog::error("[Ngspice] Data collection failed - no simulation data available");
                spdlog::error("[Ngspice] Possible issues: simulation didn't complete, netlist errors, or vector name mismatch");
            }
        }
    }

    return result;
#else
    SimulationResult result;
    result.success = false;
    result.error = "ngspice support not enabled during build";
    return result;
#endif
}

void NgspiceWrapper::collect_simulation_data() {
#if MECHATRON_HAVE_NGSPICE
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

    spdlog::trace("Current plot: {}", cur_plot);

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
        spdlog::trace("Vector: {}", all_vecs[i]);
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

    spdlog::trace("Time vector has {} points", num_points);

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

    spdlog::trace("Collected {} time points with {} vectors each",
                  m_simulation_data.size(), m_node_names.size());
#endif
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
        spdlog::trace("[ngspice] {}", output);
    }
    return 0;
}

int NgspiceWrapper::send_stat(char* status, int id, void* user_data) {
    spdlog::trace("[ngspice status] {}", status);
    return 0;
}

int NgspiceWrapper::controlled_exit(int exit_status, bool immediate, bool quit, int id, void* user_data) {
    spdlog::debug("[ngspice] Exit requested: status={}, immediate={}, quit={}", exit_status, immediate, quit);
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper) {
        spdlog::debug("[ngspice] Setting m_simulation_running = false");
        wrapper->m_simulation_running = false;
    }
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
    // spdlog::debug("send_data called: veccount={}, vecindex={}", data->veccount, data->vecindex);  // Disabled for performance

    // Find time vector and extract time point
    double time_val = 0.0;
    std::vector<NodeVoltage> voltages;

    for (int i = 0; i < data->veccount; i++) {
        pvecvalues vv = data->vecsa[i];
        if (!vv) continue;

        std::string vec_name = vv->name;
        double value = vv->creal;

        if (vv->is_scale && vec_name == "time") {
            // This is the transient time vector. Operating-point analyses may
            // mark a voltage vector as scale, so only the actual time vector
            // should be excluded from node/current collection.
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

    // spdlog::debug("Collected time point {}: {} voltages", time_val, voltages.size());  // Disabled for performance

    return 0;
}

int NgspiceWrapper::send_init_data(pvecinfoall vec_info, int id, void* user_data) {
    spdlog::trace("[ngspice] Init data received: plot={}, veccount={}",
                  vec_info ? vec_info->name : "null", vec_info ? vec_info->veccount : 0);

    // Store vector info for later data collection
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper && vec_info) {
        spdlog::trace("Vectors in plot:");
        for (int i = 0; i < vec_info->veccount; i++) {
            if (vec_info->vecs && vec_info->vecs[i]) {
                spdlog::trace("  {}: {}", i, vec_info->vecs[i]->vecname);
            }
        }
    }
    return 0;
}

int NgspiceWrapper::bg_thread_running(bool is_running, int id, void* user_data) {
    NgspiceWrapper* wrapper = static_cast<NgspiceWrapper*>(user_data);
    if (wrapper) {
        wrapper->m_simulation_running = is_running;
        spdlog::trace("[ngspice] Background thread running: {}", is_running);
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

void NetlistBuilder::add_mosfet(const std::string& name,
                               const std::string& node_drain,
                               const std::string& node_gate,
                               const std::string& node_source,
                               const std::string& node_bulk,
                               const std::string& model,
                               double length,
                               double width) {
    // MOSFET: M<name> <nd> <ng> <ns> <nb> <model> L=<length> W=<width>
    std::stringstream ss;
    ss << name << " " << node_drain << " " << node_gate << " " << node_source << " "
       << node_bulk << " " << model << " L=" << length << " W=" << width;
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_jfet(const std::string& name,
                             const std::string& node_drain,
                             const std::string& node_gate,
                             const std::string& node_source,
                             const std::string& model) {
    m_components.push_back(name + " " + node_drain + " " + node_gate + " " +
                          node_source + " " + model);
}

void NetlistBuilder::add_opamp(const std::string& name,
                              const std::string& node_non_inv,
                              const std::string& node_inv,
                              const std::string& node_vcc,
                              const std::string& node_vee,
                              const std::string& node_out,
                              const std::string& model) {
    // Op-amps are implemented as subcircuits
    // Format: X<name> <nodes> <subcircuit_name>
    std::string nodes = node_non_inv + " " + node_inv + " " + node_vcc + " " +
                       node_vee + " " + node_out;
    add_subcircuit(name, model, {node_non_inv, node_inv, node_vcc, node_vee, node_out});
}

void NetlistBuilder::add_dc_current(const std::string& name,
                                   const std::string& node_pos,
                                   const std::string& node_neg,
                                   double current) {
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " +
                          std::to_string(current));
}

void NetlistBuilder::add_sine_voltage(const std::string& name,
                                     const std::string& node_pos,
                                     const std::string& node_neg,
                                     double offset,
                                     double amplitude,
                                     double frequency) {
    // SIN(<vo> <va> <freq>)
    std::stringstream ss;
    ss << name << " " << node_pos << " " << node_neg << " DC 0 SIN("
       << offset << " " << amplitude << " " << frequency << ")";
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_pulse_current(const std::string& name,
                                      const std::string& node_pos,
                                      const std::string& node_neg,
                                      double i_low, double i_high,
                                      double delay, double rise_time, double fall_time,
                                      double pulse_width, double period) {
    std::stringstream ss;
    ss << name << " " << node_pos << " " << node_neg << " DC 0 PULSE("
       << i_low << " " << i_high << " "
       << delay << " " << rise_time << " " << fall_time << " "
       << pulse_width << " " << period << ")";
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_transformer(const std::string& name,
                                    const std::string& inductor1,
                                    const std::string& inductor2,
                                    double coupling_coefficient) {
    // Coupling: K<name> <L1> <L2> <k>
    std::string k_name = "K_" + name;
    m_components.push_back(k_name + " " + inductor1 + " " + inductor2 + " " +
                          std::to_string(coupling_coefficient));
}

void NetlistBuilder::add_vswitch(const std::string& name,
                                const std::string& node_pos,
                                const std::string& node_neg,
                                const std::string& node_ctrl_pos,
                                const std::string& node_ctrl_neg,
                                const std::string& model) {
    // Add switch model if not already present
    bool model_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".model " + model) == 0) {
            model_exists = true;
            break;
        }
    }
    if (!model_exists) {
        m_models.push_back(".model " + model + " SW(Ron=0.01 Roff=1e9 Vt=0.5 Vh=0)");
    }
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " +
                          node_ctrl_pos + " " + node_ctrl_neg + " " + model);
}

void NetlistBuilder::add_cswitch(const std::string& name,
                                const std::string& node_pos,
                                const std::string& node_neg,
                                const std::string& controlling_source,
                                const std::string& model) {
    // Add switch model if not already present
    bool model_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".model " + model) == 0) {
            model_exists = true;
            break;
        }
    }
    if (!model_exists) {
        m_models.push_back(".model " + model + " CSW(Ron=0.01 Roff=1e9 It=0.1 Ih=0)");
    }
    m_components.push_back(name + " " + node_pos + " " + node_neg + " " +
                          controlling_source + " " + model);
}

void NetlistBuilder::add_resistor_tc(const std::string& name,
                                    const std::string& node1,
                                    const std::string& node2,
                                    double resistance,
                                    double temp_coeff,
                                    double nominal_temp) {
    // R<name> <n1> <n2> <value> TC=<tc1> TC=<tc2>
    std::stringstream ss;
    ss << name << " " << node1 << " " << node2 << " " << resistance;
    if (temp_coeff != 0.0) {
        ss << " TC=" << temp_coeff << ",0";
    }
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_capacitor_ic(const std::string& name,
                                     const std::string& node1,
                                     const std::string& node2,
                                     double capacitance,
                                     double initial_voltage) {
    // C<name> <n1> <n2> <value> IC=<initial_voltage>
    std::stringstream ss;
    ss << name << " " << node1 << " " << node2 << " " << capacitance;
    if (initial_voltage != 0.0) {
        ss << " IC=" << initial_voltage;
    }
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_inductor_ic(const std::string& name,
                                    const std::string& node1,
                                    const std::string& node2,
                                    double inductance,
                                    double initial_current) {
    // L<name> <n1> <n2> <value> IC=<initial_current>
    std::stringstream ss;
    ss << name << " " << node1 << " " << node2 << " " << inductance;
    if (initial_current != 0.0) {
        ss << " IC=" << initial_current;
    }
    m_components.push_back(ss.str());
}

void NetlistBuilder::add_subcircuit(const std::string& name,
                                   const std::string& subcircuit_name,
                                   const std::vector<std::string>& nodes,
                                   const std::vector<std::pair<std::string, double>>& parameters) {
    // X<name> <node1> <node2> ... <subcircuit_name> [params]
    std::stringstream ss;
    ss << "X" << name << " ";  // CRITICAL: Add 'X' prefix for subcircuit calls!
    for (const auto& node : nodes) {
        ss << node << " ";
    }
    ss << subcircuit_name;

    // Add optional parameters
    for (const auto& param : parameters) {
        ss << " " << param.first << "=" << param.second;
    }

    m_components.push_back(ss.str());
}

void NetlistBuilder::add_motor_driver(const std::string& name,
                                     const std::string& node_vcc,
                                     const std::string& node_gnd,
                                     const std::string& node_in1,
                                     const std::string& node_in2,
                                     const std::string& node_in3,
                                     const std::string& node_in4,
                                     const std::string& node_out1,
                                     const std::string& node_out2,
                                     const std::string& mosfet_model) {
    std::string subckt_name = "MOTOR_DRIVER_PWM_DIR";
    bool subckt_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".subckt " + subckt_name) == 0) {
            subckt_exists = true;
            break;
        }
    }

    if (!subckt_exists) {
        std::stringstream subckt;
        subckt << ".subckt " << subckt_name << " VCC GND PWM DIR EN UNUSED OUT1 OUT2\n";
        subckt << "R_PWM PWM GND 1e9\n";
        subckt << "R_DIR DIR GND 1e9\n";
        subckt << "R_EN EN GND 1e9\n";
        subckt << "R_UNUSED UNUSED GND 1e9\n";
        subckt << "R_LOAD1 OUT1 GND 1e6\n";
        subckt << "R_LOAD2 OUT2 GND 1e6\n";
        subckt << "B_OUT1 OUT1 GND V=V(VCC,GND)/(1+exp(-8*(V(EN,GND)-2.5)))/(1+exp(-8*(V(PWM,GND)-2.5)))*(1-1/(1+exp(-8*(V(DIR,GND)-2.5))))\n";
        subckt << "B_OUT2 OUT2 GND V=V(VCC,GND)/(1+exp(-8*(V(EN,GND)-2.5)))/(1+exp(-8*(V(PWM,GND)-2.5)))/(1+exp(-8*(V(DIR,GND)-2.5)))\n";
        subckt << ".ends\n";
        m_models.push_back(subckt.str());
    }

    // Add subcircuit instance
    add_subcircuit(name, subckt_name,
                   {node_vcc, node_gnd, node_in1, node_in2, node_in3, node_in4, node_out1, node_out2});
}

void NetlistBuilder::add_h_bridge(const std::string& name,
                                  const std::string& node_vcc,
                                  const std::string& node_gnd,
                                  const std::string& node_in1,
                                  const std::string& node_in2,
                                  const std::string& node_en,
                                  const std::string& node_out1,
                                  const std::string& node_out2) {
    std::string subckt_name = "HBRIDGE_LOGIC";
    bool subckt_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".subckt " + subckt_name) == 0) {
            subckt_exists = true;
            break;
        }
    }

    if (!subckt_exists) {
        std::stringstream subckt;
        subckt << ".subckt " << subckt_name << " VCC GND IN1 IN2 EN OUT1 OUT2\n";
        subckt << "R_IN1 IN1 GND 1e9\n";
        subckt << "R_IN2 IN2 GND 1e9\n";
        subckt << "R_EN EN GND 1e9\n";
        subckt << "R_LOAD1 OUT1 GND 1e6\n";
        subckt << "R_LOAD2 OUT2 GND 1e6\n";
        subckt << "B_OUT1 OUT1 GND V=V(VCC,GND)/(1+exp(-8*(V(EN,GND)-2.5)))/(1+exp(-8*(V(IN1,GND)-2.5)))\n";
        subckt << "B_OUT2 OUT2 GND V=V(VCC,GND)/(1+exp(-8*(V(EN,GND)-2.5)))/(1+exp(-8*(V(IN2,GND)-2.5)))\n";
        subckt << ".ends\n";
        m_models.push_back(subckt.str());
    }

    add_subcircuit(name, subckt_name,
                   {node_vcc, node_gnd, node_in1, node_in2, node_en, node_out1, node_out2});
}

void NetlistBuilder::add_buck_converter(const std::string& name,
                                      const std::string& node_vin,
                                      const std::string& node_gnd,
                                      const std::string& node_vout,
                                      double inductance,
                                      double capacitance,
                                      double switching_freq,
                                      const std::string& mosfet_model,
                                      double duty_cycle) {
    // For buck converter, use PMOS for high-side switching
    // If default NMOS_PWR is specified, use PMOS_PWR instead
    std::string actual_model = (mosfet_model == "NMOS_PWR") ? "PMOS_PWR" : mosfet_model;

    // Generate buck converter subcircuit
    // Use a single shared subcircuit for all buck converters
    std::string subckt_name = "BUCK_CONV";

    // Check if subcircuit already exists
    bool subckt_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".subckt " + subckt_name) == 0) {
            subckt_exists = true;
            break;
        }
    }

    if (!subckt_exists) {
        std::stringstream subckt;

        // Calculate PWM pulse parameters from switching frequency
        double period = 1.0 / switching_freq;  // Period in seconds
        duty_cycle = std::clamp(duty_cycle, 0.0, 0.95);
        double pulse_width = period * duty_cycle;  // ON time

        // Ensure minimum pulse width (at least 1ns)
        if (pulse_width < 1e-9) pulse_width = 1e-9;

        // For a real buck converter: Vout = Vin * Duty_Cycle
        // With 50% duty: Vout = 0.5 * Vin
        //
        // Standard Buck Topology:
        // VIN ──[PMOS high-side switch]──SW───L_FILTER───VOUT
        //         │                        │
        //        PWM                     C_FILTER
        //         │                        │
        //        GND                       │
        //                                R_LOAD
        //                                 │
        //                                GND
        //
        // When PWM=0V: PMOS ON, SW connects to VIN, inductor charges
        // When PWM=5V: PMOS OFF, inductor discharges through diode

        subckt << ".subckt " << subckt_name << " VIN GND VOUT params: DUTY=0.5 CVAL=100u ROUT=0.5\n";

        // Simplified regulated buck output with finite output impedance. It is
        // low enough to regulate under normal loads while still avoiding the
        // unrealistic near-ideal source behavior that produced huge fault
        // currents when VOUT was externally back-driven.
        subckt << "V_IN_SENSE VIN VIN_INT 0\n";
        subckt << "R_SENSE VIN_INT SENSE 1e9\n";
        subckt << "E_BUCK VINT GND SENSE GND {DUTY}\n";
        subckt << "R_OUT VINT VOUT {ROUT}\n";
        subckt << "B_INPUT VIN_INT GND I={max(0, -DUTY*I(E_BUCK))}\n";

        // Output filter capacitor
        subckt << "C_FILTER VOUT GND {CVAL}\n";

        // Load resistor (for simulation stability)
        subckt << "R_LOAD VOUT GND 100k\n";

        subckt << ".ends\n";

        // Add the complete subcircuit definition
        std::string subckt_str = subckt.str();
        m_models.push_back(subckt_str);
        spdlog::debug("[BuckConverter] Added subcircuit definition:\n{}", subckt_str);
    }

    // Add subcircuit instance - simplified interface: VIN, GND, VOUT only
    // PWM is generated internally by the subcircuit
    add_subcircuit(name, subckt_name,
                   {node_vin, node_gnd, node_vout},
                   {{"DUTY", duty_cycle}, {"CVAL", capacitance}, {"ROUT", 0.5}});
}


void NetlistBuilder::add_boost_converter(const std::string& name,
                                      const std::string& node_vin,
                                      const std::string& node_gnd,
                                      const std::string& node_vout,
                                      double inductance,
                                      double capacitance,
                                      double switching_freq,
                                      const std::string& mosfet_model,
                                      double duty_cycle) {
    // Generate boost converter subcircuit
    // Use a single shared subcircuit for all boost converters
    std::string subckt_name = "BOOST_CONV";
    duty_cycle = std::clamp(duty_cycle, 0.0, 0.95);
    const double boost_gain = 1.0 / (1.0 - duty_cycle);

    // Check if subcircuit already exists
    bool subckt_exists = false;
    for (const auto& m : m_models) {
        if (m.find(".subckt " + subckt_name) == 0) {
            subckt_exists = true;
            break;
        }
    }

    if (!subckt_exists) {
        std::stringstream subckt;

        subckt << ".subckt " << subckt_name << " VIN GND VOUT params: GAIN=2 CVAL=100u ROUT=0.01\n";

        // Behavioral voltage source plus finite output impedance to avoid
        // ideal source conflicts when output is externally forced.
        subckt << "V_IN_SENSE VIN VIN_INT 0\n";
        subckt << "R_SENSE VIN_INT SENSE 1e9\n";
        subckt << "E_BOOST VINT GND SENSE GND {GAIN}\n";
        subckt << "R_OUT VINT VOUT {ROUT}\n";
        subckt << "B_INPUT VIN_INT GND I={max(0, -GAIN*I(E_BOOST))}\n";

        // Output filter capacitor
        subckt << "C_FILTER VOUT GND {CVAL}\n";

        // Load resistor (for simulation stability)
        subckt << "R_LOAD VOUT GND 1000\n";

        subckt << ".ends\n";

        // Add the complete subcircuit definition
        std::string subckt_str = subckt.str();
        m_models.push_back(subckt_str);
        spdlog::debug("[BoostConverter] Added subcircuit definition:\n{}", subckt_str);
    }

    // Add subcircuit instance - simplified interface: VIN, GND, VOUT only
    add_subcircuit(name, subckt_name,
                   {node_vin, node_gnd, node_vout},
                   {{"GAIN", boost_gain}, {"CVAL", capacitance}, {"ROUT", 0.01}});
}

void NetlistBuilder::add_simulation(const std::string& type,
                                   const std::string& parameters) {
    std::string command = "." + type;
    if (!parameters.empty()) {
        command += " " + parameters;
    }
    m_commands.push_back(command);
}

void NetlistBuilder::add_plot(const std::vector<std::string>& nodes) {
    std::string cmd = ".plot";
    for (const auto& node : nodes) {
        cmd += " " + node;
    }
    m_commands.push_back(cmd);
}

void NetlistBuilder::add_model(const std::string& model_def) {
    m_models.push_back(model_def);
    spdlog::debug("[NetlistBuilder] Added model ({} chars): '{}'", model_def.length(), model_def);
}

std::string NetlistBuilder::build() const {
    std::stringstream ss;

    // Title
    ss << "* " << m_title << "\n";

    // Models
    spdlog::debug("[NetlistBuilder] Building netlist with {} models", m_models.size());
    for (size_t i = 0; i < m_models.size(); ++i) {
        const auto& model = m_models[i];
        spdlog::debug("[NetlistBuilder] Model {} ({} chars, starts with '{}', contains .ends: {})",
                      i, model.length(), model.substr(0, std::min(size_t(20), model.length())),
                      model.find(".ends") != std::string::npos);
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
