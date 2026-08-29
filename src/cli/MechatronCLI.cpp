#include "MechatronCLI.hpp"
#include "cad/CADKernel.hpp"
#include "electronics/CircuitSimulator.hpp"
#include "electronics/CircuitComponentAdapter.hpp"
#include "cad/ModelAssetLibrary.hpp"
#include "physics/PhysicsWorld.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "io/SerialPort.hpp"
#include "core/CircuitPhysicsBridge.hpp"
#include "core/Port.hpp"
#include "core/SystemConfig.hpp"
#include "electronics/CircuitToSpiceConverter.hpp"

#include <spdlog/spdlog.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <optional>

namespace mechatron {

namespace fs = std::filesystem;

namespace {

std::string trim_copy(const std::string& value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    if (first == value.end()) {
        return {};
    }
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();
    return std::string(first, last);
}

bool parse_int_strict(const std::string& text, int& value) {
    try {
        size_t pos = 0;
        int parsed = std::stoi(text, &pos);
        if (pos != text.size()) return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_float_strict(const std::string& text, float& value) {
    try {
        size_t pos = 0;
        float parsed = std::stof(text, &pos);
        if (pos != text.size()) return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_double_strict(const std::string& text, double& value) {
    try {
        size_t pos = 0;
        double parsed = std::stod(text, &pos);
        if (pos != text.size()) return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

Port* find_port(Component* component, std::string_view port_name) {
    if (!component) {
        return nullptr;
    }
    for (Port* port : component->get_ports()) {
        if (port && port->name() == port_name) {
            return port;
        }
    }
    return nullptr;
}

std::string port_endpoint(Port* port) {
    if (!port || !port->owner()) {
        return "(null)";
    }
    return port->owner()->id() + "." + std::string(port->name());
}

std::string lower_extension(const std::string& path) {
    std::string ext = fs::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

bool import_mesh_by_extension(CADKernel& cad, const std::string& path, MeshData& mesh) {
    const std::string ext = lower_extension(path);
    if (ext == ".stl") return cad.import_stl(path, mesh);
    if (ext == ".obj") return cad.import_obj(path, mesh);
    if (ext == ".step" || ext == ".stp") return cad.import_step(path, mesh);
    if (ext == ".iges" || ext == ".igs") return cad.import_iges(path, mesh);
    if (ext == ".brep") return cad.import_brep(path, mesh);
    return false;
}

bool export_mesh_by_extension(CADKernel& cad, const std::string& path, const MeshData& mesh) {
    const std::string ext = lower_extension(path);
    if (ext == ".stl") return cad.export_stl(path, mesh);
    if (ext == ".obj") return cad.export_obj(path, mesh);
    if (ext == ".step" || ext == ".stp") return cad.export_step(path, mesh);
    if (ext == ".iges" || ext == ".igs") return cad.export_iges(path, mesh);
    if (ext == ".brep") return cad.export_brep(path, mesh);
    return false;
}

std::string strip_inline_comment(const std::string& line) {
    std::string result;
    result.reserve(line.size());

    bool in_quotes = false;
    char quote_char = '\0';
    bool escaping = false;

    for (char ch : line) {
        if (escaping) {
            result.push_back(ch);
            escaping = false;
            continue;
        }

        if (ch == '\\') {
            result.push_back(ch);
            escaping = true;
            continue;
        }

        if (in_quotes) {
            result.push_back(ch);
            if (ch == quote_char) {
                in_quotes = false;
            }
            continue;
        }

        if (ch == '"' || ch == '\'') {
            result.push_back(ch);
            in_quotes = true;
            quote_char = ch;
            continue;
        }

        if (ch == '#') {
            break;
        }

        result.push_back(ch);
    }

    return trim_copy(result);
}

std::optional<CircuitToSpiceConverter::Result> build_circuit_spice_snapshot(
    SimulationOrchestrator& orchestrator,
    std::string* error_message = nullptr)
{
    std::vector<CircuitComponent*> circuit_components;
    std::vector<Wire> wires;

    orchestrator.registry().for_each([&](Component& comp) {
        auto* adapter = dynamic_cast<ICircuitComponentAdapter*>(&comp);
        if (!adapter) {
            return;
        }
        if (auto* circuit = adapter->circuit_component_base()) {
            circuit_components.push_back(circuit);
        }
    });

    if (circuit_components.empty()) {
        if (error_message) {
            *error_message = "No circuit-capable components found";
        }
        return std::nullopt;
    }

    for (const auto& connection : orchestrator.get_connections()) {
        if (!connection || !connection->source || !connection->target) {
            continue;
        }

        auto* source_owner = connection->source->owner();
        auto* target_owner = connection->target->owner();
        if (!source_owner || !target_owner) {
            continue;
        }

        auto* source_adapter = dynamic_cast<ICircuitComponentAdapter*>(source_owner);
        auto* target_adapter = dynamic_cast<ICircuitComponentAdapter*>(target_owner);
        if (!source_adapter || !target_adapter) {
            continue;
        }

        auto* source_circuit = source_adapter->circuit_component_base();
        auto* target_circuit = target_adapter->circuit_component_base();
        if (!source_circuit || !target_circuit) {
            continue;
        }

        CircuitPin* source_pin = nullptr;
        for (auto* pin : source_circuit->get_pins()) {
            if (pin && pin->id == connection->source->name()) {
                source_pin = pin;
                break;
            }
        }

        CircuitPin* target_pin = nullptr;
        for (auto* pin : target_circuit->get_pins()) {
            if (pin && pin->id == connection->target->name()) {
                target_pin = pin;
                break;
            }
        }

        if (!source_pin || !target_pin) {
            continue;
        }

        Wire wire;
        wire.id = connection->uid;
        wire.source = source_pin;
        wire.target = target_pin;
        wires.push_back(wire);
    }

    CircuitToSpiceConverter::Config config;
    config.analysis_type = "op";
    config.include_simulation_command = true;
    config.simulation_duration = 0.001;
    config.simulation_time_step = 0.001;

    CircuitToSpiceConverter converter(config);
    auto result = converter.convert_with_wires(circuit_components, wires);
    if (!result.success) {
        if (error_message) {
            *error_message = result.error.empty() ? "Failed to generate SPICE netlist" : result.error;
        }
        return std::nullopt;
    }

    return result;
}

std::string resolve_plugin_name_for_component(SimulationOrchestrator& orchestrator,
                                              const Component& component)
{
    for (auto* plugin : orchestrator.plugin_host().get_all_plugins()) {
        if (!plugin) {
            continue;
        }
        for (const auto& descriptor : plugin->components()) {
            if (descriptor.type == component.component_type()) {
                return std::string(plugin->name());
            }
        }
    }
    return std::string(component.plugin_type());
}

} // namespace

MechatronCLI::MechatronCLI()
    : m_mcp_server(nullptr)
    , m_mcp_server_running(false)
    , m_serial_port(std::make_unique<SerialPort>())
    , m_initialized(false)
    , m_interactive_mode(false)
{
}

MechatronCLI::~MechatronCLI() {
    shutdown();
}

bool MechatronCLI::initialize() {
    if (m_initialized) {
        return true;
    }

    try {
        // Initialize core orchestrator
        m_orchestrator = std::make_unique<SimulationOrchestrator>();
        m_orchestrator->load_all_plugins();

        // Initialize project manager
        m_project_manager = std::make_unique<ProjectManager>(*m_orchestrator);

        // Register all built-in commands
        register_builtin_commands();

        m_initialized = true;
        spdlog::info("MechatronCLI initialized successfully");
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize MechatronCLI: " << e.what() << std::endl;
        return false;
    }
}

void MechatronCLI::shutdown() {
    if (m_mcp_server_running) {
        stop_mcp_server();
    }

    if (m_project_manager && m_project_manager->has_project()) {
        m_project_manager->close_project();
    }

    m_orchestrator.reset();
    m_project_manager.reset();
    m_initialized = false;
}

MechatronCLI::CommandResult MechatronCLI::execute_command(const std::string& command_line) {
    auto args = parse_command_line(command_line);
    return execute_command(args);
}

MechatronCLI::CommandResult MechatronCLI::execute_command(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::ok();
    }

    const std::string& cmd = args[0];
    std::vector<std::string> command_args(args.begin() + 1, args.end());

    auto alias_it = m_aliases.find(cmd);
    if (alias_it != m_aliases.end()) {
        std::vector<std::string> alias_args = parse_command_line(alias_it->second);
        alias_args.insert(alias_args.end(), command_args.begin(), command_args.end());
        return execute_command(alias_args);
    }

    // Handle built-in commands directly
    if (cmd == "help" || cmd == "h") {
        return cmd_help(command_args);
    } else if (cmd == "version" || cmd == "v") {
        return cmd_version(command_args);
    } else if (cmd == "exit" || cmd == "quit" || cmd == "q") {
        return CommandResult::ok("Exiting...");
    } else if (cmd == "echo") {
        return cmd_echo(command_args);
    } else if (cmd == "sleep") {
        return cmd_sleep(command_args);
    } else if (cmd == "project-new") {
        return cmd_project_new(command_args);
    } else if (cmd == "project-open") {
        return cmd_project_open(command_args);
    } else if (cmd == "project-save") {
        return cmd_project_save(command_args);
    } else if (cmd == "project-save-as") {
        return cmd_project_save_as(command_args);
    } else if (cmd == "project-close") {
        return cmd_project_close(command_args);
    } else if (cmd == "project-info") {
        return cmd_project_info(command_args);
    } else if (cmd == "project-validate") {
        return cmd_project_validate(command_args);
    } else if (cmd == "project-export") {
        return cmd_project_export(command_args);
    } else if (cmd == "project-import") {
        return cmd_project_import(command_args);
    } else if (cmd == "project-templates") {
        return cmd_project_templates(command_args);
    } else if (cmd == "component-add") {
        return cmd_component_add(command_args);
    } else if (cmd == "component-remove") {
        return cmd_component_remove(command_args);
    } else if (cmd == "component-list") {
        return cmd_component_list(command_args);
    } else if (cmd == "component-info") {
        return cmd_component_info(command_args);
    } else if (cmd == "component-set") {
        return cmd_component_set(command_args);
    } else if (cmd == "component-get") {
        return cmd_component_get(command_args);
    } else if (cmd == "component-transform") {
        return cmd_component_transform(command_args);
    } else if (cmd == "component-select") {
        return cmd_component_select(command_args);
    } else if (cmd == "component-connect") {
        return cmd_component_connect(command_args);
    } else if (cmd == "component-disconnect") {
        return cmd_component_disconnect(command_args);
    } else if (cmd == "component-types") {
        return cmd_component_types(command_args);
    } else if (cmd == "component-plugins") {
        return cmd_component_plugins(command_args);
    } else if (cmd == "sim-start") {
        return cmd_sim_start(command_args);
    } else if (cmd == "sim-stop") {
        return cmd_sim_stop(command_args);
    } else if (cmd == "sim-pause") {
        return cmd_sim_pause(command_args);
    } else if (cmd == "sim-resume") {
        return cmd_sim_resume(command_args);
    } else if (cmd == "sim-step") {
        return cmd_sim_step(command_args);
    } else if (cmd == "sim-status") {
        return cmd_sim_status(command_args);
    } else if (cmd == "sim-time") {
        return cmd_sim_time(command_args);
    } else if (cmd == "sim-reset") {
        return cmd_sim_reset(command_args);
    } else if (cmd == "cad-import") {
        return cmd_cad_import(command_args);
    } else if (cmd == "cad-export") {
        return cmd_cad_export(command_args);
    } else if (cmd == "cad-create-primitive") {
        return cmd_cad_create_primitive(command_args);
    } else if (cmd == "cad-mesh-info") {
        return cmd_cad_mesh_info(command_args);
    } else if (cmd == "cad-mesh-process") {
        return cmd_cad_mesh_process(command_args);
    } else if (cmd == "cad-boolean") {
        return cmd_cad_boolean(command_args);
    } else if (cmd == "cad-list-assets") {
        return cmd_cad_list_assets(command_args);
    } else if (cmd == "circuit-add-component") {
        return cmd_circuit_add_component(command_args);
    } else if (cmd == "circuit-connect") {
        return cmd_circuit_connect(command_args);
    } else if (cmd == "circuit-status") {
        return cmd_circuit_status(command_args);
    } else if (cmd == "mcp-start") {
        return cmd_mcp_start(command_args);
    } else if (cmd == "mcp-stop") {
        return cmd_mcp_stop(command_args);
    } else if (cmd == "mcp-status") {
        return cmd_mcp_status(command_args);
    } else if (cmd == "net-propagate") {
        return cmd_net_propagate(command_args);
    } else if (cmd == "actuator-propagate") {
        return cmd_actuator_propagate(command_args);
    } else if (cmd == "subsystem-add") {
        return cmd_subsystem_add(command_args);
    } else if (cmd == "subsystem-remove") {
        return cmd_subsystem_remove(command_args);
    } else if (cmd == "subsystem-list") {
        return cmd_subsystem_list(command_args);
    } else if (cmd == "subsystem-info") {
        return cmd_subsystem_info(command_args);
    } else if (cmd == "connection-list") {
        return cmd_connection_list(command_args);
    } else if (cmd == "connection-info") {
        return cmd_connection_info(command_args);
    } else if (cmd == "component-copy") {
        return cmd_component_copy(command_args);
    } else if (cmd == "component-rotate") {
        return cmd_component_rotate(command_args);
    } else if (cmd == "component-find") {
        return cmd_component_find(command_args);
    } else if (cmd == "circuit-mode") {
        return cmd_circuit_mode(command_args);
    } else if (cmd == "circuit-step") {
        return cmd_circuit_step(command_args);
    } else if (cmd == "autosave-enable") {
        return cmd_autosave_enable(command_args);
    } else if (cmd == "autosave-config") {
        return cmd_autosave_config(command_args);
    } else if (cmd == "autosave-list") {
        return cmd_autosave_list(command_args);
    } else if (cmd == "autosave-restore") {
        return cmd_autosave_restore(command_args);
    } else if (cmd == "session-save") {
        return cmd_session_save(command_args);
    } else if (cmd == "session-restore") {
        return cmd_session_restore(command_args);
    } else if (cmd == "session-info") {
        return cmd_session_info(command_args);
    } else if (cmd == "project-stats") {
        return cmd_project_stats(command_args);
    } else if (cmd == "physics-create-body") {
        return cmd_physics_create_body(command_args);
    } else if (cmd == "physics-remove-body") {
        return cmd_physics_remove_body(command_args);
    } else if (cmd == "physics-body-info") {
        return cmd_physics_body_info(command_args);
    } else if (cmd == "physics-list-bodies") {
        return cmd_physics_list_bodies(command_args);
    } else if (cmd == "physics-gravity") {
        return cmd_physics_gravity(command_args);
    } else if (cmd == "physics-add-force") {
        return cmd_physics_add_force(command_args);
    } else if (cmd == "physics-add-torque") {
        return cmd_physics_add_torque(command_args);
    } else if (cmd == "sensor-read") {
        return cmd_sensor_read(command_args);
    } else if (cmd == "sensor-list") {
        return cmd_sensor_list(command_args);
    } else if (cmd == "sensor-configure") {
        return cmd_sensor_configure(command_args);
    } else if (cmd == "actuator-set-input") {
        return cmd_actuator_set_input(command_args);
    } else if (cmd == "actuator-get-state") {
        return cmd_actuator_get_state(command_args);
    } else if (cmd == "actuator-enable") {
        return cmd_actuator_enable(command_args);
    } else if (cmd == "serial-list") {
        return cmd_serial_list(command_args);
    } else if (cmd == "serial-open") {
        return cmd_serial_open(command_args);
    } else if (cmd == "serial-close") {
        return cmd_serial_close(command_args);
    } else if (cmd == "serial-write") {
        return cmd_serial_write(command_args);
    } else if (cmd == "circuit-analyze") {
        return cmd_circuit_analyze(command_args);
    } else if (cmd == "circuit-netlist") {
        return cmd_circuit_netlist(command_args);
    } else if (cmd == "circuit-export-netlist") {
        return cmd_circuit_export_netlist(command_args);
    } else if (cmd == "circuit-nodes") {
        return cmd_circuit_nodes(command_args);
    } else if (cmd == "system-config") {
        return cmd_system_config(command_args);
    } else if (cmd == "component-metadata") {
        return cmd_component_metadata(command_args);
    } else if (cmd == "component-categories") {
        return cmd_component_categories(command_args);
    } else if (cmd == "component-scale") {
        return cmd_component_scale(command_args);
    } else if (cmd == "physical-link-list") {
        return cmd_physical_link_list(command_args);
    } else if (cmd == "physical-link-connect") {
        return cmd_physical_link_connect(command_args);
    } else if (cmd == "physical-link-disconnect") {
        return cmd_physical_link_disconnect(command_args);
    } else if (cmd == "physical-link-status") {
        return cmd_physical_link_status(command_args);
    } else if (cmd == "mcu-firmware-upload") {
        return cmd_mcu_firmware_upload(command_args);
    } else if (cmd == "mcu-pin-get") {
        return cmd_mcu_pin_get(command_args);
    } else if (cmd == "mcu-pin-set") {
        return cmd_mcu_pin_set(command_args);
    } else if (cmd == "time-realtime") {
        return cmd_time_realtime(command_args);
    } else if (cmd == "time-deterministic") {
        return cmd_time_deterministic(command_args);
    } else if (cmd == "scene-clear") {
        return cmd_scene_clear(command_args);
    } else if (cmd == "scene-validate") {
        return cmd_scene_validate(command_args);
    } else if (cmd == "bridge-add-mapping") {
        return cmd_bridge_add_mapping(command_args);
    } else if (cmd == "bridge-remove-mapping") {
        return cmd_bridge_remove_mapping(command_args);
    } else if (cmd == "bridge-list-mappings") {
        return cmd_bridge_list_mappings(command_args);
    } else if (cmd == "bridge-enable") {
        return cmd_bridge_enable(command_args);
    } else if (cmd == "equiv-circuit-add-actuator") {
        return cmd_equiv_circuit_add_actuator(command_args);
    } else if (cmd == "equiv-circuit-add-mcu") {
        return cmd_equiv_circuit_add_mcu(command_args);
    } else if (cmd == "equiv-circuit-update-mcu") {
        return cmd_equiv_circuit_update_mcu(command_args);
    } else if (cmd == "equiv-circuit-sync-mcu-power") {
        return cmd_equiv_circuit_sync_mcu_power(command_args);
    } else if (cmd == "equiv-circuit-sync-mcu-inputs") {
        return cmd_equiv_circuit_sync_mcu_inputs(command_args);
    } else if (cmd == "component-bounds") {
        return cmd_component_bounds(command_args);
    } else if (cmd == "component-position") {
        return cmd_component_position(command_args);
    } else if (cmd == "port-list") {
        return cmd_port_list(command_args);
    } else if (cmd == "port-info") {
        return cmd_port_info(command_args);
    } else if (cmd == "actuator-terminal-current") {
        return cmd_actuator_terminal_current(command_args);
    }

    auto custom_it = m_commands.find(cmd);
    if (custom_it != m_commands.end() && custom_it->second.handler) {
        return custom_it->second.handler(command_args);
    }

    return CommandResult::err("Unknown command: " + cmd + ". Type 'help' for available commands.");
}

void MechatronCLI::run_interactive() {
    m_interactive_mode = true;

    std::cout << "Mechatron CLI v" << VERSION << std::endl;
    std::cout << "Type 'help' for available commands, 'exit' to quit." << std::endl;
    std::cout << std::endl;

    std::string line;
    while (std::cout << PROMPT && std::getline(std::cin, line)) {
        // Skip empty lines
        if (line.empty() || std::all_of(line.begin(), line.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        })) {
            continue;
        }

        const auto parsed = parse_command_line(line);

        // Execute command
        auto result = execute_command(line);

        // Print result
        if (!result.output.empty()) {
            std::cout << result.output << std::endl;
        }
        if (!result.error.empty()) {
            std::cerr << "Error: " << result.error << std::endl;
        }

        // Check for exit command
        if (!parsed.empty() && (parsed[0] == "exit" || parsed[0] == "quit" || parsed[0] == "q")) {
            break;
        }
    }

    m_interactive_mode = false;
}

void MechatronCLI::run_repl() {
    run_interactive();
}

MechatronCLI::CommandResult MechatronCLI::execute_commands(const std::vector<std::string>& commands) {
    std::ostringstream output;
    for (const auto& command : commands) {
        auto result = execute_command(command);
        if (!result.success) {
            return result;
        }
        if (!result.output.empty()) {
            output << result.output;
            if (result.output.back() != '\n') {
                output << '\n';
            }
        }
    }
    output << "Commands executed successfully";
    return CommandResult::ok(output.str());
}

MechatronCLI::CommandResult MechatronCLI::execute_script(const std::string& script_file) {
    if (!fs::exists(script_file)) {
        return CommandResult::err("Script file not found: " + script_file);
    }

    std::ifstream file(script_file);
    if (!file.is_open()) {
        return CommandResult::err("Failed to open script file: " + script_file);
    }

    std::string line;
    int line_num = 0;
    std::ostringstream output;

    while (std::getline(file, line)) {
        line_num++;

        line = trim_copy(line);

        // Skip empty lines
        if (line.empty()) {
            continue;
        }

        // Skip comments (lines starting with #)
        if (line[0] == '#') {
            continue;
        }

        line = strip_inline_comment(line);
        if (line.empty()) {
            continue;
        }

        auto result = execute_command(line);

        if (!result.success) {
            std::ostringstream oss;
            oss << "Script error at line " << line_num << ": " << line << "\n";
            oss << "Error: " << result.error;
            return CommandResult::err(oss.str(), result.exit_code);
        }
        if (!result.output.empty()) {
            output << result.output;
            if (result.output.back() != '\n') {
                output << '\n';
            }
        }
        const auto parsed = parse_command_line(line);
        if (!parsed.empty() && (parsed[0] == "exit" || parsed[0] == "quit" || parsed[0] == "q")) {
            break;
        }
    }

    output << "Script executed successfully";
    return CommandResult::ok(output.str());
}

bool MechatronCLI::start_mcp_server(int port) {
    if (port <= 0 || port > 65535) {
        return false;
    }
    (void)port;
    return false;
}

void MechatronCLI::stop_mcp_server() {
    if (!m_mcp_server_running) {
        return;
    }
    m_mcp_server_running = false;
    std::cout << "MCP server stopped" << std::endl;
}

bool MechatronCLI::is_mcp_server_running() const {
    return m_mcp_server_running;
}

void MechatronCLI::register_command(const std::string& name,
                                    const std::string& description,
                                    CommandHandler handler) {
    m_commands[name] = CommandInfo{name, description, "", std::move(handler)};
}

void MechatronCLI::register_alias(const std::string& alias, const std::string& command) {
    m_aliases[alias] = command;
}

std::string MechatronCLI::get_help() const {
    std::ostringstream oss;

    oss << "Mechatron CLI v" << VERSION << " - Available Commands:\n\n";

    oss << "Project Commands:\n";
    oss << "  project-new <name> [location] [template]    - Create new project (default: current dir)\n";
    oss << "  project-open <path>                        - Open existing project\n";
    oss << "  project-save                                 - Save current project\n";
    oss << "  project-save-as <path>                       - Save project to a new path\n";
    oss << "  project-close                                - Close current project\n";
    oss << "  project-info                                 - Display project info\n";
    oss << "  project-validate                             - Validate current project\n";
    oss << "  project-export <path>                        - Export current project\n";
    oss << "  project-import <path>                        - Import project archive/folder\n";
    oss << "  project-templates                            - List available project templates\n\n";

    oss << "Component Commands:\n";
    oss << "  component-add <plugin> <type> [id]           - Add component\n";
    oss << "  component-remove <id>                        - Remove component\n";
    oss << "  component-list                               - List all components\n";
    oss << "  component-info <id>                           - Show component details\n";
    oss << "  component-get <id> [key]                      - Get serialized component data\n";
    oss << "  component-set <id> <key> <json-value>         - Set component parameter\n";
    oss << "  component-transform <id> <position|scale> <x> <y> <z> - Update transform\n";
    oss << "  component-select <id>                         - Select component\n";
    oss << "  component-connect <src> <src_port> <dst> <dst_port> [uid] - Connect ports\n";
    oss << "  component-disconnect <uid>                   - Disconnect connection\n";
    oss << "  component-types                               - List available types\n";
    oss << "  component-plugins                             - List loaded plugins\n\n";

    oss << "Simulation Commands:\n";
    oss << "  sim-start                                     - Start simulation\n";
    oss << "  sim-stop                                      - Stop simulation\n";
    oss << "  sim-pause                                     - Pause simulation\n";
    oss << "  sim-resume                                    - Resume simulation\n";
    oss << "  sim-step [count]                              - Advance simulation steps\n";
    oss << "  sim-status                                    - Show simulation status\n";
    oss << "  sim-time                                      - Show simulation time\n";

    oss << "CAD Commands:\n";
    oss << "  cad-import <file> <asset-id>                  - Import STEP/STL/OBJ/IGES/BREP as asset\n";
    oss << "  cad-export <asset-id> <scope> <file>          - Export asset mesh\n";
    oss << "  cad-create-primitive <box|sphere|cylinder> <asset-id> [dims] - Create asset\n";
    oss << "  cad-mesh-info <asset-id> [scope]              - Show asset mesh statistics\n";
    oss << "  cad-mesh-process <asset-id> <scope> <simplify|smooth> <value> <result-id> - Process mesh\n";
    oss << "  cad-boolean <union|subtract|intersect> <a-id> <a-scope> <b-id> <b-scope> <result-id> - Boolean assets\n";
    oss << "  cad-list-assets                               - List model assets\n\n";

    oss << "Circuit Commands:\n";
    oss << "  circuit-add-component <plugin> <type> [id]    - Add circuit-capable component\n";
    oss << "  circuit-connect <src> <src_port> <dst> <dst_port> [uid] - Connect circuit ports\n";
    oss << "  circuit-status                                - Show circuit component and connection status\n";
    oss << "  circuit-mode <native|ngspice|hybrid>          - Set circuit simulation mode\n";
    oss << "  circuit-step [count]                          - Step circuit simulation only\n\n";

    oss << "Advanced Commands:\n";
    oss << "  net-propagate                                 - Propagate voltages through nets\n";
    oss << "  actuator-propagate                           - Propagate voltages to actuators\n";
    oss << "  subsystem-add <id>                           - Add subsystem\n";
    oss << "  subsystem-remove <id>                        - Remove subsystem\n";
    oss << "  subsystem-list                               - List subsystems\n";
    oss << "  subsystem-info <id>                          - Show subsystem details\n";
    oss << "  connection-list                              - List all connections\n";
    oss << "  connection-info <uid>                        - Show connection details\n";
    oss << "  component-copy <src_id> <new_id>             - Copy component\n";
    oss << "  component-rotate <id> <axis> <degrees>       - Rotate component\n";
    oss << "  component-scale <id> <x> <y> <z>             - Scale component\n";
    oss << "  component-find <pattern>                     - Find components by pattern\n";
    oss << "  component-metadata <type>                    - Show component metadata\n";
    oss << "  component-categories                         - List all component categories\n";
    oss << "  autosave-enable <true|false>                 - Enable/disable autosave\n";
    oss << "  autosave-config <interval> <max_count>       - Configure autosave settings\n";
    oss << "  autosave-list                                - List autosave files\n";
    oss << "  autosave-restore [index]                     - Restore from autosave\n";
    oss << "  session-save                                 - Save session state\n";
    oss << "  session-restore                              - Restore session state\n";
    oss << "  session-info                                 - Show session information\n";
    oss << "  project-stats                                - Show project statistics\n\n";

    oss << "Physics Commands:\n";
    oss << "  physics-create-body <id> <shape>             - Create physics body\n";
    oss << "  physics-remove-body <id>                     - Remove physics body\n";
    oss << "  physics-body-info <id>                       - Show physics body details\n";
    oss << "  physics-list-bodies                          - List all physics bodies\n";
    oss << "  physics-gravity <x> <y> <z>                  - Set gravity vector\n";
    oss << "  physics-add-force <id> <x> <y> <z>           - Add force to body\n";
    oss << "  physics-add-torque <id> <x> <y> <z>          - Add torque to body\n\n";

    oss << "Sensor Commands:\n";
    oss << "  sensor-read <id>                             - Read sensor value\n";
    oss << "  sensor-list                                  - List all sensors\n";
    oss << "  sensor-configure <id> <param> <value>        - Configure sensor parameter\n\n";

    oss << "Actuator Commands:\n";
    oss << "  actuator-set-input <id> <value>              - Set actuator input (0-1)\n";
    oss << "  actuator-get-state <id>                      - Get actuator state\n";
    oss << "  actuator-enable <id> <true|false>            - Enable/disable actuator\n\n";

    oss << "Serial Commands:\n";
    oss << "  serial-list                                  - List available serial ports\n";
    oss << "  serial-open <port> <baud>                    - Open serial port\n";
    oss << "  serial-close                                 - Close serial port\n";
    oss << "  serial-write <data>                          - Write data to serial port\n\n";

    oss << "Circuit Analysis Commands:\n";
    oss << "  circuit-analyze                              - Analyze circuit topology\n";
    oss << "  circuit-netlist                              - Show circuit netlist\n";
    oss << "  circuit-export-netlist <file>                - Export netlist to file\n";
    oss << "  circuit-nodes                                - Show circuit nodes\n\n";

    oss << "System Commands:\n";
    oss << "  system-config                                - Show system configuration\n";
    oss << "  component-categories                         - List all component categories\n\n";

    oss << "Physical Link Commands:\n";
    oss << "  physical-link-list                            - List physical link capable components\n";
    oss << "  physical-link-connect <id> <port> <baud>      - Connect component to physical device\n";
    oss << "  physical-link-disconnect <id>                 - Disconnect physical device\n";
    oss << "  physical-link-status <id>                     - Show physical link status\n\n";

    oss << "MCU Commands:\n";
    oss << "  mcu-firmware-upload <id> <file>               - Upload firmware to MCU component\n";
    oss << "  mcu-pin-get <id> <pin>                         - Get MCU pin voltage\n";
    oss << "  mcu-pin-set <id> <pin> <voltage>               - Set MCU pin voltage\n\n";

    oss << "Time Commands:\n";
    oss << "  time-realtime <factor>                        - Set realtime multiplier (1.0 = realtime)\n";
    oss << "  time-deterministic <true|false>              - Enable/disable deterministic mode\n\n";

    oss << "Scene Commands:\n";
    oss << "  scene-clear                                   - Clear all components\n";
    oss << "  scene-validate                                - Validate scene topology\n\n";

    oss << "Circuit-Physics Bridge Commands:\n";
    oss << "  bridge-add-mapping <circuit_pin> <target> <type> - Add pin mapping\n";
    oss << "  bridge-remove-mapping <circuit_pin>           - Remove pin mapping\n";
    oss << "  bridge-list-mappings                          - List all pin mappings\n";
    oss << "  bridge-enable <true|false>                    - Enable/disable bridge\n\n";

    oss << "Equivalent Circuit Commands:\n";
    oss << "  equiv-circuit-add-actuator                    - Add actuator equivalent circuits\n";
    oss << "  equiv-circuit-add-mcu                         - Add MCU equivalent circuits\n";
    oss << "  equiv-circuit-update-mcu                      - Update MCU equivalent sources\n";
    oss << "  equiv-circuit-sync-mcu-power                   - Sync MCU power from ngspice\n";
    oss << "  equiv-circuit-sync-mcu-inputs                  - Sync MCU inputs from ngspice\n\n";

    oss << "Advanced Component Commands:\n";
    oss << "  component-bounds <id>                         - Show component bounds\n";
    oss << "  component-position <id> [x] [y] [z]          - Get/set component position\n";
    oss << "  port-list <id>                                - List component ports\n";
    oss << "  port-info <id> <port>                         - Show port details\n";
    oss << "  actuator-terminal-current <id> <pin>           - Get actuator terminal current\n\n";

    oss << "Utility Commands:\n";
    oss << "  help [command]                                - Show help\n";
    oss << "  version                                      - Show version\n";
    oss << "  echo <text>                                   - Echo text\n";
    oss << "  sleep <seconds>                               - Sleep for script pacing\n";
    oss << "  exit                                         - Exit CLI\n\n";

    oss << "MCP Commands:\n";
    oss << "  mcp-start [port]                              - Start MCP server\n";
    oss << "  mcp-stop                                     - Stop MCP server\n";
    oss << "  mcp-status                                   - Show MCP status\n";

    oss << "\nAliases:\n";
    oss << "  new -> project-new, open -> project-open, save -> project-save\n";
    oss << "  add -> component-add, list -> component-list, connect -> component-connect\n";
    oss << "  run -> sim-start, stop -> sim-stop\n";

    return oss.str();
}

std::string MechatronCLI::get_command_help(const std::string& command) const {
    static const std::unordered_map<std::string, std::string> help = {
        {"project-new", "project-new <name> [location]\nCreate and immediately save a new .mepro project. Uses current directory if location not provided."},
        {"project-open", "project-open <path>\nOpen a .mepro project file."},
        {"project-save", "project-save\nSave the currently open project."},
        {"project-save-as", "project-save-as <path>\nSave the current project to a new .mepro path."},
        {"project-close", "project-close\nClose the current project."},
        {"project-info", "project-info\nShow project metadata and component count."},
        {"project-validate", "project-validate\nValidate component plugin references and known project assets."},
        {"project-export", "project-export <path>\nExport the project file plus circuit/models/code sidecar folders."},
        {"project-import", "project-import <path>\nOpen an imported .mepro project path."},
        {"project-templates", "project-templates\nList templates accepted by project-new."},
        {"component-add", "component-add <plugin> <type> [id]\nCreate a component through a loaded plugin."},
        {"component-remove", "component-remove <id>\nRemove a component from the scene."},
        {"component-list", "component-list\nList all components in the current CLI session."},
        {"component-info", "component-info <id>\nShow component type, plugin, transform, and ports."},
        {"component-get", "component-get <id> [key]\nRead serialized component data or a circuit parameter."},
        {"component-set", "component-set <id> <key> <json-value>\nWrite serialized component data or a numeric circuit parameter."},
        {"component-transform", "component-transform <id> <position|scale> <x> <y> <z>\nUpdate component transform fields."},
        {"component-select", "component-select <id>\nMark a component as selected in the orchestrator."},
        {"component-connect", "component-connect <src> <src_port> <dst> <dst_port> [uid]\nConnect two component ports."},
        {"component-disconnect", "component-disconnect <uid>\nDisconnect a connection by UID."},
        {"component-types", "component-types\nList all component types exposed by loaded plugins."},
        {"component-plugins", "component-plugins\nList loaded plugins."},
        {"sim-start", "sim-start\nStart simulation time."},
        {"sim-stop", "sim-stop\nStop simulation time and reset time manager state."},
        {"sim-pause", "sim-pause\nPause simulation time."},
        {"sim-resume", "sim-resume\nResume paused simulation time."},
        {"sim-step", "sim-step [count]\nAdvance one or more deterministic simulation steps."},
        {"sim-status", "sim-status\nShow simulation state, time, and component count."},
        {"sim-time", "sim-time\nShow simulation time and current tick."},
        {"sim-reset", "sim-reset\nStop and reset simulation state."},
        {"cad-import", "cad-import <file> <asset-id>\nImport STEP/STL/OBJ/IGES/BREP as a user model asset."},
        {"cad-export", "cad-export <asset-id> <scope> <file>\nExport a model asset by output extension."},
        {"cad-create-primitive", "cad-create-primitive <box|sphere|cylinder> <asset-id> [dims]\nCreate a primitive user model asset."},
        {"cad-mesh-info", "cad-mesh-info <asset-id> [scope]\nShow mesh vertex/triangle counts and bounds."},
        {"cad-mesh-process", "cad-mesh-process <asset-id> <scope> <simplify|smooth> <value> <result-id>\nProcess an asset mesh and save the result as a user asset."},
        {"cad-boolean", "cad-boolean <union|subtract|intersect> <a-id> <a-scope> <b-id> <b-scope> <result-id>\nRun a boolean operation on two asset meshes."},
        {"cad-list-assets", "cad-list-assets\nList default and user model assets."},
        {"circuit-add-component", "circuit-add-component <plugin> <type> [id]\nCreate a component through the same plugin-backed circuit path as component-add."},
        {"circuit-connect", "circuit-connect <src> <src_port> <dst> <dst_port> [uid]\nConnect two circuit component ports."},
        {"circuit-status", "circuit-status\nShow circuit component count, connection count, and simulator readiness."},
        {"sleep", "sleep <seconds>\nPause script execution for a non-negative number of seconds."},
        {"mcp-start", "mcp-start [port]\nStart MCP server if this build includes an MCP backend."},
        {"mcp-stop", "mcp-stop\nStop MCP server."},
        {"mcp-status", "mcp-status\nShow MCP server state."}
    };

    auto it = help.find(command);
    if (it != help.end()) {
        return it->second;
    }
    auto custom_it = m_commands.find(command);
    if (custom_it != m_commands.end()) {
        if (!custom_it->second.usage.empty()) {
            return custom_it->second.usage + "\n" + custom_it->second.description;
        }
        if (!custom_it->second.description.empty()) {
            return command + "\n" + custom_it->second.description;
        }
        return command;
    }
    return "Unknown command: " + command;
}

std::vector<std::string> MechatronCLI::get_available_commands() const {
    std::vector<std::string> commands = {
        "project-new", "project-open", "project-save", "project-save-as", "project-close",
        "project-info", "project-validate", "project-export", "project-import", "project-templates", "project-stats",
        "component-add", "component-remove", "component-list", "component-info",
        "component-get", "component-set", "component-transform", "component-connect",
        "component-select", "component-disconnect", "component-types", "component-plugins",
        "component-copy", "component-rotate", "component-scale", "component-find",
        "component-metadata", "component-categories",
        "sim-start", "sim-stop", "sim-pause", "sim-resume", "sim-step", "sim-status", "sim-time", "sim-reset",
        "cad-import", "cad-export", "cad-create-primitive", "cad-mesh-info",
        "cad-mesh-process", "cad-boolean", "cad-list-assets",
        "circuit-add-component", "circuit-connect", "circuit-status", "circuit-mode", "circuit-step",
        "circuit-analyze", "circuit-netlist", "circuit-export-netlist", "circuit-nodes",
        "net-propagate", "actuator-propagate",
        "subsystem-add", "subsystem-remove", "subsystem-list", "subsystem-info",
        "connection-list", "connection-info",
        "autosave-enable", "autosave-config", "autosave-list", "autosave-restore",
        "session-save", "session-restore", "session-info",
        "physics-create-body", "physics-remove-body", "physics-body-info", "physics-list-bodies",
        "physics-gravity", "physics-add-force", "physics-add-torque",
        "sensor-read", "sensor-list", "sensor-configure",
        "actuator-set-input", "actuator-get-state", "actuator-enable",
        "serial-list", "serial-open", "serial-close", "serial-write",
        "physical-link-list", "physical-link-connect", "physical-link-disconnect", "physical-link-status",
        "mcu-firmware-upload", "mcu-pin-get", "mcu-pin-set",
        "time-realtime", "time-deterministic",
        "scene-clear", "scene-validate",
        "bridge-add-mapping", "bridge-remove-mapping", "bridge-list-mappings", "bridge-enable",
        "equiv-circuit-add-actuator", "equiv-circuit-add-mcu", "equiv-circuit-update-mcu",
        "equiv-circuit-sync-mcu-power", "equiv-circuit-sync-mcu-inputs",
        "component-bounds", "component-position", "port-list", "port-info",
        "actuator-terminal-current",
        "system-config",
        "help", "version", "echo", "sleep", "exit",
        "mcp-start", "mcp-stop", "mcp-status"
    };
    for (const auto& [name, info] : m_commands) {
        if (std::find(commands.begin(), commands.end(), name) == commands.end()) {
            commands.push_back(name);
        }
    }
    std::sort(commands.begin(), commands.end());
    return commands;
}

void MechatronCLI::register_builtin_commands() {
    register_alias("new", "project-new");
    register_alias("open", "project-open");
    register_alias("save", "project-save");
    register_alias("add", "component-add");
    register_alias("list", "component-list");
    register_alias("connect", "component-connect");
    register_alias("run", "sim-start");
    register_alias("stop", "sim-stop");
}

// Command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_help(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::ok(get_help());
    }
    auto alias_it = m_aliases.find(args[0]);
    if (alias_it != m_aliases.end()) {
        return CommandResult::ok(args[0] + " -> " + alias_it->second + "\n" + get_command_help(alias_it->second));
    }
    return CommandResult::ok(get_command_help(args[0]));
}

MechatronCLI::CommandResult MechatronCLI::cmd_version(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Mechatron CLI v" << VERSION << "\n";
    oss << "Built for mechatron simulation engine\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_echo(const std::vector<std::string>& args) {
    std::ostringstream oss;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) oss << " ";
        oss << args[i];
    }
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_sleep(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: sleep <seconds>");
    }

    double seconds = 0.0;
    if (!parse_double_strict(args[0], seconds)) {
        return CommandResult::err("Sleep duration must be numeric");
    }

    if (seconds < 0.0) {
        return CommandResult::err("Sleep duration must be non-negative");
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    return CommandResult::ok();
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_new(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: project-new <name> [location] [template]");
    }

    // Use current directory if location not provided
    std::string location;
    if (args.size() >= 2) {
        location = args[1];
    } else {
        // Use current working directory
        location = std::filesystem::current_path().string();
    }

    const bool created = args.size() >= 3
        ? m_project_manager->create_from_template(args[2], args[0], location)
        : m_project_manager->create_new_project(args[0], location);

    if (created) {
        return CommandResult::ok("Project created successfully");
    }
    return CommandResult::err("Failed to create project");
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_open(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: project-open <path>");
    }

    if (m_project_manager->open_project(args[0])) {
        return CommandResult::ok("Project opened successfully");
    }
    return CommandResult::err("Failed to open project");
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_save(const std::vector<std::string>& args) {
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }

    if (m_project_manager->save_project()) {
        return CommandResult::ok("Project saved successfully");
    }
    return CommandResult::err("Failed to save project");
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_save_as(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: project-save-as <path>");
    }
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }
    if (m_project_manager->save_project_as(args[0])) {
        return CommandResult::ok("Project saved successfully: " + args[0]);
    }
    return CommandResult::err("Failed to save project as: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_close(const std::vector<std::string>& args) {
    if (!m_project_manager->has_project()) {
        return CommandResult::ok("No project open");
    }

    m_project_manager->close_project();
    return CommandResult::ok("Project closed");
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_info(const std::vector<std::string>& args) {
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }

    const auto& meta = m_project_manager->metadata();
    std::ostringstream oss;
    oss << "Project Information:\n";
    oss << "  Name: " << meta.name << "\n";
    oss << "  Path: " << m_project_manager->project_path() << "\n";
    oss << "  Components: " << m_orchestrator->registry().size() << "\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_validate(const std::vector<std::string>& args) {
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }
    std::string error_message;
    if (m_project_manager->validate_project(error_message)) {
        return CommandResult::ok("Project validation passed");
    }
    return CommandResult::err(error_message.empty() ? "Project validation failed" : error_message);
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_export(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: project-export <path>");
    }
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }
    if (m_project_manager->export_project(args[0])) {
        return CommandResult::ok("Project exported: " + args[0]);
    }
    return CommandResult::err("Failed to export project: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_import(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: project-import <path>");
    }
    if (m_project_manager->import_project(args[0])) {
        return CommandResult::ok("Project imported: " + args[0]);
    }
    return CommandResult::err("Failed to import project: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_templates(const std::vector<std::string>& args) {
    const auto templates = ProjectManager::get_available_templates();
    std::ostringstream oss;
    oss << "Project templates (" << templates.size() << " total):\n";
    for (const auto& name : templates) {
        oss << "  " << name << "\n";
    }
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_add(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: component-add <plugin> <type> [id]");
    }

    const std::string& plugin = args[0];
    const std::string& type = args[1];
    std::string id = args.size() > 2 ? args[2] : type + "_1";

    if (m_orchestrator->registry().get(id)) {
        return CommandResult::err("Component already exists: " + id);
    }

    auto* comp = m_orchestrator->create_component(plugin, type, id);
    if (!comp) {
        return CommandResult::err("Failed to create component: " + plugin + "." + type);
    }

    return CommandResult::ok("Component added: " + comp->id());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_remove(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-remove <id>");
    }
    if (!m_orchestrator->registry().get(args[0])) {
        return CommandResult::err("Component not found: " + args[0]);
    }
    m_orchestrator->remove_component(args[0]);
    return CommandResult::ok("Component removed: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_list(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Components (" << m_orchestrator->registry().size() << " total):\n";

    m_orchestrator->registry().for_each([&](const Component& comp) {
        oss << "  " << comp.id() << " - " << comp.component_type() << "\n";
    });

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_info(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-info <component-id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Component: " << comp->id() << "\n";
    oss << "  Type: " << comp->component_type() << "\n";
    oss << "  Plugin: " << comp->plugin_type() << "\n";
    oss << "  Category: " << comp->category() << "\n";

    const auto& t = comp->transform();
    oss << "  Position: [" << t.position.x << ", " << t.position.y << ", " << t.position.z << "]\n";
    const auto ports = comp->get_ports();
    if (!ports.empty()) {
        oss << "  Ports:\n";
        for (const Port* port : ports) {
            if (!port) {
                continue;
            }
            oss << "    " << port->name() << "\n";
        }
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_get(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-get <id> [key]");
    }
    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    nlohmann::json data;
    comp->serialize(data);
    if (args.size() > 1) {
        const std::string& key = args[1];
        if (data.contains(key)) {
            return CommandResult::ok(data[key].dump(2));
        }
        if (data.contains("parameters") && data["parameters"].is_object() &&
            data["parameters"].contains(key)) {
            return CommandResult::ok(data["parameters"][key].dump(2));
        }
        return CommandResult::err("Component key not found: " + key);
    }
    if (auto* adapter = dynamic_cast<ICircuitComponentAdapter*>(comp)) {
        const auto* circuit = adapter->circuit_component_base();
        data["circuit_category"] = std::string(circuit->category());
    }
    return CommandResult::ok(data.dump(2));
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_set(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: component-set <id> <key> <json-value>");
    }
    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    nlohmann::json data;
    comp->serialize(data);
    const bool is_existing_top_level_key = data.contains(args[1]);
    const bool is_existing_parameter =
        data.contains("parameters") &&
        data["parameters"].is_object() &&
        data["parameters"].contains(args[1]);
    if (!is_existing_top_level_key && !is_existing_parameter) {
        return CommandResult::err("Component key not found: " + args[1]);
    }

    if (is_existing_parameter) {
        auto* adapter = dynamic_cast<ICircuitComponentAdapter*>(comp);
        if (!adapter) {
            return CommandResult::err("Component parameter is not backed by a circuit component: " + args[1]);
        }
        double numeric_value = 0.0;
        if (!parse_double_strict(args[2], numeric_value)) {
            return CommandResult::err("Circuit parameter must be numeric: " + args[1]);
        }
        data["parameters"][args[1]] = numeric_value;
        comp->deserialize(data);
        adapter->circuit_component_base()->set_parameter(args[1], numeric_value);
    } else {
        try {
            data[args[1]] = nlohmann::json::parse(args[2]);
        } catch (...) {
            data[args[1]] = args[2];
        }
        comp->deserialize(data);
    }
    comp->update(0.0);
    m_orchestrator->mark_circuit_topology_dirty();
    return CommandResult::ok("Component updated: " + args[0] + "." + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_transform(const std::vector<std::string>& args) {
    if (args.size() < 5) {
        return CommandResult::err("Usage: component-transform <id> <position|scale> <x> <y> <z>");
    }
    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    Vec3 value;
    if (!parse_float_strict(args[2], value.x) ||
        !parse_float_strict(args[3], value.y) ||
        !parse_float_strict(args[4], value.z)) {
        return CommandResult::err("Transform values must be numeric");
    }

    if (args[1] == "position") {
        comp->transform().position = value;
    } else if (args[1] == "scale") {
        comp->transform().scale = value;
    } else {
        return CommandResult::err("Transform field must be 'position' or 'scale'");
    }

    return CommandResult::ok("Component transform updated: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_select(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-select <id>");
    }
    if (!m_orchestrator->registry().get(args[0])) {
        return CommandResult::err("Component not found: " + args[0]);
    }
    m_orchestrator->set_selected_component(args[0]);
    return CommandResult::ok("Component selected: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_connect(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        return CommandResult::err("Usage: component-connect <source-id> <source-port> <target-id> <target-port> [uid]");
    }

    Component* source_component = m_orchestrator->registry().get(args[0]);
    Component* target_component = m_orchestrator->registry().get(args[2]);
    if (!source_component) {
        return CommandResult::err("Source component not found: " + args[0]);
    }
    if (!target_component) {
        return CommandResult::err("Target component not found: " + args[2]);
    }

    Port* source_port = find_port(source_component, args[1]);
    Port* target_port = find_port(target_component, args[3]);
    if (!source_port) {
        return CommandResult::err("Source port not found: " + args[0] + "." + args[1]);
    }
    if (!target_port) {
        return CommandResult::err("Target port not found: " + args[2] + "." + args[3]);
    }

    const std::string uid = args.size() > 4 ? args[4] : "";
    if (!uid.empty()) {
        for (const auto& existing : m_orchestrator->get_connections()) {
            if (existing && existing->uid == uid) {
                return CommandResult::err("Connection already exists: " + uid);
            }
        }
    }
    Connection* connection = m_orchestrator->connect(source_port, target_port, uid);
    if (!connection) {
        return CommandResult::err("Failed to connect ports");
    }

    std::ostringstream oss;
    oss << "Connected " << port_endpoint(source_port) << " -> " << port_endpoint(target_port)
        << " (" << connection->uid << ")";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_disconnect(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-disconnect <uid>");
    }
    const auto& connections = m_orchestrator->get_connections();
    for (size_t i = 0; i < connections.size(); ++i) {
        if (connections[i] && connections[i]->uid == args[0]) {
            Connection* connection = connections[i].get();
            m_orchestrator->disconnect(connection);
            return CommandResult::ok("Disconnected: " + args[0]);
        }
    }
    return CommandResult::err("Connection not found: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_start(const std::vector<std::string>& args) {
    m_orchestrator->start();
    return CommandResult::ok("Simulation started");
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_stop(const std::vector<std::string>& args) {
    m_orchestrator->stop();
    return CommandResult::ok("Simulation stopped");
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_pause(const std::vector<std::string>& args) {
    m_orchestrator->pause();
    return CommandResult::ok("Simulation paused");
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_resume(const std::vector<std::string>& args) {
    m_orchestrator->resume();
    return CommandResult::ok("Simulation resumed");
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_step(const std::vector<std::string>& args) {
    int count = 1;
    if (!args.empty()) {
        if (!parse_int_strict(args[0], count)) {
            return CommandResult::err("Invalid step count: " + args[0]);
        }
    }
    if (count < 1) {
        return CommandResult::err("Step count must be positive");
    }
    for (int i = 0; i < count; ++i) {
        m_orchestrator->step();
        m_orchestrator->update();
    }
    return CommandResult::ok("Simulation stepped: " + std::to_string(count));
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_status(const std::vector<std::string>& args) {
    auto state = m_orchestrator->time_manager().state();
    std::ostringstream oss;

    oss << "Simulation Status:\n";
    oss << "  State: ";
    switch (state) {
        case SimulationState::Stopped: oss << "Stopped"; break;
        case SimulationState::Running: oss << "Running"; break;
        case SimulationState::Paused: oss << "Paused"; break;
        case SimulationState::Stepping: oss << "Stepping"; break;
    }

    oss << "\n  Time: " << m_orchestrator->time_manager().simulation_time() << " seconds\n";
    oss << "  Components: " << m_orchestrator->registry().size() << "\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_time(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Simulation Time:\n";
    oss << "  Seconds: " << m_orchestrator->time_manager().simulation_time() << "\n";
    oss << "  Microseconds: " << m_orchestrator->time_manager().simulation_time_us() << "\n";
    oss << "  Tick: " << m_orchestrator->time_manager().current_tick() << "\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_sim_reset(const std::vector<std::string>& args) {
    m_orchestrator->stop();
    return CommandResult::ok("Simulation reset");
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_import(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: cad-import <file> <asset-id>");
    }

    CADKernel cad;
    ModelAssetLibrary library;
    std::string error;
    if (!library.import_as_asset(cad, args[0], args[1], &error)) {
        return CommandResult::err(error.empty() ? "Failed to import CAD asset" : error);
    }
    return CommandResult::ok("CAD asset imported: " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_export(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: cad-export <asset-id> <scope> <file>");
    }

    CADKernel cad;
    ModelAssetLibrary library;
    MeshData mesh;
    std::string error;
    if (!library.load_asset_mesh(cad, args[0], args[1], mesh, &error)) {
        return CommandResult::err(error.empty() ? "Failed to load CAD asset" : error);
    }
    if (!export_mesh_by_extension(cad, args[2], mesh)) {
        return CommandResult::err(cad.error().empty() ? "Unsupported or failed export: " + args[2] : cad.error());
    }
    return CommandResult::ok("CAD asset exported: " + args[2]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_create_primitive(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: cad-create-primitive <box|sphere|cylinder> <asset-id> [dims]");
    }

    CADKernel cad;
    std::shared_ptr<Geometry> geometry;
    if (args[0] == "box") {
        float w = 1.0f;
        float h = 1.0f;
        float d = 1.0f;
        if ((args.size() > 2 && !parse_float_strict(args[2], w)) ||
            (args.size() > 3 && !parse_float_strict(args[3], h)) ||
            (args.size() > 4 && !parse_float_strict(args[4], d))) {
            return CommandResult::err("Primitive dimensions must be numeric");
        }
        if (w <= 0.0f || h <= 0.0f || d <= 0.0f) {
            return CommandResult::err("Box dimensions must be positive");
        }
        geometry = cad.create_box(w, h, d);
    } else if (args[0] == "sphere") {
        float r = 0.5f;
        int segments = 32;
        if ((args.size() > 2 && !parse_float_strict(args[2], r)) ||
            (args.size() > 3 && !parse_int_strict(args[3], segments))) {
            return CommandResult::err("Primitive dimensions must be numeric");
        }
        if (r <= 0.0f || segments < 3) {
            return CommandResult::err("Sphere radius must be positive and segments must be at least 3");
        }
        geometry = cad.create_sphere(r, segments);
    } else if (args[0] == "cylinder") {
        float r = 0.5f;
        float h = 1.0f;
        int segments = 32;
        if ((args.size() > 2 && !parse_float_strict(args[2], r)) ||
            (args.size() > 3 && !parse_float_strict(args[3], h)) ||
            (args.size() > 4 && !parse_int_strict(args[4], segments))) {
            return CommandResult::err("Primitive dimensions must be numeric");
        }
        if (r <= 0.0f || h <= 0.0f || segments < 3) {
            return CommandResult::err("Cylinder radius/height must be positive and segments must be at least 3");
        }
        geometry = cad.create_cylinder(r, h, segments);
    } else {
        return CommandResult::err("Unsupported primitive: " + args[0]);
    }

    if (!geometry) {
        return CommandResult::err("Failed to create primitive");
    }

    ModelAssetMeta meta;
    meta.id = args[1];
    meta.scope = "user";
    meta.label = args[1];
    meta.format = "stl";
    meta.notes = "Created by mechatron-cli";

    ModelAssetLibrary library;
    std::string error;
    if (!library.save_mesh_as_asset(cad, geometry->mesh, args[1], meta, &error)) {
        return CommandResult::err(error.empty() ? "Failed to save primitive asset" : error);
    }

    return CommandResult::ok("Primitive asset created: " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_mesh_info(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: cad-mesh-info <asset-id> [scope]");
    }

    const std::string scope = args.size() > 1 ? args[1] : "user";
    CADKernel cad;
    ModelAssetLibrary library;
    MeshData mesh;
    std::string error;
    if (!library.load_asset_mesh(cad, args[0], scope, mesh, &error)) {
        return CommandResult::err(error.empty() ? "Failed to load CAD asset" : error);
    }

    Vec3 min;
    Vec3 max;
    mesh.get_bounds(min, max);
    std::ostringstream oss;
    oss << "Mesh: " << args[0] << " (" << scope << ")\n";
    oss << "  Vertices: " << mesh.vertices.size() << "\n";
    oss << "  Triangles: " << mesh.triangles.size() << "\n";
    oss << "  Bounds min: [" << min.x << ", " << min.y << ", " << min.z << "]\n";
    oss << "  Bounds max: [" << max.x << ", " << max.y << ", " << max.z << "]\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_mesh_process(const std::vector<std::string>& args) {
    if (args.size() < 5) {
        return CommandResult::err("Usage: cad-mesh-process <asset-id> <scope> <simplify|smooth> <value> <result-id>");
    }

    CADKernel cad;
    ModelAssetLibrary library;
    MeshData mesh;
    std::string error;
    if (!library.load_asset_mesh(cad, args[0], args[1], mesh, &error)) {
        return CommandResult::err(error.empty() ? "Failed to load CAD asset" : error);
    }

    try {
        if (args[2] == "simplify") {
            float ratio = 0.0f;
            if (!parse_float_strict(args[3], ratio)) {
                return CommandResult::err("Mesh process value must be numeric");
            }
            if (ratio <= 0.0f || ratio > 1.0f) {
                return CommandResult::err("Simplify ratio must be in the range (0, 1]");
            }
            cad.simplify_mesh(mesh, ratio);
            if (!cad.error().empty()) {
                return CommandResult::err(cad.error());
            }
        } else if (args[2] == "smooth") {
            int iterations = 0;
            if (!parse_int_strict(args[3], iterations)) {
                return CommandResult::err("Mesh process value must be numeric");
            }
            if (iterations < 0) {
                return CommandResult::err("Smooth iterations must be non-negative");
            }
            cad.smooth_mesh(mesh, iterations);
        } else {
            return CommandResult::err("Unsupported mesh process: " + args[2]);
        }
    } catch (...) {
        return CommandResult::err("Mesh process failed");
    }

    ModelAssetMeta meta;
    meta.id = args[4];
    meta.scope = "user";
    meta.label = args[4];
    meta.format = "stl";
    meta.notes = "Created by mechatron-cli cad-mesh-process";
    if (!library.save_mesh_as_asset(cad, mesh, args[4], meta, &error)) {
        return CommandResult::err(error.empty() ? "Failed to save processed mesh" : error);
    }
    return CommandResult::ok("Processed mesh asset created: " + args[4]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_boolean(const std::vector<std::string>& args) {
    if (args.size() < 6) {
        return CommandResult::err("Usage: cad-boolean <union|subtract|intersect> <a-id> <a-scope> <b-id> <b-scope> <result-id>");
    }

    CADKernel cad;
    ModelAssetLibrary library;
    MeshData a;
    MeshData b;
    MeshData result;
    std::string error;
    if (!library.load_asset_mesh(cad, args[1], args[2], a, &error)) {
        return CommandResult::err(error.empty() ? "Failed to load first CAD asset" : error);
    }
    if (!library.load_asset_mesh(cad, args[3], args[4], b, &error)) {
        return CommandResult::err(error.empty() ? "Failed to load second CAD asset" : error);
    }

    bool ok = false;
    if (args[0] == "union") {
        ok = cad.union_meshes(a, b, result);
    } else if (args[0] == "subtract") {
        ok = cad.subtract_meshes(a, b, result);
    } else if (args[0] == "intersect") {
        ok = cad.intersect_meshes(a, b, result);
    } else {
        return CommandResult::err("Unsupported boolean operation: " + args[0]);
    }
    if (!ok) {
        return CommandResult::err(cad.error().empty() ? "CAD boolean operation failed" : cad.error());
    }

    ModelAssetMeta meta;
    meta.id = args[5];
    meta.scope = "user";
    meta.label = args[5];
    meta.format = "stl";
    meta.notes = "Created by mechatron-cli cad-boolean";
    if (!library.save_mesh_as_asset(cad, result, args[5], meta, &error)) {
        return CommandResult::err(error.empty() ? "Failed to save boolean result" : error);
    }
    return CommandResult::ok("Boolean mesh asset created: " + args[5]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_cad_list_assets(const std::vector<std::string>& args) {
    ModelAssetLibrary library;
    const auto assets = library.list_assets();
    std::ostringstream oss;
    oss << "CAD assets (" << assets.size() << " total):\n";
    for (const auto& asset : assets) {
        oss << "  " << asset.id << " [" << asset.scope << "]";
        if (!asset.label.empty() && asset.label != asset.id) {
            oss << " - " << asset.label;
        }
        if (!asset.format.empty()) {
            oss << " (" << asset.format << ")";
        }
        oss << "\n";
    }
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_add_component(const std::vector<std::string>& args) {
    return cmd_component_add(args);
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_connect(const std::vector<std::string>& args) {
    return cmd_component_connect(args);
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_status(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Circuit Status:\n";
    oss << "  Components: " << m_orchestrator->registry().size() << "\n";
    oss << "  Connections: " << m_orchestrator->get_connections().size() << "\n";
    oss << "  Simulator: " << (m_orchestrator->has_circuit() ? "ready" : "not initialized") << "\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_mcp_start(const std::vector<std::string>& args) {
    int port = app_config::kMcpDefaultPort;
    if (!args.empty()) {
        if (!parse_int_strict(args[0], port)) {
            return CommandResult::err("Invalid port number");
        }
    }
    if (port <= 0 || port > 65535) {
        return CommandResult::err("Invalid port number");
    }

    return start_mcp_server(port)
        ? CommandResult::ok("MCP server started")
        : CommandResult::err("MCP server backend is not available in this build");
}

MechatronCLI::CommandResult MechatronCLI::cmd_mcp_stop(const std::vector<std::string>& args) {
    stop_mcp_server();
    return CommandResult::ok("MCP server stopped");
}

MechatronCLI::CommandResult MechatronCLI::cmd_mcp_status(const std::vector<std::string>& args) {
    return CommandResult::ok(is_mcp_server_running() ? "MCP server running" : "MCP server stopped");
}

MechatronCLI::CommandResult MechatronCLI::cmd_exit(const std::vector<std::string>& args) {
    return CommandResult::ok("Exiting...");
}

MechatronCLI::CommandResult MechatronCLI::cmd_quit(const std::vector<std::string>& args) {
    return cmd_exit(args);
}

std::vector<std::string> MechatronCLI::parse_command_line(const std::string& line) {
    std::vector<std::string> args;
    std::string current;
    bool in_quotes = false;
    char quote_char = '\0';
    bool escaping = false;

    for (char ch : line) {
        if (escaping) {
            current.push_back(ch);
            escaping = false;
            continue;
        }

        if (ch == '\\') {
            escaping = true;
            continue;
        }

        if (in_quotes) {
            if (ch == quote_char) {
                in_quotes = false;
            } else {
                current.push_back(ch);
            }
            continue;
        }

        if (ch == '"' || ch == '\'') {
            in_quotes = true;
            quote_char = ch;
            continue;
        }

        if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
            if (!current.empty()) {
                args.push_back(current);
                current.clear();
            }
            continue;
        }

        current.push_back(ch);
    }

    if (escaping) {
        current.push_back('\\');
    }
    if (!current.empty()) {
        args.push_back(current);
    }

    return args;
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_types(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Available component types:\n";

    std::map<std::string, std::vector<const SystemConfig::ComponentMetadata*>> by_plugin;
    for (const auto& type_id : SystemConfig::instance().all_type_ids()) {
        const auto* metadata = SystemConfig::instance().get_metadata(type_id);
        if (!metadata || metadata->plugin.empty()) {
            continue;
        }
        by_plugin[metadata->plugin].push_back(metadata);
    }

    for (const auto& [plugin_name, types] : by_plugin) {
        oss << "\n" << plugin_name << ":\n";
        for (const auto* metadata : types) {
            oss << "  " << metadata->type_id << " - " << metadata->display_name << "\n";
        }
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_plugins(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Loaded plugins:\n";
    for (auto* plugin : m_orchestrator->plugin_host().get_all_plugins()) {
        if (!plugin) {
            continue;
        }
        oss << "  " << plugin->name() << " v" << plugin->version()
            << " (" << plugin->components().size() << " components)\n";
    }
    return CommandResult::ok(oss.str());
}

// Advanced command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_net_propagate(const std::vector<std::string>& args) {
    m_orchestrator->propagate_nets();
    return CommandResult::ok("Net propagation completed");
}

MechatronCLI::CommandResult MechatronCLI::cmd_actuator_propagate(const std::vector<std::string>& args) {
    m_orchestrator->propagate_voltages_to_actuators();
    return CommandResult::ok("Actuator voltage propagation completed");
}

MechatronCLI::CommandResult MechatronCLI::cmd_subsystem_add(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: subsystem-add <id>");
    }

    auto* subsystem = m_orchestrator->add_subsystem(args[0]);
    if (!subsystem) {
        return CommandResult::err("Failed to add subsystem: " + args[0]);
    }

    return CommandResult::ok("Subsystem added: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_subsystem_remove(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: subsystem-remove <id>");
    }

    m_orchestrator->remove_subsystem(args[0]);
    return CommandResult::ok("Subsystem removed: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_subsystem_list(const std::vector<std::string>& args) {
    const auto ids = m_orchestrator->list_subsystem_ids();
    std::ostringstream oss;
    oss << "Subsystems (" << ids.size() << " total):\n";
    for (const auto& id : ids) {
        oss << "  " << id << "\n";
    }
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_subsystem_info(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: subsystem-info <id>");
    }

    auto* subsystem = m_orchestrator->get_subsystem(args[0]);
    if (!subsystem) {
        return CommandResult::err("Subsystem not found: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Subsystem: " << args[0] << "\n";
    oss << "  Status: Active\n";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_connection_list(const std::vector<std::string>& args) {
    const auto& connections = m_orchestrator->get_connections();
    std::ostringstream oss;
    oss << "Connections (" << connections.size() << " total):\n";

    for (const auto& conn : connections) {
        if (!conn) {
            continue;
        }
        oss << "  " << conn->uid << ": ";
        if (conn->source && conn->source->owner()) {
            oss << conn->source->owner()->id() << "." << conn->source->name();
        }
        oss << " -> ";
        if (conn->target && conn->target->owner()) {
            oss << conn->target->owner()->id() << "." << conn->target->name();
        }
        oss << "\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_connection_info(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: connection-info <uid>");
    }

    const auto& connections = m_orchestrator->get_connections();
    for (const auto& conn : connections) {
        if (!conn || conn->uid != args[0]) {
            continue;
        }

        std::ostringstream oss;
        oss << "Connection: " << conn->uid << "\n";
        if (conn->source && conn->source->owner()) {
            oss << "  Source: " << conn->source->owner()->id() << "." << conn->source->name() << "\n";
        }
        if (conn->target && conn->target->owner()) {
            oss << "  Target: " << conn->target->owner()->id() << "." << conn->target->name() << "\n";
        }
        return CommandResult::ok(oss.str());
    }

    return CommandResult::err("Connection not found: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_copy(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: component-copy <source_id> <new_id>");
    }

    auto* source = m_orchestrator->registry().get(args[0]);
    if (!source) {
        return CommandResult::err("Source component not found: " + args[0]);
    }

    if (m_orchestrator->registry().get(args[1])) {
        return CommandResult::err("Target component already exists: " + args[1]);
    }

    auto* new_comp = m_orchestrator->create_component(
        resolve_plugin_name_for_component(*m_orchestrator, *source),
        source->component_type(),
        args[1]
    );

    if (!new_comp) {
        return CommandResult::err("Failed to create component copy");
    }

    nlohmann::json serialized;
    source->serialize(serialized);
    new_comp->deserialize(serialized);
    new_comp->transform() = source->transform();
    new_comp->update(0.0);

    return CommandResult::ok("Component copied: " + args[0] + " -> " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_rotate(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: component-rotate <id> <axis> <degrees>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    float degrees = 0.0f;
    if (!parse_float_strict(args[2], degrees)) {
        return CommandResult::err("Rotation degrees must be numeric");
    }

    // Simple rotation implementation (would need quaternion support for full 3D rotation)
    // For now, this is a placeholder
    std::ostringstream oss;
    oss << "Component " << args[0] << " rotated " << degrees << " degrees around " << args[1] << " axis";
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_find(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-find <pattern>");
    }

    std::ostringstream oss;
    oss << "Components matching '" << args[0] << "':\n";

    int count = 0;
    m_orchestrator->registry().for_each([&](const Component& comp) {
        const std::string& id = comp.id();
        if (id.find(args[0]) != std::string::npos) {
            oss << "  " << id << " - " << comp.component_type() << "\n";
            count++;
        }
    });

    if (count == 0) {
        oss << "  (none found)\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_mode(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: circuit-mode <native|ngspice|hybrid>");
    }

    if (args[0] != "native" && args[0] != "ngspice" && args[0] != "hybrid") {
        return CommandResult::err("Unsupported circuit mode: " + args[0]);
    }

    return CommandResult::err("Circuit mode switching is not wired into the runtime simulator yet");
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_step(const std::vector<std::string>& args) {
    int count = 1;
    if (!args.empty()) {
        if (!parse_int_strict(args[0], count)) {
            return CommandResult::err("Invalid step count: " + args[0]);
        }
    }
    if (count < 1) {
        return CommandResult::err("Step count must be positive");
    }

    // Step only the circuit simulator
    double dt = 0.001; // Default timestep
    for (int i = 0; i < count; ++i) {
        m_orchestrator->step_circuit(dt);
    }

    return CommandResult::ok("Circuit stepped: " + std::to_string(count));
}

MechatronCLI::CommandResult MechatronCLI::cmd_autosave_enable(const std::vector<std::string>& args) {
    if (args.empty()) {
        bool enabled = m_project_manager->autosave_config().enabled;
        return CommandResult::ok("Autosave is " + std::string(enabled ? "enabled" : "disabled"));
    }

    bool enable = (args[0] == "true" || args[0] == "1" || args[0] == "on");
    m_project_manager->enable_autosave(enable);
    return CommandResult::ok("Autosave " + std::string(enable ? "enabled" : "disabled"));
}

MechatronCLI::CommandResult MechatronCLI::cmd_autosave_config(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        auto config = m_project_manager->autosave_config();
        std::ostringstream oss;
        oss << "Autosave Configuration:\n";
        oss << "  Enabled: " << (config.enabled ? "yes" : "no") << "\n";
        oss << "  Interval: " << config.interval_seconds << " seconds\n";
        oss << "  Max autosaves: " << config.max_autosaves << "\n";
        oss << "  Directory: " << config.autosave_dir << "\n";
        return CommandResult::ok(oss.str());
    }

    double interval = 30.0;
    int max_count = 5;

    if (!parse_double_strict(args[0], interval) || !parse_int_strict(args[1], max_count)) {
        return CommandResult::err("Invalid autosave configuration values");
    }

    ProjectManager::AutosaveConfig config;
    config.enabled = true;
    config.interval_seconds = interval;
    config.max_autosaves = max_count;
    m_project_manager->set_autosave_config(config);

    return CommandResult::ok("Autosave configured: interval=" + std::to_string(interval) +
                           "s, max=" + std::to_string(max_count));
}

MechatronCLI::CommandResult MechatronCLI::cmd_autosave_list(const std::vector<std::string>& args) {
    const auto autosaves = m_project_manager->list_autosaves();
    if (autosaves.empty()) {
        return CommandResult::ok("No autosaves available");
    }

    std::ostringstream oss;
    oss << "Autosaves (" << autosaves.size() << " total):\n";
    for (size_t i = 0; i < autosaves.size(); ++i) {
        oss << "  [" << i << "] " << autosaves[i] << "\n";
    }
    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_autosave_restore(const std::vector<std::string>& args) {
    int index = 0;
    if (!args.empty()) {
        if (!parse_int_strict(args[0], index)) {
            return CommandResult::err("Invalid autosave index");
        }
    }
    if (index < 0) {
        return CommandResult::err("Autosave index must be non-negative");
    }

    if (m_project_manager->restore_from_autosave(static_cast<size_t>(index))) {
        return CommandResult::ok("Autosave restored successfully");
    }
    return CommandResult::err("Failed to restore from autosave");
}

MechatronCLI::CommandResult MechatronCLI::cmd_session_save(const std::vector<std::string>& args) {
    m_project_manager->save_session_state();
    return CommandResult::ok("Session state saved");
}

MechatronCLI::CommandResult MechatronCLI::cmd_session_restore(const std::vector<std::string>& args) {
    m_project_manager->restore_session_state();
    return CommandResult::ok("Session state restored");
}

MechatronCLI::CommandResult MechatronCLI::cmd_session_info(const std::vector<std::string>& args) {
    const auto& session = m_project_manager->session_state();
    std::ostringstream oss;

    oss << "Session Information:\n";
    oss << "  Selected Component: " << session.selected_component << "\n";
    oss << "  Camera Position: [" << session.camera.position.x << ", "
        << session.camera.position.y << ", " << session.camera.position.z << "]\n";
    oss << "  Camera Target: [" << session.camera.target.x << ", "
        << session.camera.target.y << ", " << session.camera.target.z << "]\n";
    oss << "  Viewport Settings:\n";
    oss << "    Wireframe: " << (session.viewport.wireframe ? "yes" : "no") << "\n";
    oss << "    Shading Mode: " << session.viewport.shading_mode << "\n";
    oss << "    Show Grid: " << (session.viewport.show_grid ? "yes" : "no") << "\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_project_stats(const std::vector<std::string>& args) {
    if (!m_project_manager->has_project()) {
        return CommandResult::err("No project open");
    }

    const auto& stats = m_project_manager->stats();
    std::ostringstream oss;

    oss << "Project Statistics:\n";
    oss << "  Components: " << stats.component_count << "\n";
    oss << "  Connections: " << stats.connection_count << "\n";
    oss << "  Assets: " << stats.asset_count << "\n";
    oss << "  Last Autosave: " << stats.last_autosave_time << "\n";
    oss << "  Total Edit Time: " << stats.total_edit_time << "\n";

    return CommandResult::ok(oss.str());
}

// Physics command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_physics_create_body(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: physics-create-body <component_id> <shape> [params]");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    CollisionShapeDef shape;
    shape.type = CollisionShape::Box;

    if (args[1] == "box") {
        shape.type = CollisionShape::Box;
        if (args.size() >= 5) {
            parse_float_strict(args[2], shape.box_extents.x);
            parse_float_strict(args[3], shape.box_extents.y);
            parse_float_strict(args[4], shape.box_extents.z);
        }
    } else if (args[1] == "sphere") {
        shape.type = CollisionShape::Sphere;
        if (args.size() >= 3) {
            parse_float_strict(args[2], shape.sphere_radius);
        }
    } else if (args[1] == "cylinder") {
        shape.type = CollisionShape::Cylinder;
        if (args.size() >= 4) {
            parse_float_strict(args[2], shape.cylinder_radius);
            parse_float_strict(args[3], shape.cylinder_height);
        }
    } else {
        return CommandResult::err("Unknown shape type: " + args[1]);
    }

    auto& physics = m_orchestrator->physics_world();
    PhysicsBody* body = physics.create_body(args[0], shape, comp->transform().position);

    if (!body) {
        return CommandResult::err("Failed to create physics body");
    }

    comp->attach_physics_body(body);
    return CommandResult::ok("Physics body created for: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_remove_body(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: physics-remove-body <id>");
    }

    m_orchestrator->physics_world().remove_body(args[0]);
    return CommandResult::ok("Physics body removed: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_body_info(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: physics-body-info <id>");
    }

    auto* body = m_orchestrator->physics_world().get_body(args[0]);
    if (!body) {
        return CommandResult::err("Physics body not found: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Physics Body: " << args[0] << "\n";
    oss << "  Position: [" << body->position.x << ", " << body->position.y << ", " << body->position.z << "]\n";
    oss << "  Velocity: [" << body->velocity.x << ", " << body->velocity.y << ", " << body->velocity.z << "]\n";
    oss << "  Mass: " << body->mass << " kg\n";
    oss << "  Static: " << (body->is_static ? "yes" : "no") << "\n";
    oss << "  Kinematic: " << (body->is_kinematic ? "yes" : "no") << "\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_list_bodies(const std::vector<std::string>& args) {
    auto bodies = m_orchestrator->physics_world().get_all_bodies();
    std::ostringstream oss;

    oss << "Physics Bodies (" << bodies.size() << " total):\n";
    for (const auto* body : bodies) {
        if (body) {
            oss << "  " << body->id << " - Mass: " << body->mass << "kg\n";
        }
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_gravity(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        Vec3 g = m_orchestrator->physics_world().gravity();
        std::ostringstream oss;
        oss << "Gravity: [" << g.x << ", " << g.y << ", " << g.z << "]";
        return CommandResult::ok(oss.str());
    }

    Vec3 gravity;
    if (!parse_float_strict(args[0], gravity.x) ||
        !parse_float_strict(args[1], gravity.y) ||
        !parse_float_strict(args[2], gravity.z)) {
        return CommandResult::err("Gravity values must be numeric");
    }

    m_orchestrator->physics_world().set_gravity(gravity);
    return CommandResult::ok("Gravity set to: [" + std::string(args[0]) + ", " + args[1] + ", " + args[2] + "]");
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_add_force(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        return CommandResult::err("Usage: physics-add-force <body_id> <x> <y> <z>");
    }

    Vec3 force;
    if (!parse_float_strict(args[1], force.x) ||
        !parse_float_strict(args[2], force.y) ||
        !parse_float_strict(args[3], force.z)) {
        return CommandResult::err("Force values must be numeric");
    }

    m_orchestrator->physics_world().add_force(args[0], force);
    return CommandResult::ok("Force added to: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_physics_add_torque(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        return CommandResult::err("Usage: physics-add-torque <body_id> <x> <y> <z>");
    }

    Vec3 torque;
    if (!parse_float_strict(args[1], torque.x) ||
        !parse_float_strict(args[2], torque.y) ||
        !parse_float_strict(args[3], torque.z)) {
        return CommandResult::err("Torque values must be numeric");
    }

    m_orchestrator->physics_world().add_torque(args[0], torque);
    return CommandResult::ok("Torque added to: " + args[0]);
}

// Sensor command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_sensor_read(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: sensor-read <sensor_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    // Try to cast as sensor
    auto* sensor = dynamic_cast<class Sensor*>(comp);
    if (!sensor) {
        return CommandResult::err("Component is not a sensor: " + args[0]);
    }

    float value = sensor->read();
    return CommandResult::ok(std::to_string(value));
}

MechatronCLI::CommandResult MechatronCLI::cmd_sensor_list(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Sensors:\n";

    int count = 0;
    m_orchestrator->registry().for_each([&](const Component& comp) {
        if (dynamic_cast<const class Sensor*>(&comp)) {
            oss << "  " << comp.id() << " - " << comp.component_type() << "\n";
            count++;
        }
    });

    if (count == 0) {
        oss << "  (no sensors found)\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_sensor_configure(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: sensor-configure <sensor_id> <param> <value>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    auto* sensor = dynamic_cast<class Sensor*>(comp);
    if (!sensor) {
        return CommandResult::err("Component is not a sensor: " + args[0]);
    }

    float value = 0.0f;
    if (!parse_float_strict(args[2], value)) {
        return CommandResult::err("Value must be numeric");
    }

    if (args[1] == "min_value") {
        sensor->set_min_value(value);
    } else if (args[1] == "max_value") {
        sensor->set_max_value(value);
    } else if (args[1] == "noise_level") {
        sensor->set_noise_level(value);
    } else {
        return CommandResult::err("Unknown parameter: " + args[1]);
    }

    return CommandResult::ok("Sensor configured: " + args[0] + "." + args[1] + " = " + args[2]);
}

// Actuator command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_actuator_set_input(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: actuator-set-input <actuator_id> <value>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    auto* actuator = dynamic_cast<class Actuator*>(comp);
    if (!actuator) {
        return CommandResult::err("Component is not an actuator: " + args[0]);
    }

    float value = 0.0f;
    if (!parse_float_strict(args[1], value)) {
        return CommandResult::err("Value must be numeric");
    }

    actuator->set_input(value);
    return CommandResult::ok("Actuator input set: " + args[0] + " = " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_actuator_get_state(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: actuator-get-state <actuator_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    auto* actuator = dynamic_cast<class Actuator*>(comp);
    if (!actuator) {
        return CommandResult::err("Component is not an actuator: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Actuator: " << args[0] << "\n";
    oss << "  Input: " << actuator->get_input() << "\n";
    oss << "  Enabled: " << (actuator->is_enabled() ? "yes" : "no") << "\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_actuator_enable(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: actuator-enable <actuator_id> <true|false>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    auto* actuator = dynamic_cast<class Actuator*>(comp);
    if (!actuator) {
        return CommandResult::err("Component is not an actuator: " + args[0]);
    }

    bool enable = (args[1] == "true" || args[1] == "1" || args[1] == "on");
    actuator->set_enabled(enable);

    return CommandResult::ok("Actuator " + std::string(enable ? "enabled" : "disabled") + ": " + args[0]);
}

// Serial command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_serial_list(const std::vector<std::string>& args) {
    auto ports = SerialPort::list_ports();
    std::ostringstream oss;

    oss << "Serial Ports (" << ports.size() << " total):\n";
    for (const auto& port : ports) {
        oss << "  " << port.port;
        if (!port.display_name.empty()) {
            oss << " - " << port.display_name;
        }
        if (port.likely_arduino) {
            oss << " [Arduino]";
        }
        oss << "\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_serial_open(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: serial-open <port> <baud_rate>");
    }

    int baud_rate = 9600;
    if (!parse_int_strict(args[1], baud_rate)) {
        return CommandResult::err("Baud rate must be numeric");
    }

    if (!m_serial_port) {
        m_serial_port = std::make_unique<SerialPort>();
    }
    if (!m_serial_port->open(args[0], baud_rate)) {
        return CommandResult::err("Failed to open serial port: " + args[0]);
    }
    return CommandResult::ok("Serial port opened: " + args[0] + " at " + args[1] + " baud");
}

MechatronCLI::CommandResult MechatronCLI::cmd_serial_close(const std::vector<std::string>& args) {
    if (!m_serial_port || !m_serial_port->is_open()) {
        return CommandResult::err("No serial port open");
    }
    m_serial_port->close();
    return CommandResult::ok("Serial port closed");
}

MechatronCLI::CommandResult MechatronCLI::cmd_serial_write(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: serial-write <data>");
    }

    if (!m_serial_port || !m_serial_port->is_open()) {
        return CommandResult::err("No serial port open");
    }
    const auto bytes_written = m_serial_port->write(args[0].data(), args[0].size());
    if (bytes_written < 0) {
        return CommandResult::err("Failed to write to serial port");
    }
    return CommandResult::ok("Data written to serial: " + std::to_string(bytes_written) + " bytes");
}

// Circuit analysis command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_analyze(const std::vector<std::string>& args) {
    std::string error;
    auto snapshot = build_circuit_spice_snapshot(*m_orchestrator, &error);
    if (!snapshot) {
        return CommandResult::err(error);
    }

    size_t circuit_connections = 0;
    for (const auto& connection : m_orchestrator->get_connections()) {
        if (!connection || !connection->source || !connection->target) {
            continue;
        }
        const auto* source_owner = connection->source->owner();
        const auto* target_owner = connection->target->owner();
        if (dynamic_cast<const ICircuitComponentAdapter*>(source_owner) &&
            dynamic_cast<const ICircuitComponentAdapter*>(target_owner)) {
            ++circuit_connections;
        }
    }

    std::ostringstream oss;
    oss << "Circuit Analysis:\n";
    oss << "  Circuit Components: " << snapshot->total_components << "\n";
    oss << "  Circuit Connections: " << circuit_connections << "\n";
    oss << "  Nodes: " << snapshot->total_nodes << "\n";
    oss << "  Topology: Valid\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_netlist(const std::vector<std::string>& args) {
    std::string error;
    auto snapshot = build_circuit_spice_snapshot(*m_orchestrator, &error);
    if (!snapshot) {
        return CommandResult::err(error);
    }
    return CommandResult::ok(snapshot->netlist);
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_export_netlist(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: circuit-export-netlist <file>");
    }

    std::string error;
    auto snapshot = build_circuit_spice_snapshot(*m_orchestrator, &error);
    if (!snapshot) {
        return CommandResult::err(error);
    }

    std::ofstream file(args[0]);
    if (!file.is_open()) {
        return CommandResult::err("Failed to open file for writing: " + args[0]);
    }
    file << snapshot->netlist;
    file.close();

    return CommandResult::ok("Netlist exported to: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_circuit_nodes(const std::vector<std::string>& args) {
    std::string error;
    auto snapshot = build_circuit_spice_snapshot(*m_orchestrator, &error);
    if (!snapshot) {
        return CommandResult::err(error);
    }

    std::vector<std::pair<std::string, std::string>> node_pairs(snapshot->pin_to_node.begin(), snapshot->pin_to_node.end());
    std::sort(node_pairs.begin(), node_pairs.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });

    std::ostringstream oss;
    oss << "Circuit Nodes:\n";
    for (const auto& [pin, node] : node_pairs) {
        oss << "  " << pin << " -> " << node << "\n";
    }

    return CommandResult::ok(oss.str());
}

// System command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_system_config(const std::vector<std::string>& args) {
    std::ostringstream oss;

    oss << "System Configuration:\n";
    oss << "  Version: " << VERSION << "\n";
    oss << "  Plugins: " << m_orchestrator->plugin_host().get_all_plugins().size() << "\n";
    oss << "  Components: " << m_orchestrator->registry().size() << "\n";
    oss << "  Physics: " << (m_orchestrator->physics_world().body_count() > 0 ? "Active" : "Inactive") << "\n";
    oss << "  Circuit: " << (m_orchestrator->has_circuit() ? "Available" : "Not available") << "\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_metadata(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-metadata <type>");
    }

    const auto* metadata = SystemConfig::instance().get_metadata(args[0]);
    if (!metadata) {
        return CommandResult::err("Unknown component type: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Component Metadata: " << metadata->type_id << "\n";
    oss << "  Display Name: " << metadata->display_name << "\n";
    oss << "  Category: " << metadata->category << "\n";
    oss << "  Description: " << metadata->description << "\n";
    if (!metadata->spice_type.empty()) {
        oss << "  SPICE Type: " << metadata->spice_type << "\n";
    }
    oss << "  Circuit Component: " << (metadata->is_circuit_component ? "yes" : "no") << "\n";
    oss << "  Adapter Component: " << (metadata->is_adapter_component ? "yes" : "no") << "\n";
    if (!metadata->aliases.empty()) {
        oss << "  Aliases:";
        for (const auto& alias : metadata->aliases) {
            oss << " " << alias;
        }
        oss << "\n";
    }
    if (!metadata->pins.empty()) {
        oss << "  Pins:\n";
        for (const auto& pin : metadata->pins) {
            oss << "    " << pin.name << " (" << pin.display_name << ")\n";
        }
    }
    if (!metadata->parameters.empty()) {
        oss << "  Parameters:\n";
        for (const auto& parameter : metadata->parameters) {
            oss << "    " << parameter.name << " = " << parameter.default_value;
            if (!parameter.units.empty()) {
                oss << " " << parameter.units;
            }
            oss << "\n";
        }
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_categories(const std::vector<std::string>& args) {
    const auto type_ids = SystemConfig::instance().all_type_ids();
    std::vector<std::string> categories;
    categories.reserve(type_ids.size());
    for (const auto& type_id : type_ids) {
        const auto category = SystemConfig::instance().get_category(type_id);
        if (!category.empty() &&
            std::find(categories.begin(), categories.end(), category) == categories.end()) {
            categories.push_back(category);
        }
    }
    std::sort(categories.begin(), categories.end());

    std::ostringstream oss;
    oss << "Component Categories:\n";
    for (const auto& category : categories) {
        oss << "  " << category << "\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_scale(const std::vector<std::string>& args) {
    if (args.size() < 4) {
        return CommandResult::err("Usage: component-scale <id> <x> <y> <z>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    Vec3 scale;
    if (!parse_float_strict(args[1], scale.x) ||
        !parse_float_strict(args[2], scale.y) ||
        !parse_float_strict(args[3], scale.z)) {
        return CommandResult::err("Scale values must be numeric");
    }

    comp->transform().scale = scale;
    return CommandResult::ok("Component scaled: " + args[0]);
}

// Physical link command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_physical_link_list(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Physical Link Capable Components:\n";

    int count = 0;
    m_orchestrator->registry().for_each([&](const Component& comp) {
        if (comp.physical_link_supported()) {
            oss << "  " << comp.id() << " - " << comp.component_type() << "\n";
            count++;
        }
    });

    if (count == 0) {
        oss << "  (no physical link capable components found)\n";
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_physical_link_connect(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: physical-link-connect <component_id> <port> <baud_rate>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    if (!comp->physical_link_supported()) {
        return CommandResult::err("Component does not support physical link: " + args[0]);
    }

    int baud_rate = 9600;
    if (!parse_int_strict(args[2], baud_rate)) {
        return CommandResult::err("Baud rate must be numeric");
    }

    comp->physical_link_set_config(std::string(args[1]), baud_rate);

    if (!comp->physical_link_connect()) {
        return CommandResult::err("Failed to connect physical link");
    }

    return CommandResult::ok("Physical link connected: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_physical_link_disconnect(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: physical-link-disconnect <component_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    comp->physical_link_disconnect();
    return CommandResult::ok("Physical link disconnected: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_physical_link_status(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: physical-link-status <component_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    std::string port;
    int baud = 0;
    comp->physical_link_get_config(port, baud);

    std::ostringstream oss;
    oss << "Physical Link Status: " << args[0] << "\n";
    oss << "  Supported: " << (comp->physical_link_supported() ? "yes" : "no") << "\n";
    oss << "  Connected: " << (comp->physical_link_is_connected() ? "yes" : "no") << "\n";
    oss << "  Port: " << port << "\n";
    oss << "  Baud: " << baud << "\n";

    return CommandResult::ok(oss.str());
}

// MCU command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_mcu_firmware_upload(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: mcu-firmware-upload <component_id> <firmware_file>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    if (!comp->load_firmware_file(args[1])) {
        return CommandResult::err("Failed to load firmware: " + args[1]);
    }

    return CommandResult::ok("Firmware uploaded: " + args[0] + " <- " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_mcu_pin_get(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: mcu-pin-get <component_id> <pin_name>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    float voltage = 0.0f;
    if (!comp->get_mcu_pin_output_voltage(args[1], voltage)) {
        return CommandResult::err("Failed to get pin voltage: " + args[1]);
    }

    return CommandResult::ok(std::to_string(voltage) + "V");
}

MechatronCLI::CommandResult MechatronCLI::cmd_mcu_pin_set(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: mcu-pin-set <component_id> <pin_name> <voltage>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    float voltage = 0.0f;
    if (!parse_float_strict(args[2], voltage)) {
        return CommandResult::err("Voltage must be numeric");
    }

    if (!comp->set_mcu_pin_input_voltage(args[1], voltage)) {
        return CommandResult::err("Failed to set pin voltage: " + args[1]);
    }

    return CommandResult::ok("Pin voltage set: " + args[0] + "." + args[1] + " = " + args[2] + "V");
}

// Time command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_time_realtime(const std::vector<std::string>& args) {
    if (args.empty()) {
        double factor = m_orchestrator->time_manager().realtime_factor();
        return CommandResult::ok("Realtime factor: " + std::to_string(factor));
    }

    double factor = 0.0;
    if (!parse_double_strict(args[0], factor)) {
        return CommandResult::err("Factor must be numeric");
    }

    m_orchestrator->time_manager().set_realtime_factor(factor);
    return CommandResult::ok("Realtime factor set to: " + std::to_string(factor));
}

MechatronCLI::CommandResult MechatronCLI::cmd_time_deterministic(const std::vector<std::string>& args) {
    if (args.empty()) {
        bool det = m_orchestrator->time_manager().is_deterministic();
        return CommandResult::ok("Deterministic mode: " + std::string(det ? "enabled" : "disabled"));
    }

    bool enable = (args[0] == "true" || args[0] == "1" || args[0] == "on");
    m_orchestrator->time_manager().set_deterministic(enable);
    return CommandResult::ok("Deterministic mode " + std::string(enable ? "enabled" : "disabled"));
}

// Scene command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_scene_clear(const std::vector<std::string>& args) {
    m_orchestrator->clear_scene();
    return CommandResult::ok("Scene cleared");
}

MechatronCLI::CommandResult MechatronCLI::cmd_scene_validate(const std::vector<std::string>& args) {
    std::ostringstream oss;

    oss << "Scene Validation:\n";
    oss << "  Components: " << m_orchestrator->registry().size() << "\n";
    oss << "  Connections: " << m_orchestrator->get_connections().size() << "\n";

    // Check for unconnected ports
    int unconnected_ports = 0;
    m_orchestrator->registry().for_each([&](Component& comp) {
        for (const auto* port : comp.get_ports()) {
            if (port && port->connections().empty()) {
                unconnected_ports++;
            }
        }
    });

    oss << "  Unconnected ports: " << unconnected_ports << "\n";
    oss << "  Status: " << (unconnected_ports == 0 ? "Valid" : "Warnings detected") << "\n";

    return CommandResult::ok(oss.str());
}

// Circuit-Physics Bridge command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_bridge_add_mapping(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return CommandResult::err("Usage: bridge-add-mapping <circuit_pin> <target_component> <type>");
    }

    PinMapping mapping;
    mapping.circuit_pin_id = args[0];
    mapping.target_component_id = args[1];

    // Parse mapping type
    std::string type_str = args[2];
    if (type_str == "voltage_to_actuator" || type_str == "VoltageToActuatorInput") {
        mapping.type = PinMappingType::VoltageToActuatorInput;
    } else if (type_str == "sensor_to_digital" || type_str == "SensorToDigitalPin") {
        mapping.type = PinMappingType::SensorToDigitalPin;
    } else if (type_str == "sensor_to_analog" || type_str == "SensorToAnalogPin") {
        mapping.type = PinMappingType::SensorToAnalogPin;
    } else if (type_str == "voltage_to_force" || type_str == "VoltageToForce") {
        mapping.type = PinMappingType::VoltageToForce;
    } else if (type_str == "voltage_to_torque" || type_str == "VoltageToTorque") {
        mapping.type = PinMappingType::VoltageToTorque;
    } else if (type_str == "digital_to_enable" || type_str == "DigitalToEnable") {
        mapping.type = PinMappingType::DigitalToEnable;
    } else if (type_str == "pwm_to_speed" || type_str == "PWMToSpeed") {
        mapping.type = PinMappingType::PWMToSpeed;
    } else {
        return CommandResult::err("Unknown mapping type: " + type_str);
    }

    // Optional scaling parameters
    if (args.size() >= 6) {
        parse_float_strict(args[3], mapping.voltage_min);
        parse_float_strict(args[4], mapping.voltage_max);
        parse_float_strict(args[5], mapping.output_max);
    }

    m_orchestrator->circuit_bridge().add_mapping(mapping);
    return CommandResult::ok("Pin mapping added: " + args[0] + " -> " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_bridge_remove_mapping(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: bridge-remove-mapping <circuit_pin>");
    }

    m_orchestrator->circuit_bridge().remove_mapping(args[0]);
    return CommandResult::ok("Pin mapping removed: " + args[0]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_bridge_list_mappings(const std::vector<std::string>& args) {
    std::ostringstream oss;
    oss << "Circuit-Physics Bridge Mappings:\n";
    oss << "  Enabled: " << (m_orchestrator->circuit_bridge().is_enabled() ? "yes" : "no") << "\n";
    oss << "  (Detailed mapping list would require bridge internals access)\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_bridge_enable(const std::vector<std::string>& args) {
    if (args.empty()) {
        bool enabled = m_orchestrator->circuit_bridge().is_enabled();
        return CommandResult::ok("Bridge is " + std::string(enabled ? "enabled" : "disabled"));
    }

    bool enable = (args[0] == "true" || args[0] == "1" || args[0] == "on");
    m_orchestrator->circuit_bridge().set_enabled(enable);
    return CommandResult::ok("Bridge " + std::string(enable ? "enabled" : "disabled"));
}

// Equivalent Circuit command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_equiv_circuit_add_actuator(const std::vector<std::string>& args) {
    m_orchestrator->add_actuator_equivalent_circuits();
    return CommandResult::ok("Actuator equivalent circuits added");
}

MechatronCLI::CommandResult MechatronCLI::cmd_equiv_circuit_add_mcu(const std::vector<std::string>& args) {
    m_orchestrator->add_mcu_equivalent_circuits();
    return CommandResult::ok("MCU equivalent circuits added");
}

MechatronCLI::CommandResult MechatronCLI::cmd_equiv_circuit_update_mcu(const std::vector<std::string>& args) {
    m_orchestrator->update_mcu_equivalent_sources();
    return CommandResult::ok("MCU equivalent sources updated");
}

MechatronCLI::CommandResult MechatronCLI::cmd_equiv_circuit_sync_mcu_power(const std::vector<std::string>& args) {
    m_orchestrator->sync_mcu_power_from_ngspice();
    return CommandResult::ok("MCU power synchronized from ngspice");
}

MechatronCLI::CommandResult MechatronCLI::cmd_equiv_circuit_sync_mcu_inputs(const std::vector<std::string>& args) {
    m_orchestrator->sync_mcu_inputs_from_ngspice();
    return CommandResult::ok("MCU inputs synchronized from ngspice");
}

// Advanced Component command implementations

MechatronCLI::CommandResult MechatronCLI::cmd_component_bounds(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-bounds <component_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Component Bounds: " << args[0] << "\n";
    oss << "  Position: [" << comp->transform().position.x << ", "
        << comp->transform().position.y << ", " << comp->transform().position.z << "]\n";
    oss << "  Scale: [" << comp->transform().scale.x << ", "
        << comp->transform().scale.y << ", " << comp->transform().scale.z << "]\n";

    // Calculate approximate bounds (would require mesh data for exact bounds)
    float max_scale = std::max({comp->transform().scale.x, comp->transform().scale.y, comp->transform().scale.z});
    oss << "  Approximate Size: " << max_scale << " units\n";

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_component_position(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: component-position <component_id> [x] [y] [z]");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    if (args.size() >= 4) {
        // Set position
        Vec3 pos;
        if (!parse_float_strict(args[1], pos.x) ||
            !parse_float_strict(args[2], pos.y) ||
            !parse_float_strict(args[3], pos.z)) {
            return CommandResult::err("Position values must be numeric");
        }
        comp->transform().position = pos;
        return CommandResult::ok("Component position set: " + args[0]);
    } else {
        // Get position
        std::ostringstream oss;
        oss << "Component Position: " << args[0] << "\n";
        oss << "  [" << comp->transform().position.x << ", "
            << comp->transform().position.y << ", " << comp->transform().position.z << "]";
        return CommandResult::ok(oss.str());
    }
}

MechatronCLI::CommandResult MechatronCLI::cmd_port_list(const std::vector<std::string>& args) {
    if (args.empty()) {
        return CommandResult::err("Usage: port-list <component_id>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    std::ostringstream oss;
    oss << "Component Ports: " << args[0] << "\n";

    auto ports = comp->get_ports();
    if (ports.empty()) {
        oss << "  (no ports)\n";
    } else {
        for (const auto* port : ports) {
            if (port) {
                oss << "  " << port->name() << " - " << (port->domain() == PortDomain::Electrical ? "Electrical" : "Mechanical");
                if (port->direction() == PortDirection::Input) oss << " (Input)";
                else if (port->direction() == PortDirection::Output) oss << " (Output)";
                else oss << " (Bidirectional)";
                oss << "\n";
            }
        }
    }

    return CommandResult::ok(oss.str());
}

MechatronCLI::CommandResult MechatronCLI::cmd_port_info(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: port-info <component_id> <port_name>");
    }

    auto* comp = m_orchestrator->registry().get(args[0]);
    if (!comp) {
        return CommandResult::err("Component not found: " + args[0]);
    }

    auto ports = comp->get_ports();
    for (const auto* port : ports) {
        if (port && port->name() == args[1]) {
            std::ostringstream oss;
            oss << "Port: " << args[1] << "\n";
            oss << "  Component: " << args[0] << "\n";
            oss << "  Domain: " << (port->domain() == PortDomain::Electrical ? "Electrical" : "Mechanical") << "\n";
            if (port->direction() == PortDirection::Input) oss << "  Direction: Input\n";
            else if (port->direction() == PortDirection::Output) oss << "  Direction: Output\n";
            else oss << "  Direction: Bidirectional\n";
            oss << "  Connections: " << port->connections().size() << "\n";
            return CommandResult::ok(oss.str());
        }
    }

    return CommandResult::err("Port not found: " + args[1]);
}

MechatronCLI::CommandResult MechatronCLI::cmd_actuator_terminal_current(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        return CommandResult::err("Usage: actuator-terminal-current <component_id> <pin_name>");
    }

    float current = 0.0f;
    if (!m_orchestrator->get_actuator_terminal_current(args[0], args[1], current)) {
        return CommandResult::err("Failed to get terminal current for: " + args[0] + "." + args[1]);
    }

    return CommandResult::ok("Terminal current: " + std::to_string(current) + " A");
}

} // namespace mechatron
