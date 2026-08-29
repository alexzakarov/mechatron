#pragma once

#include "core/SimulationOrchestrator.hpp"
#include "core/ProjectManager.hpp"
#include "core/AppConfig.hpp"
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <unordered_map>

// Forward declarations to avoid circular dependencies
namespace mechatron {
    class MechatronCLI;
    class SerialPort;
}

namespace mechatron {

/**
 * MechatronCLI - Command Line Interface for Mechatron Simulation Engine
 *
 * This class provides comprehensive CLI access to all mechatron functionality:
 * - Project management (create, open, save, close)
 * - Component management (add, remove, list, configure)
 * - Simulation control (start, stop, pause, step, status)
 * - CAD operations (import, export, mesh operations)
 * - Circuit operations (netlist management, simulation)
 * - MCP (Model Context Protocol) server for AI integration
 */
class MechatronCLI {
public:
    // Command result structure
    struct CommandResult {
        bool success = false;
        std::string output;
        std::string error;
        int exit_code = 0;

        static CommandResult ok(const std::string& msg = "") {
            return {true, msg, "", 0};
        }

        static CommandResult err(const std::string& msg, int code = 1) {
            return {false, "", msg, code};
        }
    };

    // Command handler function type
    using CommandHandler = std::function<CommandResult(const std::vector<std::string>&)>;

    MechatronCLI();
    ~MechatronCLI();

    // Initialize the CLI system
    bool initialize();
    void shutdown();

    // Main entry points
    CommandResult execute_command(const std::string& command_line);
    CommandResult execute_command(const std::vector<std::string>& args);

    // Interactive mode
    void run_interactive();
    void run_repl();

    // Script execution
    CommandResult execute_script(const std::string& script_file);
    CommandResult execute_commands(const std::vector<std::string>& commands);

    // MCP Server integration
    bool start_mcp_server(int port = app_config::kMcpDefaultPort);
    void stop_mcp_server();
    bool is_mcp_server_running() const;

    // Core systems access (for testing and advanced usage)
    SimulationOrchestrator& orchestrator() { return *m_orchestrator; }
    ProjectManager& project_manager() { return *m_project_manager; }

    // Command registration system (for extensibility)
    void register_command(const std::string& name, const std::string& description, CommandHandler handler);
    void register_alias(const std::string& alias, const std::string& command);

    // Help and info
    std::string get_help() const;
    std::string get_command_help(const std::string& command) const;
    std::vector<std::string> get_available_commands() const;

private:
    // Core systems
    std::unique_ptr<SimulationOrchestrator> m_orchestrator;
    std::unique_ptr<ProjectManager> m_project_manager;

    // Command registry
    struct CommandInfo {
        std::string name;
        std::string description;
        std::string usage;
        CommandHandler handler;
    };
    std::unordered_map<std::string, CommandInfo> m_commands;
    std::unordered_map<std::string, std::string> m_aliases;

    // MCP Server (using raw pointer to avoid incomplete type issues)
    void* m_mcp_server = nullptr;
    bool m_mcp_server_running = false;
    std::unique_ptr<SerialPort> m_serial_port;

    // State
    bool m_initialized = false;
    bool m_interactive_mode = false;
    std::string m_current_script_file;

    // Built-in command handlers
    void register_builtin_commands();

    // Project commands
    CommandResult cmd_project_new(const std::vector<std::string>& args);
    CommandResult cmd_project_open(const std::vector<std::string>& args);
    CommandResult cmd_project_save(const std::vector<std::string>& args);
    CommandResult cmd_project_save_as(const std::vector<std::string>& args);
    CommandResult cmd_project_close(const std::vector<std::string>& args);
    CommandResult cmd_project_info(const std::vector<std::string>& args);
    CommandResult cmd_project_validate(const std::vector<std::string>& args);
    CommandResult cmd_project_export(const std::vector<std::string>& args);
    CommandResult cmd_project_import(const std::vector<std::string>& args);
    CommandResult cmd_project_templates(const std::vector<std::string>& args);

    // Component commands
    CommandResult cmd_component_add(const std::vector<std::string>& args);
    CommandResult cmd_component_remove(const std::vector<std::string>& args);
    CommandResult cmd_component_list(const std::vector<std::string>& args);
    CommandResult cmd_component_info(const std::vector<std::string>& args);
    CommandResult cmd_component_set(const std::vector<std::string>& args);
    CommandResult cmd_component_get(const std::vector<std::string>& args);
    CommandResult cmd_component_select(const std::vector<std::string>& args);
    CommandResult cmd_component_transform(const std::vector<std::string>& args);
    CommandResult cmd_component_connect(const std::vector<std::string>& args);
    CommandResult cmd_component_disconnect(const std::vector<std::string>& args);
    CommandResult cmd_component_types(const std::vector<std::string>& args);
    CommandResult cmd_component_plugins(const std::vector<std::string>& args);

    // Simulation commands
    CommandResult cmd_sim_start(const std::vector<std::string>& args);
    CommandResult cmd_sim_stop(const std::vector<std::string>& args);
    CommandResult cmd_sim_pause(const std::vector<std::string>& args);
    CommandResult cmd_sim_resume(const std::vector<std::string>& args);
    CommandResult cmd_sim_step(const std::vector<std::string>& args);
    CommandResult cmd_sim_status(const std::vector<std::string>& args);
    CommandResult cmd_sim_time(const std::vector<std::string>& args);
    CommandResult cmd_sim_reset(const std::vector<std::string>& args);

    // CAD commands
    CommandResult cmd_cad_import(const std::vector<std::string>& args);
    CommandResult cmd_cad_export(const std::vector<std::string>& args);
    CommandResult cmd_cad_create_primitive(const std::vector<std::string>& args);
    CommandResult cmd_cad_mesh_info(const std::vector<std::string>& args);
    CommandResult cmd_cad_mesh_process(const std::vector<std::string>& args);
    CommandResult cmd_cad_boolean(const std::vector<std::string>& args);
    CommandResult cmd_cad_list_assets(const std::vector<std::string>& args);

    // Circuit commands
    CommandResult cmd_circuit_add_component(const std::vector<std::string>& args);
    CommandResult cmd_circuit_connect(const std::vector<std::string>& args);
    CommandResult cmd_circuit_status(const std::vector<std::string>& args);

    // Utility commands
    CommandResult cmd_help(const std::vector<std::string>& args);
    CommandResult cmd_version(const std::vector<std::string>& args);
    CommandResult cmd_echo(const std::vector<std::string>& args);
    CommandResult cmd_sleep(const std::vector<std::string>& args);
    CommandResult cmd_exit(const std::vector<std::string>& args);
    CommandResult cmd_quit(const std::vector<std::string>& args);

    // MCP commands
    CommandResult cmd_mcp_start(const std::vector<std::string>& args);
    CommandResult cmd_mcp_stop(const std::vector<std::string>& args);
    CommandResult cmd_mcp_status(const std::vector<std::string>& args);

    // Advanced commands
    CommandResult cmd_net_propagate(const std::vector<std::string>& args);
    CommandResult cmd_actuator_propagate(const std::vector<std::string>& args);
    CommandResult cmd_subsystem_add(const std::vector<std::string>& args);
    CommandResult cmd_subsystem_remove(const std::vector<std::string>& args);
    CommandResult cmd_subsystem_list(const std::vector<std::string>& args);
    CommandResult cmd_subsystem_info(const std::vector<std::string>& args);
    CommandResult cmd_connection_list(const std::vector<std::string>& args);
    CommandResult cmd_connection_info(const std::vector<std::string>& args);
    CommandResult cmd_component_copy(const std::vector<std::string>& args);
    CommandResult cmd_component_rotate(const std::vector<std::string>& args);
    CommandResult cmd_component_find(const std::vector<std::string>& args);
    CommandResult cmd_circuit_mode(const std::vector<std::string>& args);
    CommandResult cmd_circuit_step(const std::vector<std::string>& args);
    CommandResult cmd_autosave_enable(const std::vector<std::string>& args);
    CommandResult cmd_autosave_config(const std::vector<std::string>& args);
    CommandResult cmd_autosave_list(const std::vector<std::string>& args);
    CommandResult cmd_autosave_restore(const std::vector<std::string>& args);
    CommandResult cmd_session_save(const std::vector<std::string>& args);
    CommandResult cmd_session_restore(const std::vector<std::string>& args);
    CommandResult cmd_session_info(const std::vector<std::string>& args);
    CommandResult cmd_project_stats(const std::vector<std::string>& args);

    // Physics commands
    CommandResult cmd_physics_create_body(const std::vector<std::string>& args);
    CommandResult cmd_physics_remove_body(const std::vector<std::string>& args);
    CommandResult cmd_physics_body_info(const std::vector<std::string>& args);
    CommandResult cmd_physics_list_bodies(const std::vector<std::string>& args);
    CommandResult cmd_physics_gravity(const std::vector<std::string>& args);
    CommandResult cmd_physics_add_force(const std::vector<std::string>& args);
    CommandResult cmd_physics_add_torque(const std::vector<std::string>& args);

    // Sensor commands
    CommandResult cmd_sensor_read(const std::vector<std::string>& args);
    CommandResult cmd_sensor_list(const std::vector<std::string>& args);
    CommandResult cmd_sensor_configure(const std::vector<std::string>& args);

    // Actuator commands
    CommandResult cmd_actuator_set_input(const std::vector<std::string>& args);
    CommandResult cmd_actuator_get_state(const std::vector<std::string>& args);
    CommandResult cmd_actuator_enable(const std::vector<std::string>& args);

    // Serial commands
    CommandResult cmd_serial_list(const std::vector<std::string>& args);
    CommandResult cmd_serial_open(const std::vector<std::string>& args);
    CommandResult cmd_serial_close(const std::vector<std::string>& args);
    CommandResult cmd_serial_write(const std::vector<std::string>& args);

    // Circuit analysis commands
    CommandResult cmd_circuit_analyze(const std::vector<std::string>& args);
    CommandResult cmd_circuit_netlist(const std::vector<std::string>& args);
    CommandResult cmd_circuit_export_netlist(const std::vector<std::string>& args);
    CommandResult cmd_circuit_nodes(const std::vector<std::string>& args);

    // System commands
    CommandResult cmd_system_config(const std::vector<std::string>& args);
    CommandResult cmd_component_metadata(const std::vector<std::string>& args);
    CommandResult cmd_component_categories(const std::vector<std::string>& args);
    CommandResult cmd_component_scale(const std::vector<std::string>& args);

    // Physical link commands
    CommandResult cmd_physical_link_list(const std::vector<std::string>& args);
    CommandResult cmd_physical_link_connect(const std::vector<std::string>& args);
    CommandResult cmd_physical_link_disconnect(const std::vector<std::string>& args);
    CommandResult cmd_physical_link_status(const std::vector<std::string>& args);

    // MCU commands
    CommandResult cmd_mcu_firmware_upload(const std::vector<std::string>& args);
    CommandResult cmd_mcu_pin_get(const std::vector<std::string>& args);
    CommandResult cmd_mcu_pin_set(const std::vector<std::string>& args);

    // Time commands
    CommandResult cmd_time_realtime(const std::vector<std::string>& args);
    CommandResult cmd_time_deterministic(const std::vector<std::string>& args);

    // Scene commands
    CommandResult cmd_scene_clear(const std::vector<std::string>& args);
    CommandResult cmd_scene_validate(const std::vector<std::string>& args);

    // Circuit-Physics Bridge commands
    CommandResult cmd_bridge_add_mapping(const std::vector<std::string>& args);
    CommandResult cmd_bridge_remove_mapping(const std::vector<std::string>& args);
    CommandResult cmd_bridge_list_mappings(const std::vector<std::string>& args);
    CommandResult cmd_bridge_enable(const std::vector<std::string>& args);

    // Equivalent Circuit commands
    CommandResult cmd_equiv_circuit_add_actuator(const std::vector<std::string>& args);
    CommandResult cmd_equiv_circuit_add_mcu(const std::vector<std::string>& args);
    CommandResult cmd_equiv_circuit_update_mcu(const std::vector<std::string>& args);
    CommandResult cmd_equiv_circuit_sync_mcu_power(const std::vector<std::string>& args);
    CommandResult cmd_equiv_circuit_sync_mcu_inputs(const std::vector<std::string>& args);

    // Advanced component commands
    CommandResult cmd_component_bounds(const std::vector<std::string>& args);
    CommandResult cmd_component_position(const std::vector<std::string>& args);
    CommandResult cmd_port_list(const std::vector<std::string>& args);
    CommandResult cmd_port_info(const std::vector<std::string>& args);

    // Actuator terminal current
    CommandResult cmd_actuator_terminal_current(const std::vector<std::string>& args);

    // Helper functions
    // Script parsing
    std::vector<std::string> parse_command_line(const std::string& line);

    // Constants
    static constexpr const char* VERSION = "1.0.0";
    static constexpr const char* PROMPT = "mechatron> ";
};

} // namespace mechatron
