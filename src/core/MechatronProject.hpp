#pragma once

#include "ProjectFile.hpp"
#include "Component.hpp"
#include "physics/PhysicsWorld.hpp"
#include <vector>
#include <memory>

namespace mechatron {

// .mtrx project file handler
class MechatronProject {
public:
    MechatronProject() = default;

    // Load project from .mtrx file
    bool load(const std::string& path);

    // Save project to .mtrx file
    bool save(const std::string& path);

    // Create new project
    void create_new(const std::string& name);

    // Project metadata
    std::string name() const { return m_name; }
    std::string version() const { return m_version; }
    std::string description() const { return m_description; }
    void set_name(const std::string& name) { m_name = name; }
    void set_description(const std::string& desc) { m_description = desc; }

    // Component management
    struct ComponentInstance {
        std::string id;
        std::string type;
        Transform transform;
        nlohmann::json parameters;
    };

    struct Connection {
        std::string source_component;
        std::string source_port;
        std::string target_component;
        std::string target_port;
    };

    std::vector<ComponentInstance>& components() { return m_components; }
    std::vector<Connection>& connections() { return m_connections; }

    // Add/remove components
    void add_component(const ComponentInstance& comp);
    void remove_component(const std::string& id);
    ComponentInstance* get_component(const std::string& id);

    // Add/remove connections
    void add_connection(const Connection& conn);
    void remove_connection(const std::string& source_comp, const std::string& source_port);

    // Simulation settings
    struct SimulationSettings {
        double timestep_ms = 1.0;
        double duration_s = 10.0;
        double realtime_factor = 1.0;
        bool deterministic = true;
    };

    SimulationSettings& simulation_settings() { return m_sim_settings; }

    // Project file path
    std::string file_path() const { return m_file_path; }
    bool is_modified() const { return m_modified; }
    void set_modified(bool modified = true) { m_modified = modified; }

private:
    std::string m_file_path;
    std::string m_name = "Untitled";
    std::string m_version = "1.0";
    std::string m_description;

    std::vector<ComponentInstance> m_components;
    std::vector<Connection> m_connections;
    SimulationSettings m_sim_settings;

    bool m_modified = false;
    ProjectFile m_project_file;
};

} // namespace mechatron
