#include "MechatronProject.hpp"
#include <spdlog/spdlog.h>
#include <fstream>

namespace mechatron {

bool MechatronProject::load(const std::string& path) {
    if (!m_project_file.load(path)) {
        spdlog::error("Failed to load project file: {}", path);
        return false;
    }

    m_file_path = path;

    // Parse JSON
    const auto& data = m_project_file.data();

    if (data.contains("name")) m_name = data["name"];
    if (data.contains("version")) m_version = data["version"];
    if (data.contains("description")) m_description = data["description"];

    // Load components
    m_components.clear();
    if (data.contains("components")) {
        for (const auto& comp_json : data["components"]) {
            ComponentInstance comp;
            comp.id = comp_json.value("id", "");
            comp.type = comp_json.value("type", "");

            if (comp_json.contains("transform")) {
                const auto& trans = comp_json["transform"];
                if (trans.contains("position")) {
                    comp.transform.position.x = trans["position"][0];
                    comp.transform.position.y = trans["position"][1];
                    comp.transform.position.z = trans["position"][2];
                }
                if (trans.contains("rotation")) {
                    comp.transform.rotation.x = trans["rotation"][0];
                    comp.transform.rotation.y = trans["rotation"][1];
                    comp.transform.rotation.z = trans["rotation"][2];
                    comp.transform.rotation.w = trans["rotation"][3];
                }
                if (trans.contains("scale")) {
                    comp.transform.scale.x = trans["scale"][0];
                    comp.transform.scale.y = trans["scale"][1];
                    comp.transform.scale.z = trans["scale"][2];
                }
            }

            if (comp_json.contains("parameters")) {
                comp.parameters = comp_json["parameters"];
            }

            m_components.push_back(comp);
        }
    }

    // Load connections
    m_connections.clear();
    if (data.contains("connections")) {
        for (const auto& conn_json : data["connections"]) {
            Connection conn;
            conn.source_component = conn_json.value("source_component", "");
            conn.source_port = conn_json.value("source_port", "");
            conn.target_component = conn_json.value("target_component", "");
            conn.target_port = conn_json.value("target_port", "");
            m_connections.push_back(conn);
        }
    }

    // Load simulation settings
    if (data.contains("simulation")) {
        const auto& sim = data["simulation"];
        m_sim_settings.timestep_ms = sim.value("timestep_ms", 1.0);
        m_sim_settings.duration_s = sim.value("duration_s", 10.0);
        m_sim_settings.realtime_factor = sim.value("realtime_factor", 1.0);
        m_sim_settings.deterministic = sim.value("deterministic", true);
    }

    m_modified = false;
    spdlog::info("Project loaded: {} (v{})", m_name, m_version);
    return true;
}

bool MechatronProject::save(const std::string& path) {
    nlohmann::json data;

    data["version"] = m_version;
    data["name"] = m_name;
    data["description"] = m_description;

    // Save components
    data["components"] = nlohmann::json::array();
    for (const auto& comp : m_components) {
        nlohmann::json comp_json;
        comp_json["id"] = comp.id;
        comp_json["type"] = comp.type;

        nlohmann::json trans;
        trans["position"] = {comp.transform.position.x, comp.transform.position.y, comp.transform.position.z};
        trans["rotation"] = {comp.transform.rotation.x, comp.transform.rotation.y, comp.transform.rotation.z, comp.transform.rotation.w};
        trans["scale"] = {comp.transform.scale.x, comp.transform.scale.y, comp.transform.scale.z};
        comp_json["transform"] = trans;

        if (!comp.parameters.empty()) {
            comp_json["parameters"] = comp.parameters;
        }

        data["components"].push_back(comp_json);
    }

    // Save connections
    data["connections"] = nlohmann::json::array();
    for (const auto& conn : m_connections) {
        nlohmann::json conn_json;
        conn_json["source_component"] = conn.source_component;
        conn_json["source_port"] = conn.source_port;
        conn_json["target_component"] = conn.target_component;
        conn_json["target_port"] = conn.target_port;
        data["connections"].push_back(conn_json);
    }

    // Save simulation settings
    data["simulation"] = {
        {"timestep_ms", m_sim_settings.timestep_ms},
        {"duration_s", m_sim_settings.duration_s},
        {"realtime_factor", m_sim_settings.realtime_factor},
        {"deterministic", m_sim_settings.deterministic}
    };

    m_project_file.data() = data;

    if (!m_project_file.save(path)) {
        spdlog::error("Failed to save project: {}", path);
        return false;
    }

    m_file_path = path;
    m_modified = false;
    spdlog::info("Project saved: {}", path);
    return true;
}

void MechatronProject::create_new(const std::string& name) {
    m_name = name;
    m_version = "1.0";
    m_description = "";
    m_components.clear();
    m_connections.clear();
    m_sim_settings = SimulationSettings{};
    m_file_path = "";
    m_modified = false;
    spdlog::info("Created new project: {}", name);
}

void MechatronProject::add_component(const ComponentInstance& comp) {
    m_components.push_back(comp);
    m_modified = true;
}

void MechatronProject::remove_component(const std::string& id) {
    auto it = std::remove_if(m_components.begin(), m_components.end(),
        [&id](const ComponentInstance& c) { return c.id == id; });
    if (it != m_components.end()) {
        m_components.erase(it, m_components.end());
        m_modified = true;
    }
}

MechatronProject::ComponentInstance* MechatronProject::get_component(const std::string& id) {
    auto it = std::find_if(m_components.begin(), m_components.end(),
        [&id](const ComponentInstance& c) { return c.id == id; });
    return it != m_components.end() ? &(*it) : nullptr;
}

void MechatronProject::add_connection(const Connection& conn) {
    m_connections.push_back(conn);
    m_modified = true;
}

void MechatronProject::remove_connection(const std::string& source_comp, const std::string& source_port) {
    auto it = std::remove_if(m_connections.begin(), m_connections.end(),
        [&source_comp, &source_port](const Connection& c) {
            return c.source_component == source_comp && c.source_port == source_port;
        });
    if (it != m_connections.end()) {
        m_connections.erase(it, m_connections.end());
        m_modified = true;
    }
}

} // namespace mechatron
