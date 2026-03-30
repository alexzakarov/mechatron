#pragma once

#include "Component.hpp"
#include <vector>
#include <string>
#include <memory>
#include <nlohmann/json.hpp>

namespace mechatron {

struct ExposedPort {
    std::string name;
    std::string component_id;
    std::string port_name;
};

class Subsystem {
public:
    explicit Subsystem(std::string id) : m_id(std::move(id)) {}

    const std::string& id() const { return m_id; }

    void add_component(std::string component_id, std::string role) {
        m_members.push_back({std::move(component_id), std::move(role)});
    }

    void add_internal_connection(std::string source, std::string target) {
        m_internal_connections.push_back({std::move(source), std::move(target)});
    }

    void add_exposed_port(ExposedPort port) {
        m_exposed_ports.push_back(std::move(port));
    }

    void set_display_name(std::string name) { m_display_name = std::move(name); }
    const std::string& display_name() const { return m_display_name; }

    struct Member {
        std::string component_id;
        std::string role;
    };

    struct InternalConnection {
        std::string source;
        std::string target;
    };

    const std::vector<Member>& members() const { return m_members; }
    const std::vector<InternalConnection>& internal_connections() const { return m_internal_connections; }
    const std::vector<ExposedPort>& exposed_ports() const { return m_exposed_ports; }

    void serialize(nlohmann::json& out) const;
    void deserialize(const nlohmann::json& in);

private:
    std::string m_id;
    std::string m_display_name;
    std::vector<Member> m_members;
    std::vector<InternalConnection> m_internal_connections;
    std::vector<ExposedPort> m_exposed_ports;
};

} // namespace mechatron
