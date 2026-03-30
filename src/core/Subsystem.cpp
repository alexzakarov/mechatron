#include "Subsystem.hpp"

namespace mechatron {

void Subsystem::serialize(nlohmann::json& out) const {
    out["id"] = m_id;
    out["display_name"] = m_display_name;

    auto& members_json = out["members"];
    for (const auto& m : m_members) {
        members_json.push_back({{"component_id", m.component_id}, {"role", m.role}});
    }

    auto& conns_json = out["internal_connections"];
    for (const auto& c : m_internal_connections) {
        conns_json.push_back({{"source", c.source}, {"target", c.target}});
    }

    auto& ports_json = out["exposed_ports"];
    for (const auto& p : m_exposed_ports) {
        ports_json.push_back({{"name", p.name}, {"component_id", p.component_id}, {"port_name", p.port_name}});
    }
}

void Subsystem::deserialize(const nlohmann::json& in) {
    m_id = in.value("id", "");
    m_display_name = in.value("display_name", "");

    if (in.contains("members")) {
        for (const auto& m : in["members"]) {
            m_members.push_back({m.value("component_id", ""), m.value("role", "")});
        }
    }

    if (in.contains("internal_connections")) {
        for (const auto& c : in["internal_connections"]) {
            m_internal_connections.push_back({c.value("source", ""), c.value("target", "")});
        }
    }

    if (in.contains("exposed_ports")) {
        for (const auto& p : in["exposed_ports"]) {
            m_exposed_ports.push_back({p.value("name", ""), p.value("component_id", ""), p.value("port_name", "")});
        }
    }
}

} // namespace mechatron
