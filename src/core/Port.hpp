#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <functional>
#include <any>
#include <variant>
#include <cstdint>

namespace mechatron {

enum class PortDomain {
    Mechanical,
    Electrical,
    Digital,
    Analog,
    Fluid,
    Thermal,
    Data
};

enum class PortDirection {
    Input,
    Output,
    Bidirectional
};

using PortValue = std::variant<
    bool,           // Digital
    float,          // Analog / Electrical
    double,         // High-precision
    int32_t,        // Integer
    uint32_t,       // Unsigned
    std::string     // Data
>;

class Connection;

class Port {
public:
    Port(std::string name, PortDomain domain, PortDirection direction)
        : m_name(std::move(name)), m_domain(domain), m_direction(direction) {}

    std::string_view name() const { return m_name; }
    PortDomain domain() const { return m_domain; }
    PortDirection direction() const { return m_direction; }

    template <typename T>
    void set_value(T val) { m_value = std::move(val); }

    template <typename T>
    const T* get_value() const {
        return std::get_if<T>(&m_value);
    }

    const PortValue& value() const { return m_value; }

    void connect(Connection* conn) { m_connections.push_back(conn); }
    const std::vector<Connection*>& connections() const { return m_connections; }

private:
    std::string m_name;
    PortDomain m_domain;
    PortDirection m_direction;
    PortValue m_value{false};
    std::vector<Connection*> m_connections;
};

struct Connection {
    Port* source;
    Port* target;

    Connection(Port* src, Port* tgt) : source(src), target(tgt) {
        source->connect(this);
        target->connect(this);
    }
};

} // namespace mechatron
