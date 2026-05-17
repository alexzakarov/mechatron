#pragma once

#include "Types.hpp"
#include "Port.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <nlohmann/json.hpp>

namespace mechatron {

class Registry;

// Forward declaration
struct PhysicsBody;

class Component {
public:
    virtual ~Component() = default;

    virtual std::string_view plugin_type() const = 0;
    virtual std::string_view component_type() const = 0;
    virtual std::string_view category() const = 0;

    virtual void on_register(Registry& registry) {}
    virtual void update(double dt) = 0;
    virtual void on_unregister() {}

    virtual void serialize(nlohmann::json& out) const = 0;
    virtual void deserialize(const nlohmann::json& in) = 0;

    virtual std::vector<Port*> get_ports() { return {}; }

    virtual bool load_firmware_file(const std::string& path) { return false; }
    virtual bool get_mcu_pin_output_voltage(std::string_view pin_name, float& voltage) const {
        return false;
    }
    virtual bool set_mcu_pin_input_voltage(std::string_view pin_name, float voltage) {
        return false;
    }

    // Optional: bind this simulated component to a physical device (e.g., serial).
    // Defaults are no-ops so existing components remain unaffected.
    virtual bool physical_link_supported() const { return false; }
    virtual void physical_link_get_config(std::string& port, int& baud) const { (void)port; (void)baud; }
    virtual void physical_link_set_config(const std::string& port, int baud) { (void)port; (void)baud; }
    virtual bool physical_link_connect() { return false; } // uses stored config
    virtual void physical_link_disconnect() {}
    virtual bool physical_link_is_connected() const { return false; }

    const std::string& id() const { return m_id; }
    void set_id(std::string id) { m_id = std::move(id); }

    Transform& transform() { return m_transform; }
    const Transform& transform() const { return m_transform; }

    // Physics body association
    void attach_physics_body(PhysicsBody* body) { m_physics_body = body; }
    PhysicsBody* physics_body() { return m_physics_body; }
    const PhysicsBody* physics_body() const { return m_physics_body; }

    // Called after physics step to sync transform from physics body
    virtual void on_physics_update(double dt) {}

protected:
    void assign_port_owner(Port* port) {
        if (port) {
            port->m_owner = this;
        }
    }

    std::string m_id;
    Transform m_transform;
    PhysicsBody* m_physics_body = nullptr;
};

struct ComponentDescriptor {
    std::string type;
    std::string display_name;
    std::string category;       // mechanical / electronic / software / multiphysics
    std::string description;
};

} // namespace mechatron
