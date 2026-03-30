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
