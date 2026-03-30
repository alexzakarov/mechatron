#pragma once

#include "core/Types.hpp"
#include <string>
#include <string_view>
#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>

namespace mechatron {

// Simple physics body representation
struct PhysicsBody {
    std::string id;
    Vec3 position{0, 0, 0};
    Vec3 velocity{0, 0, 0};
    Vec3 angular_velocity{0, 0, 0};
    Quat rotation{0, 0, 0, 1};
    float mass = 1.0f;
    float restitution = 0.5f;  // Bounciness
    float friction = 0.5f;
    bool is_static = false;
    bool is_kinematic = false;

    // External force/torque accumulators (cleared each step)
    Vec3 force_accumulator{0, 0, 0};
    Vec3 torque_accumulator{0, 0, 0};

    // User data for rendering
    void* user_data = nullptr;
};

// Simple collision shape types
enum class CollisionShape {
    Box,
    Sphere,
    Capsule,
    Cylinder,
    Mesh
};

struct CollisionShapeDef {
    CollisionShape type;
    Vec3 center{0, 0, 0};

    // Shape-specific parameters
    Vec3 box_extents{1, 1, 1};      // Half-sizes
    float sphere_radius = 0.5f;
    float capsule_height = 1.0f;
    float capsule_radius = 0.25f;
    float cylinder_height = 1.0f;
    float cylinder_radius = 0.5f;
};

// Physics world configuration
struct PhysicsSettings {
    Vec3 gravity{0, -9.81f, 0};
    int solver_iterations = 8;
    int substeps = 1;
    bool enable_sleeping = true;
};

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    // World configuration
    void set_gravity(const Vec3& g);
    Vec3 gravity() const { return m_settings.gravity; }

    // Body management
    PhysicsBody* create_body(std::string id, const CollisionShapeDef& shape, const Vec3& position);
    void remove_body(std::string_view id);
    PhysicsBody* get_body(std::string_view id);

    // Simulation
    void step(double dt);
    void reset();

    // External force/torque application
    void add_force(std::string_view body_id, const Vec3& force);
    void add_torque(std::string_view body_id, const Vec3& torque);

    // Queries
    std::vector<PhysicsBody*> get_all_bodies();
    size_t body_count() const { return m_bodies.size(); }

    // Raycasting (future)
    // bool raycast(const Vec3& from, const Vec3& to, RaycastHit& hit);

private:
    void update_positions(double dt);
    void resolve_collisions();

    PhysicsSettings m_settings;
    std::unordered_map<std::string, std::unique_ptr<PhysicsBody>> m_bodies;
    std::unordered_map<std::string, CollisionShapeDef> m_shapes;
};

} // namespace mechatron
