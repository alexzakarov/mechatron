#include "PhysicsWorld.hpp"
#include <spdlog/spdlog.h>
#include <cmath>

namespace mechatron {

PhysicsWorld::PhysicsWorld() {
    spdlog::info("PhysicsWorld initialized");
}

PhysicsWorld::~PhysicsWorld() {
    spdlog::info("PhysicsWorld destroyed");
}

void PhysicsWorld::set_gravity(const Vec3& g) {
    m_settings.gravity = g;
}

PhysicsBody* PhysicsWorld::create_body(std::string id, const CollisionShapeDef& shape, const Vec3& position) {
    auto body = std::make_unique<PhysicsBody>();
    body->id = id;
    body->position = position;

    m_bodies[id] = std::move(body);
    m_shapes[id] = shape;

    spdlog::debug("Created physics body: {} at ({}, {}, {})",
        id, position.x, position.y, position.z);

    return m_bodies[id].get();
}

void PhysicsWorld::remove_body(std::string_view id) {
    std::string id_str(id);
    auto it = m_bodies.find(id_str);
    if (it != m_bodies.end()) {
        m_shapes.erase(id_str);
        m_bodies.erase(it);
        spdlog::debug("Removed physics body: {}", id);
    }
}

PhysicsBody* PhysicsWorld::get_body(std::string_view id) {
    std::string id_str(id);
    auto it = m_bodies.find(id_str);
    return (it != m_bodies.end()) ? it->second.get() : nullptr;
}

void PhysicsWorld::step(double dt) {
    // Apply gravity and external forces to velocities
    for (auto& [id, body] : m_bodies) {
        if (body->is_static || body->is_kinematic) {
            body->force_accumulator = {0, 0, 0};
            body->torque_accumulator = {0, 0, 0};
            continue;
        }

        float inv_mass = (body->mass > 0.0f) ? 1.0f / body->mass : 0.0f;

        // Apply gravity
        body->velocity.x += m_settings.gravity.x * dt;
        body->velocity.y += m_settings.gravity.y * dt;
        body->velocity.z += m_settings.gravity.z * dt;

        // Apply accumulated external forces: a = F/m
        body->velocity.x += body->force_accumulator.x * inv_mass * dt;
        body->velocity.y += body->force_accumulator.y * inv_mass * dt;
        body->velocity.z += body->force_accumulator.z * inv_mass * dt;

        // Apply accumulated torques to angular velocity
        body->angular_velocity.x += body->torque_accumulator.x * inv_mass * dt;
        body->angular_velocity.y += body->torque_accumulator.y * inv_mass * dt;
        body->angular_velocity.z += body->torque_accumulator.z * inv_mass * dt;

        // Clear accumulators
        body->force_accumulator = {0, 0, 0};
        body->torque_accumulator = {0, 0, 0};
    }

    update_positions(dt);
    resolve_collisions();
}

void PhysicsWorld::update_positions(double dt) {
    // Simple Euler integration: p = p + v * dt
    for (auto& [id, body] : m_bodies) {
        if (body->is_static) continue;

        body->position.x += body->velocity.x * dt;
        body->position.y += body->velocity.y * dt;
        body->position.z += body->velocity.z * dt;

        // Ground collision (simple plane at y=0)
        const std::string& body_id = body->id;
        auto& shape = m_shapes[body_id];
        float bottom = body->position.y;
        float half_height = 0.0f;

        switch (shape.type) {
            case CollisionShape::Box:
                half_height = shape.box_extents.y;
                break;
            case CollisionShape::Sphere:
                half_height = shape.sphere_radius;
                break;
            case CollisionShape::Capsule:
                half_height = shape.capsule_height / 2.0f + shape.capsule_radius;
                break;
            case CollisionShape::Cylinder:
                half_height = shape.cylinder_height / 2.0f;
                break;
            default:
                break;
        }

        if (bottom - half_height < 0.0f) {
            body->position.y = half_height;

            // Bounce with restitution
            if (body->velocity.y < 0.0f) {
                body->velocity.y = -body->velocity.y * body->restitution;

                // Apply friction to horizontal velocity
                body->velocity.x *= (1.0f - body->friction * 0.1f);
                body->velocity.z *= (1.0f - body->friction * 0.1f);

                // Stop if velocity is very low
                if (std::abs(body->velocity.y) < 0.1f) {
                    body->velocity.y = 0.0f;
                }
            }
        }
    }
}

void PhysicsWorld::resolve_collisions() {
    // Simple sphere-sphere collision detection
    auto bodies = get_all_bodies();

    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            PhysicsBody* a = bodies[i];
            PhysicsBody* b = bodies[j];

            if (a->is_static && b->is_static) continue;

            // Get radii
            const auto& shape_a = m_shapes[a->id];
            const auto& shape_b = m_shapes[b->id];

            float radius_a = 0.5f;
            float radius_b = 0.5f;

            if (shape_a.type == CollisionShape::Sphere) radius_a = shape_a.sphere_radius;
            if (shape_a.type == CollisionShape::Box) {
                // Use bounding sphere radius for simplified collision
                // For ground-like objects, use a reasonable default
                float max_extent = std::max({shape_a.box_extents.x, shape_a.box_extents.y, shape_a.box_extents.z});
                radius_a = (max_extent > 5.0f) ? 0.5f : max_extent;  // Cap large objects
            }
            if (shape_b.type == CollisionShape::Sphere) radius_b = shape_b.sphere_radius;
            if (shape_b.type == CollisionShape::Box) {
                float max_extent = std::max({shape_b.box_extents.x, shape_b.box_extents.y, shape_b.box_extents.z});
                radius_b = (max_extent > 5.0f) ? 0.5f : max_extent;
            }

            // Check distance
            Vec3 diff = {
                b->position.x - a->position.x,
                b->position.y - a->position.y,
                b->position.z - a->position.z
            };
            float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
            float min_dist = radius_a + radius_b;

            if (dist < min_dist && dist > 0.0001f) {
                // Collision detected - simple impulse response
                Vec3 normal = {diff.x / dist, diff.y / dist, diff.z / dist};

                float overlap = min_dist - dist;

                // Separate bodies
                float total_mass = a->mass + b->mass;
                float ratio_a = b->mass / total_mass;
                float ratio_b = a->mass / total_mass;

                if (!a->is_static) {
                    a->position.x -= normal.x * overlap * ratio_a;
                    a->position.y -= normal.y * overlap * ratio_a;
                    a->position.z -= normal.z * overlap * ratio_a;
                }
                if (!b->is_static) {
                    b->position.x += normal.x * overlap * ratio_b;
                    b->position.y += normal.y * overlap * ratio_b;
                    b->position.z += normal.z * overlap * ratio_b;
                }

                // Elastic collision response
                Vec3 rel_vel = {
                    b->velocity.x - a->velocity.x,
                    b->velocity.y - a->velocity.y,
                    b->velocity.z - a->velocity.z
                };

                float vel_along_normal = rel_vel.x * normal.x + rel_vel.y * normal.y + rel_vel.z * normal.z;

                if (vel_along_normal > 0) continue; // Moving apart

                float restitution = std::min(a->restitution, b->restitution);
                float j = -(1.0f + restitution) * vel_along_normal;
                j /= (1.0f / a->mass + 1.0f / b->mass);

                Vec3 impulse = {j * normal.x, j * normal.y, j * normal.z};

                if (!a->is_static) {
                    a->velocity.x -= impulse.x / a->mass;
                    a->velocity.y -= impulse.y / a->mass;
                    a->velocity.z -= impulse.z / a->mass;
                }
                if (!b->is_static) {
                    b->velocity.x += impulse.x / b->mass;
                    b->velocity.y += impulse.y / b->mass;
                    b->velocity.z += impulse.z / b->mass;
                }
            }
        }
    }
}

void PhysicsWorld::reset() {
    m_bodies.clear();
    m_shapes.clear();
    spdlog::info("PhysicsWorld reset");
}

std::vector<PhysicsBody*> PhysicsWorld::get_all_bodies() {
    std::vector<PhysicsBody*> result;
    result.reserve(m_bodies.size());
    for (auto& [id, body] : m_bodies) {
        result.push_back(body.get());
    }
    return result;
}

void PhysicsWorld::add_force(std::string_view body_id, const Vec3& force) {
    PhysicsBody* body = get_body(body_id);
    if (body && !body->is_static) {
        body->force_accumulator.x += force.x;
        body->force_accumulator.y += force.y;
        body->force_accumulator.z += force.z;
    }
}

void PhysicsWorld::add_torque(std::string_view body_id, const Vec3& torque) {
    PhysicsBody* body = get_body(body_id);
    if (body && !body->is_static) {
        body->torque_accumulator.x += torque.x;
        body->torque_accumulator.y += torque.y;
        body->torque_accumulator.z += torque.z;
    }
}

} // namespace mechatron
