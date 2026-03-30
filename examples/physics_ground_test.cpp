// Ground collision test
#include "core/SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include <iostream>

using namespace mechatron;

int main() {
    std::cout << "=== Ground Collision Test ===" << std::endl;

    SimulationOrchestrator sim;
    auto& physics = sim.physics_world();
    physics.set_gravity({0, -9.81f, 0});

    // Box yukarıda
    CollisionShapeDef box_shape;
    box_shape.type = CollisionShape::Box;
    box_shape.box_extents = {0.5f, 0.5f, 0.5f};

    auto* box = physics.create_body("box", box_shape, {0, 3, 0});
    box->mass = 1.0f;
    box->restitution = 0.5f;

    // Ground (static)
    CollisionShapeDef ground_shape;
    ground_shape.type = CollisionShape::Box;
    ground_shape.box_extents = {10, 0.1f, 10};

    auto* ground = physics.create_body("ground", ground_shape, {0, 0, 0});
    ground->is_static = true;

    std::cout << "Box at y=3, ground at y=0" << std::endl;
    std::cout << "Box half_height=0.5, ground half_height=0.1" << std::endl;

    const double dt = 0.016;
    for (int i = 0; i < 300; ++i) {
        physics.step(dt);

        if (i % 60 == 0) {
            double time = i * dt;
            std::cout << "t=" << time << "s | box y=" << box->position.y
                      << ", vel y=" << box->velocity.y << std::endl;
        }

        // Stop when settled
        if (i > 100 && std::abs(box->velocity.y) < 0.01f) {
            std::cout << "\nSettled at t=" << i * dt << "s, y=" << box->position.y << std::endl;
            break;
        }
    }

    return 0;
}
