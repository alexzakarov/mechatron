// Physics Debug Demo - MECHATRON
// Fizik simülasyonu debug demosu

#include "core/SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace mechatron;

int main() {
    std::cout << "=== MECHATRON Physics Debug ===" << std::endl;

    SimulationOrchestrator sim;
    auto& physics = sim.physics_world();
    physics.set_gravity({0, -9.81f, 0});

    CollisionShapeDef box_shape;
    box_shape.type = CollisionShape::Box;
    box_shape.box_extents = {0.5f, 0.5f, 0.5f};

    auto* box1 = physics.create_body("box1", box_shape, {0, 5, 0});
    box1->mass = 1.0f;
    box1->restitution = 0.7f;

    std::cout << "Initial state:" << std::endl;
    std::cout << "  position: (" << box1->position.x << ", " << box1->position.y << ", " << box1->position.z << ")" << std::endl;
    std::cout << "  velocity: (" << box1->velocity.x << ", " << box1->velocity.y << ", " << box1->velocity.z << ")" << std::endl;
    std::cout << "  gravity: (" << physics.gravity().x << ", " << physics.gravity().y << ", " << physics.gravity().z << ")" << std::endl;

    const double dt = 0.016;

    std::cout << "\nStepping physics:" << std::endl;
    for (int i = 0; i < 5; ++i) {
        std::cout << "\nStep " << i << ":" << std::endl;
        std::cout << "  Before - vel: (" << box1->velocity.x << ", " << box1->velocity.y << "), pos: " << box1->position.y << std::endl;

        physics.step(dt);

        std::cout << "  After  - vel: (" << box1->velocity.x << ", " << box1->velocity.y << "), pos: " << box1->position.y << std::endl;
    }

    return 0;
}
