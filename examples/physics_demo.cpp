// Physics Demo - MECHATRON
// Basit fizik simülasyonu demosu

#include "core/SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace mechatron;

int main() {
    std::cout << "=== MECHATRON Physics Demo ===" << std::endl;

    // Simülasyon oluştur
    SimulationOrchestrator sim;

    // Fizik dünyası
    auto& physics = sim.physics_world();

    // Yerçekimi ayarla
    physics.set_gravity({0, -9.81f, 0});

    // Test box oluştur
    CollisionShapeDef box_shape;
    box_shape.type = CollisionShape::Box;
    box_shape.box_extents = {0.5f, 0.5f, 0.5f};

    auto* box1 = physics.create_body("box1", box_shape, {0, 5, 0});
    box1->mass = 1.0f;
    box1->restitution = 0.7f;
    box1->friction = 0.3f;

    auto* box2 = physics.create_body("box2", box_shape, {0.2f, 8, 0});
    box2->mass = 1.5f;
    box2->restitution = 0.5f;

    // Sphere oluştur
    CollisionShapeDef sphere_shape;
    sphere_shape.type = CollisionShape::Sphere;
    sphere_shape.sphere_radius = 0.5f;

    auto* sphere = physics.create_body("sphere1", sphere_shape, {-1, 10, 0});
    sphere->mass = 0.5f;
    sphere->restitution = 0.9f;

    // Static ground (görünmez)
    CollisionShapeDef ground_shape;
    ground_shape.type = CollisionShape::Box;
    ground_shape.box_extents = {10, 0.1f, 10};

    auto* ground = physics.create_body("ground", ground_shape, {0, -0.1f, 0});
    ground->is_static = true;

    std::cout << "Created " << physics.body_count() << " bodies" << std::endl;
    std::cout << "Starting simulation (5 seconds)..." << std::endl;

    // Simülasyon döngüsü
    const double dt = 0.016;  // ~60 FPS
    const int total_steps = 300;  // 5 saniye

    sim.start();

    for (int i = 0; i < total_steps; ++i) {
        // Update physics for exactly one timestep per frame
        physics.step(dt);

        // Her 60 frame'de bir durum yaz
        if (i % 60 == 0) {
            double time = i * dt;  // Basit zaman hesabı
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "t=" << time << "s | ";
            std::cout << "box1: y=" << box1->position.y << " ";
            std::cout << "sphere: y=" << sphere->position.y << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    sim.stop();

    std::cout << "\nFinal positions:" << std::endl;
    std::cout << "  box1:   (" << box1->position.x << ", " << box1->position.y << ", " << box1->position.z << ")" << std::endl;
    std::cout << "  box2:   (" << box2->position.x << ", " << box2->position.y << ", " << box2->position.z << ")" << std::endl;
    std::cout << "  sphere: (" << sphere->position.x << ", " << sphere->position.y << ", " << sphere->position.z << ")" << std::endl;

    return 0;
}
