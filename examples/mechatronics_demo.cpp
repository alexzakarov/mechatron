// Mechatronics Demo - MECHATRON
// Solenoid aktüatör ve sensör entegrasyon demosu

#include "core/SimulationOrchestrator.hpp"
#include "physics/PhysicsWorld.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace mechatron;

int main() {
    std::cout << "=== MECHATRON Mechatronics Demo ===" << std::endl;
    std::cout << "Solenoid Actuator + Proximity Sensor Control Loop\n" << std::endl;

    SimulationOrchestrator sim;
    auto& physics = sim.physics_world();
    physics.set_gravity({0, -9.81f, 0});

    // Solenoid aktüatör oluştur
    auto solenoid = std::make_unique<SolenoidActuator>();
    solenoid->set_id("solenoid1");
    solenoid->set_resistance(12.0f);
    solenoid->set_stroke(10.0f);     // 10mm stroke
    solenoid->set_plunger_mass(5.0f); // 5g
    solenoid->set_spring_constant(50.0f);

    // Proximity sensör oluştur
    auto sensor = std::make_unique<ProximitySensor>();
    sensor->set_id("prox1");
    sensor->set_max_range(0.5f);    // 50cm max range
    sensor->transform().position = {0, 0.05f, 0}; // Sensor position
    sensor->set_target_position({0, 0, 0});       // Measure distance to origin

    // Physik body oluşturur (solenoid plunger)
    CollisionShapeDef plunger_shape;
    plunger_shape.type = CollisionShape::Box;
    plunger_shape.box_extents = {0.02f, 0.02f, 0.05f}; // 4cm x 4cm x 10cm

    auto* plunger = physics.create_body("plunger", plunger_shape, {0, 0.1f, 0});
    plunger->mass = 0.005f; // 5g
    plunger->restitution = 0.1f;
    plunger->friction = 0.8f;

    // Ground
    CollisionShapeDef ground_shape;
    ground_shape.type = CollisionShape::Box;
    ground_shape.box_extents = {0.5f, 0.01f, 0.5f};

    auto* ground = physics.create_body("ground", ground_shape, {0, 0, 0});
    ground->is_static = true;

    // Kontrol değişkenleri
    const double dt = 0.001; // 1ms timestep
    float control_input = 0.0f;
    bool solenoid_on = false;

    std::cout << "Starting control loop..." << std::endl;
    std::cout << "Sensor will detect plunger position and control solenoid.\n" << std::endl;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | Sensor(m) | Solenoid | Plunger Y | Vel Y" << std::endl;
    std::cout << "--------|----------|----------|----------|-------" << std::endl;

    // 10 saniye simülasyon
    const int total_steps = 10000; // 10 seconds @ 1ms

    for (int i = 0; i < total_steps; ++i) {
        double time = i * dt;

        // Sensörü güncelle (plunger pozisyonunu oku)
        sensor->transform().position = plunger->position;
        sensor->update(dt);
        float distance = sensor->read();

        // Basit kontrol: Plunger 5cm'nin üzerindeyse solenoidi aç
        bool target_solenoid_state = distance > 0.05f;

        // Debounce: 10ms'den kısa değişimleri yoksay
        static int debounce_counter = 0;
        if (target_solenoid_state != solenoid_on) {
            debounce_counter++;
            if (debounce_counter > 10) {
                solenoid_on = target_solenoid_state;
                debounce_counter = 0;
            }
        } else {
            debounce_counter = 0;
        }

        control_input = solenoid_on ? 1.0f : 0.0f;
        solenoid->set_input(control_input);
        solenoid->update(dt);

        // Solenoid kuvvetini plunger'a uygula
        if (control_input > 0.5f) {
            // Yukarı doğru kuvvet (Y ekseni) - gravity'ye karşı
            float force = 0.05f; // Newton (daha küçük kuvvet)
            plunger->velocity.y += force / plunger->mass * dt;
        }

        // Air resistance (damping)
        plunger->velocity.y *= 0.98f;

        // Fizik adımı
        physics.step(dt);

        // Her 0.5 saniyede bir durum yaz
        if (i % 500 == 0) {
            std::cout << std::setw(7) << time << " | "
                      << std::setw(8) << distance << " | "
                      << (solenoid_on ? "ON " : "OFF") << "   | "
                      << std::setw(8) << plunger->position.y << " | "
                      << std::setw(5) << plunger->velocity.y << std::endl;
        }
    }

    std::cout << "\n--- Final State ---" << std::endl;
    std::cout << "Solenoid position: " << solenoid->get_position() * 100 << "%" << std::endl;
    std::cout << "Plunger position: (" << plunger->position.x << ", "
              << plunger->position.y << ", " << plunger->position.z << ")" << std::endl;
    std::cout << "Sensor reading: " << sensor->read() << "m" << std::endl;

    std::cout << "\nDemo complete!" << std::endl;

    return 0;
}
