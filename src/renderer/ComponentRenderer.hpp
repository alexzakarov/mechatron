#pragma once

#include "Shader.hpp"
#include "Mesh.hpp"
#include "Camera.hpp"
#include "core/Component.hpp"
#include <GL/glew.h>
#include <memory>
#include <unordered_map>

namespace mechatron {

/**
 * @brief Component visualization renderer
 *
 * Renders mechatronic components in 3D viewport:
 * - DC Motors, Servo Motors, Stepper Motors
 * - Sensors (Encoders, Limit Switches, Proximity)
 * - Actuators (Solenoid, Linear Actuators)
 * - Mechanical elements (Gears, Shafts, Bearings)
 */
class ComponentRenderer {
public:
    ComponentRenderer();
    ~ComponentRenderer();

    bool init();
    void shutdown();

    /**
     * Render a single component
     */
    void render_component(const Component& comp, const Camera& camera, float aspect);

    /**
     * Render all components in registry
     */
    void render_all(const Camera& camera, float aspect);

    /**
     * Set component registry for rendering all components
     */
    void set_registry(class Registry* registry) { m_registry = registry; }

    /**
     * Highlight selected component
     */
    void set_selected(const std::string& id) { m_selected_id = id; }

private:
    void create_component_meshes();

    // Mechanics - Machine Elements
    void render_dc_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_servo_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_solenoid(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_stepper_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_linear_actuator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_gear(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected, int teeth = 20);
    void render_shaft(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_bearing(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_spring(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_damper(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_lead_screw(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_pulley(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_belt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_cam(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Sensors
    void render_encoder(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_limit_switch(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_proximity_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_potentiometer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_load_cell(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_accelerometer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_gyroscope(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_temperature_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_pressure_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_flow_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_sensor_box(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Electronics - Passive
    void render_resistor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_capacitor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_inductor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_transformer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_potentiometer_trimpot(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Electronics - Semiconductor
    void render_diode(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_led(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_bjt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_mosfet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_igbt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_thyristor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_voltage_regulator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_op_amp(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Electronics - Power
    void render_h_bridge(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_buck_converter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_boost_converter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void rectifier(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_motor_driver(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_relay(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Electronics - Digital/Active
    void render_logic_gate(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_microcontroller(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_adc_dac(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_display(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_connector(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_terminal_block(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_pcb(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Software/MCU
    void render_arduino_uno(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_arduino_mega(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_stm32_board(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_esp32_board(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_raspberry_pi(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Fluid Power
    void render_hydraulic_cylinder(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_hydraulic_pump(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_pneumatic_valve(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_compressor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_filter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_accumulator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_hose(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Multiphysics - Thermal
    void render_heat_source(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_heat_sink(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_thermal_resistance(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_fan(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_thermoelectric_cooler(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Multiphysics - Magnetic
    void render_permanent_magnet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_electromagnet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_magnetic_core(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_solenoid_coil(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Robotics
    void render_robot_link(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_robot_joint(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_end_effector(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);
    void render_gripper(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    // Generic fallback
    void render_generic_component(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected);

    Shader m_shader;
    Mesh m_box_mesh;
    Mesh m_cylinder_mesh;
    Mesh m_sphere_mesh;

    Registry* m_registry = nullptr;
    std::string m_selected_id;

    // Component type colors
    Vec3 m_color_motor{0.8f, 0.6f, 0.4f};
    Vec3 m_color_sensor{0.4f, 0.8f, 0.6f};
    Vec3 m_color_actuator{0.6f, 0.4f, 0.8f};
    Vec3 m_color_generic{0.7f, 0.7f, 0.7f};
    Vec3 m_color_selected{1.0f, 0.8f, 0.2f};
};

} // namespace mechatron
