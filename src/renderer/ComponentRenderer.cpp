#include "ComponentRenderer.hpp"
#include "core/Registry.hpp"
#include "actuators/Actuator.hpp"
#include "sensors/Sensor.hpp"
#include "electronics/CircuitSimulator.hpp"
#include <spdlog/spdlog.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace mechatron {

static const char* component_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 FragPos;
out vec3 Normal;

void main() {
    FragPos = vec3(uModel * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uProj * uView * vec4(FragPos, 1.0);
}
)";

static const char* component_fragment_shader = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
out vec4 FragColor;

uniform vec3 uColor;
uniform vec3 uViewPos;
uniform bool uSelected;

void main() {
    // Higher ambient for better base visibility
    float ambientStrength = 0.6f;
    vec3 ambient = ambientStrength * uColor;

    // Normalize the normal vector
    vec3 norm = normalize(Normal);

    // Top directional light (from above)
    vec3 lightDir1 = normalize(vec3(0.0, 1.0, 0.0));
    float diff1 = max(dot(norm, lightDir1), 0.0);
    vec3 diffuse1 = 0.4 * diff1 * uColor;

    // Bottom directional light (fill light from below)
    vec3 lightDir2 = normalize(vec3(0.0, -1.0, 0.0));
    float diff2 = max(dot(norm, lightDir2), 0.0);
    vec3 diffuse2 = 0.2 * diff2 * uColor;

    // Side lights for depth
    vec3 lightDir3 = normalize(vec3(1.0, 0.5, 1.0));
    float diff3 = max(dot(norm, lightDir3), 0.0);
    vec3 diffuse3 = 0.2 * diff3 * uColor;

    // Specular (subtle)
    float specularStrength = 0.3f;
    vec3 viewDir = normalize(uViewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir1, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
    vec3 specular = specularStrength * spec * vec3(1.0);

    // Combine all lighting
    vec3 result = ambient + diffuse1 + diffuse2 + diffuse3 + specular;

    // Highlight selected components
    if (uSelected) {
        result = mix(result, vec3(1.0, 0.8, 0.2), 0.3);
    }

    FragColor = vec4(result, 1.0);
}
)";

ComponentRenderer::ComponentRenderer() = default;

ComponentRenderer::~ComponentRenderer() {
    shutdown();
}

bool ComponentRenderer::init() {
    if (!m_shader.load_from_source(component_vertex_shader, component_fragment_shader)) {
        spdlog::error("ComponentRenderer: Failed to compile shader");
        return false;
    }

    // Create primitive meshes
    m_box_mesh = Mesh::create_box(1.0f, 1.0f, 1.0f);
    m_cylinder_mesh = Mesh::create_cylinder(0.5f, 1.0f, 32);
    m_sphere_mesh = Mesh::create_sphere(0.5f, 16, 16);

    spdlog::info("ComponentRenderer initialized");
    return true;
}

void ComponentRenderer::shutdown() {
    // Meshes cleaned up automatically
}

void ComponentRenderer::render_component(const Component& comp, const Camera& camera, float aspect) {
    const auto& t = comp.transform();
    bool selected = (comp.id() == m_selected_id);

    Vec3 rotation{0, 0, 0}; // TODO: Add rotation to Transform

    // Route to appropriate renderer based on component type
    std::string_view type = comp.component_type();

    // Mechanics - Machine Elements
    if (type == "dc_motor") {
        render_dc_motor(t.position, t.scale, rotation, selected);
    } else if (type == "servo_motor") {
        render_servo_motor(t.position, t.scale, rotation, selected);
    } else if (type == "solenoid_actuator") {
        render_solenoid(t.position, t.scale, rotation, selected);
    } else if (type == "stepper_motor") {
        render_stepper_motor(t.position, t.scale, rotation, selected);
    } else if (type == "linear_actuator") {
        render_linear_actuator(t.position, t.scale, rotation, selected);
    } else if (type == "gear" || type == "spur_gear" || type == "helical_gear" || type == "bevel_gear") {
        render_gear(t.position, t.scale, rotation, selected);
    } else if (type == "shaft") {
        render_shaft(t.position, t.scale, rotation, selected);
    } else if (type == "bearing") {
        render_bearing(t.position, t.scale, rotation, selected);
    } else if (type == "spring") {
        render_spring(t.position, t.scale, rotation, selected);
    } else if (type == "damper") {
        render_damper(t.position, t.scale, rotation, selected);
    } else if (type == "lead_screw") {
        render_lead_screw(t.position, t.scale, rotation, selected);
    } else if (type == "pulley") {
        render_pulley(t.position, t.scale, rotation, selected);
    } else if (type == "belt") {
        render_belt(t.position, t.scale, rotation, selected);
    } else if (type == "cam") {
        render_cam(t.position, t.scale, rotation, selected);
    }
    // Sensors
    else if (type == "rotary_encoder") {
        render_encoder(t.position, t.scale, rotation, selected);
    } else if (type == "limit_switch") {
        render_limit_switch(t.position, t.scale, rotation, selected);
    } else if (type == "proximity_sensor") {
        render_proximity_sensor(t.position, t.scale, rotation, selected);
    } else if (type == "potentiometer") {
        render_potentiometer(t.position, t.scale, rotation, selected);
    } else if (type == "load_cell") {
        render_load_cell(t.position, t.scale, rotation, selected);
    } else if (type == "accelerometer") {
        render_accelerometer(t.position, t.scale, rotation, selected);
    } else if (type == "gyroscope") {
        render_gyroscope(t.position, t.scale, rotation, selected);
    } else if (type == "temperature_sensor" || type == "thermistor") {
        render_temperature_sensor(t.position, t.scale, rotation, selected);
    } else if (type == "pressure_sensor") {
        render_pressure_sensor(t.position, t.scale, rotation, selected);
    } else if (type == "flow_sensor") {
        render_flow_sensor(t.position, t.scale, rotation, selected);
    }
    // Electronics - Passive
    else if (type == "resistor") {
        render_resistor(t.position, t.scale, rotation, selected);
    } else if (type == "capacitor") {
        render_capacitor(t.position, t.scale, rotation, selected);
    } else if (type == "inductor") {
        render_inductor(t.position, t.scale, rotation, selected);
    } else if (type == "transformer") {
        render_transformer(t.position, t.scale, rotation, selected);
    }
    // Electronics - Semiconductor
    else if (type == "diode" || type == "zener_diode") {
        render_diode(t.position, t.scale, rotation, selected);
    } else if (type == "led") {
        render_led(t.position, t.scale, rotation, selected);
    } else if (type == "bjt" || type == "npn" || type == "pnp") {
        render_bjt(t.position, t.scale, rotation, selected);
    } else if (type == "mosfet" || type == "nmos" || type == "pmos") {
        render_mosfet(t.position, t.scale, rotation, selected);
    } else if (type == "igbt") {
        render_igbt(t.position, t.scale, rotation, selected);
    } else if (type == "thyristor" || type == "triac") {
        render_thyristor(t.position, t.scale, rotation, selected);
    } else if (type == "voltage_regulator") {
        render_voltage_regulator(t.position, t.scale, rotation, selected);
    } else if (type == "op_amp" || type == "operational_amplifier") {
        render_op_amp(t.position, t.scale, rotation, selected);
    }
    // Electronics - Power
    else if (type == "h_bridge" || type == "hbridge") {
        render_h_bridge(t.position, t.scale, rotation, selected);
    } else if (type == "buck_converter") {
        render_buck_converter(t.position, t.scale, rotation, selected);
    } else if (type == "boost_converter") {
        render_boost_converter(t.position, t.scale, rotation, selected);
    } else if (type == "rectifier") {
        rectifier(t.position, t.scale, rotation, selected);
    } else if (type == "motor_driver") {
        render_motor_driver(t.position, t.scale, rotation, selected);
    } else if (type == "relay") {
        render_relay(t.position, t.scale, rotation, selected);
    }
    // Electronics - Digital/Active
    else if (type == "logic_gate" || type == "and_gate" || type == "or_gate" || type == "not_gate") {
        render_logic_gate(t.position, t.scale, rotation, selected);
    } else if (type == "microcontroller" || type == "mcu") {
        render_microcontroller(t.position, t.scale, rotation, selected);
    } else if (type == "adc" || type == "dac" || type == "adc_dac") {
        render_adc_dac(t.position, t.scale, rotation, selected);
    } else if (type == "display" || type == "lcd" || type == "oled") {
        render_display(t.position, t.scale, rotation, selected);
    } else if (type == "connector" || type == "header") {
        render_connector(t.position, t.scale, rotation, selected);
    } else if (type == "terminal_block") {
        render_terminal_block(t.position, t.scale, rotation, selected);
    } else if (type == "pcb" || type == "circuit_board") {
        render_pcb(t.position, t.scale, rotation, selected);
    }
    // Software/MCU
    else if (type == "arduino_uno") {
        render_arduino_uno(t.position, t.scale, rotation, selected);
    } else if (type == "arduino_mega") {
        render_arduino_mega(t.position, t.scale, rotation, selected);
    } else if (type == "stm32" || type == "stm32_board") {
        render_stm32_board(t.position, t.scale, rotation, selected);
    } else if (type == "esp32" || type == "esp32_board") {
        render_esp32_board(t.position, t.scale, rotation, selected);
    } else if (type == "raspberry_pi" || type == "rpi") {
        render_raspberry_pi(t.position, t.scale, rotation, selected);
    }
    // Fluid Power
    else if (type == "hydraulic_cylinder") {
        render_hydraulic_cylinder(t.position, t.scale, rotation, selected);
    } else if (type == "hydraulic_pump" || type == "pump") {
        render_hydraulic_pump(t.position, t.scale, rotation, selected);
    } else if (type == "pneumatic_valve" || type == "valve") {
        render_pneumatic_valve(t.position, t.scale, rotation, selected);
    } else if (type == "compressor") {
        render_compressor(t.position, t.scale, rotation, selected);
    } else if (type == "filter") {
        render_filter(t.position, t.scale, rotation, selected);
    } else if (type == "accumulator") {
        render_accumulator(t.position, t.scale, rotation, selected);
    } else if (type == "hose" || type == "pipe") {
        render_hose(t.position, t.scale, rotation, selected);
    }
    // Multiphysics - Thermal
    else if (type == "heat_source" || type == "heater") {
        render_heat_source(t.position, t.scale, rotation, selected);
    } else if (type == "heat_sink") {
        render_heat_sink(t.position, t.scale, rotation, selected);
    } else if (type == "thermal_resistance") {
        render_thermal_resistance(t.position, t.scale, rotation, selected);
    } else if (type == "fan" || type == "cooling_fan") {
        render_fan(t.position, t.scale, rotation, selected);
    } else if (type == "thermoelectric_cooler" || type == "peltier") {
        render_thermoelectric_cooler(t.position, t.scale, rotation, selected);
    }
    // Multiphysics - Magnetic
    else if (type == "permanent_magnet" || type == "magnet") {
        render_permanent_magnet(t.position, t.scale, rotation, selected);
    } else if (type == "electromagnet") {
        render_electromagnet(t.position, t.scale, rotation, selected);
    } else if (type == "magnetic_core") {
        render_magnetic_core(t.position, t.scale, rotation, selected);
    } else if (type == "solenoid_coil") {
        render_solenoid_coil(t.position, t.scale, rotation, selected);
    }
    // Robotics
    else if (type == "robot_link" || type == "link") {
        render_robot_link(t.position, t.scale, rotation, selected);
    } else if (type == "robot_joint" || type == "joint") {
        render_robot_joint(t.position, t.scale, rotation, selected);
    } else if (type == "end_effector" || type == "effector") {
        render_end_effector(t.position, t.scale, rotation, selected);
    } else if (type == "gripper") {
        render_gripper(t.position, t.scale, rotation, selected);
    }
    // Generic fallback
    else if (comp.category() == "sensor") {
        render_sensor_box(t.position, t.scale, rotation, selected);
    } else {
        render_generic_component(t.position, t.scale, rotation, selected);
    }
}

void ComponentRenderer::render_all(const Camera& camera, float aspect) {
    if (!m_registry) return;

    m_shader.bind();
    m_shader.set_uniform("uView", camera.view_matrix());
    m_shader.set_uniform("uProj", camera.projection_matrix(aspect));
    m_shader.set_uniform("uViewPos", glm::vec3(camera.position().x, camera.position().y, camera.position().z));

    m_registry->for_each([this, &camera, aspect](Component& comp) {
        render_component(comp, camera, aspect);
    });
}

void ComponentRenderer::render_dc_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Motor body (cylinder)
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_motor.x, m_color_motor.y, m_color_motor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Shaft (smaller cylinder on top)
    glm::mat4 shaft_model = model;
    shaft_model = glm::translate(shaft_model, glm::vec3(0, 0.6f, 0));
    shaft_model = glm::scale(shaft_model, glm::vec3(0.3f, 0.4f, 0.3f));
    m_shader.set_uniform("uModel", shaft_model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.6f, 0.6f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_servo_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Servo body (box)
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_motor.x, m_color_motor.y, m_color_motor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Servo horn (circle)
    glm::mat4 horn_model = model;
    horn_model = glm::translate(horn_model, glm::vec3(0, 0.55f, 0));
    horn_model = glm::scale(horn_model, glm::vec3(0.8f, 0.1f, 0.8f));
    m_shader.set_uniform("uModel", horn_model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_solenoid(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Coil (cylinder)
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_actuator.x, m_color_actuator.y, m_color_actuator.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Plunger (smaller cylinder)
    glm::mat4 plunger_model = model;
    plunger_model = glm::translate(plunger_model, glm::vec3(0, -0.4f, 0));
    plunger_model = glm::scale(plunger_model, glm::vec3(0.4f, 0.3f, 0.4f));
    m_shader.set_uniform("uModel", plunger_model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.7f, 0.7f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_encoder(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Disk (flattened cylinder)
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Marker
    glm::mat4 marker_model = model;
    marker_model = glm::translate(marker_model, glm::vec3(0.3f, 0, 0));
    marker_model = glm::scale(marker_model, glm::vec3(0.2f, 1.1f, 0.2f));
    m_shader.set_uniform("uModel", marker_model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_sensor_box(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.3f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_generic_component(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.5f, scale.z * 0.5f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_generic.x, m_color_generic.y, m_color_generic.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

// ============================================================================
// MECHANICS - MACHINE ELEMENTS
// ============================================================================

void ComponentRenderer::render_stepper_motor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Motor body (box with square shape)
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_motor.x, m_color_motor.y, m_color_motor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Shaft
    glm::mat4 shaft_model = model;
    shaft_model = glm::translate(shaft_model, glm::vec3(0, 0.55f, 0));
    shaft_model = glm::scale(shaft_model, glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uModel", shaft_model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.6f, 0.6f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_linear_actuator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y, scale.z));

    // Cylinder body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_actuator.x, m_color_actuator.y, m_color_actuator.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Rod
    glm::mat4 rod_model = model;
    rod_model = glm::translate(rod_model, glm::vec3(0, 0.8f, 0));
    rod_model = glm::scale(rod_model, glm::vec3(0.3f, 0.4f, 0.3f));
    m_shader.set_uniform("uModel", rod_model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.7f, 0.7f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_gear(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected, int teeth) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.3f, scale.z));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.35f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Center hole
    glm::mat4 hole_model = model;
    hole_model = glm::scale(hole_model, glm::vec3(0.2f, 1.1f, 0.2f));
    m_shader.set_uniform("uModel", hole_model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_shaft(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.6f, 0.6f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_bearing(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.3f, scale.z));

    // Outer ring
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Inner ring
    glm::mat4 inner_model = model;
    inner_model = glm::scale(inner_model, glm::vec3(0.5f, 1.1f, 0.5f));
    m_shader.set_uniform("uModel", inner_model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.4f, 0.4f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_spring(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.7f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_damper(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y, scale.z * 0.3f));

    // Cylinder body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.4f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Piston rod
    glm::mat4 rod_model = model;
    rod_model = glm::translate(rod_model, glm::vec3(0, 0.6f, 0));
    rod_model = glm::scale(rod_model, glm::vec3(0.3f, 0.5f, 0.3f));
    m_shader.set_uniform("uModel", rod_model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_lead_screw(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.15f, scale.y, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_pulley(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.3f, scale.z));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_belt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.05f, scale.z));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_cam(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.3f, scale.z));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.35f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

// ============================================================================
// SENSORS
// ============================================================================

void ComponentRenderer::render_limit_switch(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.3f, scale.z * 0.5f));

    // Body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Lever
    glm::mat4 lever_model = model;
    lever_model = glm::translate(lever_model, glm::vec3(0.8f, 0, 0));
    lever_model = glm::scale(lever_model, glm::vec3(0.5f, 0.2f, 0.2f));
    m_shader.set_uniform("uModel", lever_model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_box_mesh.draw();
}

void ComponentRenderer::render_proximity_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.4f, scale.z * 0.3f));

    // Body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Sensing tip
    glm::mat4 tip_model = model;
    tip_model = glm::translate(tip_model, glm::vec3(0, -0.7f, 0));
    tip_model = glm::scale(tip_model, glm::vec3(0.5f, 0.3f, 0.5f));
    m_shader.set_uniform("uModel", tip_model);
    m_shader.set_uniform("uColor", glm::vec3(0.8f, 0.2f, 0.2f));
    m_sphere_mesh.draw();
}

void ComponentRenderer::render_potentiometer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    // Round body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Shaft
    glm::mat4 shaft_model = model;
    shaft_model = glm::translate(shaft_model, glm::vec3(0, 1.5f, 0));
    shaft_model = glm::scale(shaft_model, glm::vec3(0.3f, 1.0f, 0.3f));
    m_shader.set_uniform("uModel", shaft_model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_load_cell(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_accelerometer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.1f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_gyroscope(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.1f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_temperature_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.15f, scale.y * 0.3f, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_pressure_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_flow_sensor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.5f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(m_color_sensor.x, m_color_sensor.y, m_color_sensor.z));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

// ============================================================================
// ELECTRONICS - PASSIVE
// ============================================================================

void ComponentRenderer::render_resistor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.15f, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.5f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_capacitor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.4f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.4f, 0.7f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_inductor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.3f, scale.z * 0.25f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_transformer(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.3f, scale.z * 0.4f));

    // Core
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Primary coil
    glm::mat4 coil1_model = model;
    coil1_model = glm::translate(coil1_model, glm::vec3(0, 0.6f, 0));
    coil1_model = glm::scale(coil1_model, glm::vec3(1.0f, 0.3f, 1.0f));
    m_shader.set_uniform("uModel", coil1_model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.5f, 0.2f));
    m_cylinder_mesh.draw();

    // Secondary coil
    glm::mat4 coil2_model = model;
    coil2_model = glm::translate(coil2_model, glm::vec3(0, -0.6f, 0));
    coil2_model = glm::scale(coil2_model, glm::vec3(1.0f, 0.3f, 1.0f));
    m_shader.set_uniform("uModel", coil2_model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.5f, 0.2f));
    m_cylinder_mesh.draw();
}

// ============================================================================
// ELECTRONICS - SEMICONDUCTOR
// ============================================================================

void ComponentRenderer::render_diode(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.1f, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_led(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.15f, scale.z * 0.2f));

    // Body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(1.0f, 0.2f, 0.2f)); // Red LED
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_sphere_mesh.draw();
}

void ComponentRenderer::render_bjt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.15f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_mosfet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.1f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_igbt(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.12f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_thyristor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.15f, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_voltage_regulator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.15f, scale.z * 0.25f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_op_amp(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.15f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

// ============================================================================
// ELECTRONICS - POWER
// ============================================================================

void ComponentRenderer::render_h_bridge(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.2f, scale.z * 0.4f));

    // Heat sink base
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Fins
    for (int i = 0; i < 3; i++) {
        glm::mat4 fin_model = model;
        fin_model = glm::translate(fin_model, glm::vec3(0, 0.6f + i * 0.2f, 0));
        fin_model = glm::scale(fin_model, glm::vec3(1.0f, 0.2f, 0.1f));
        m_shader.set_uniform("uModel", fin_model);
        m_shader.set_uniform("uColor", glm::vec3(0.15f, 0.15f, 0.15f));
        m_box_mesh.draw();
    }
}

void ComponentRenderer::render_buck_converter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.15f, scale.z * 0.3f));

    // PCB
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.3f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Inductor
    glm::mat4 inductor_model = model;
    inductor_model = glm::translate(inductor_model, glm::vec3(0, 0.5f, 0));
    inductor_model = glm::scale(inductor_model, glm::vec3(0.3f, 0.5f, 0.3f));
    m_shader.set_uniform("uModel", inductor_model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.1f, 0.1f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_boost_converter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.15f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.3f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::rectifier(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.15f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_motor_driver(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.15f, scale.z * 0.4f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_relay(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.2f, scale.z * 0.3f));

    // Cube body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

// ============================================================================
// ELECTRONICS - DIGITAL/ACTIVE
// ============================================================================

void ComponentRenderer::render_logic_gate(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.1f, scale.z * 0.15f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_microcontroller(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.1f, scale.z * 0.5f));

    // Chip body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_adc_dac(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.1f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_display(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.6f, scale.y * 0.05f, scale.z * 0.4f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.2f)); // Blue tint
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_connector(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.1f, scale.z * 0.4f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_terminal_block(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.15f, scale.z * 0.2f));

    // Green terminal block
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.4f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_pcb(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.05f, scale.z));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.3f, 0.1f)); // Green PCB
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

// ============================================================================
// SOFTWARE/MCU BOARDS
// ============================================================================

void ComponentRenderer::render_arduino_uno(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.7f, scale.y * 0.05f, scale.z * 0.5f));

    // Blue PCB
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.3f, 0.5f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Black USB connector
    glm::mat4 usb_model = model;
    usb_model = glm::translate(usb_model, glm::vec3(0, 1.0f, -0.7f));
    usb_model = glm::scale(usb_model, glm::vec3(0.15f, 1.5f, 0.3f));
    m_shader.set_uniform("uModel", usb_model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_box_mesh.draw();

    // Black power jack
    glm::mat4 power_model = model;
    power_model = glm::translate(power_model, glm::vec3(0, 1.0f, 0.7f));
    power_model = glm::scale(power_model, glm::vec3(0.2f, 1.5f, 0.3f));
    m_shader.set_uniform("uModel", power_model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_box_mesh.draw();
}

void ComponentRenderer::render_arduino_mega(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x, scale.y * 0.05f, scale.z * 0.7f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.3f, 0.5f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_stm32_board(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.5f, scale.y * 0.05f, scale.z * 0.7f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // USB connector
    glm::mat4 usb_model = model;
    usb_model = glm::translate(usb_model, glm::vec3(0, 1.0f, -0.6f));
    usb_model = glm::scale(usb_model, glm::vec3(0.2f, 1.5f, 0.2f));
    m_shader.set_uniform("uModel", usb_model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.4f, 0.4f));
    m_box_mesh.draw();
}

void ComponentRenderer::render_esp32_board(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.05f, scale.z * 0.5f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.1f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Metal shield
    glm::mat4 shield_model = model;
    shield_model = glm::translate(shield_model, glm::vec3(0, 1.0f, 0));
    shield_model = glm::scale(shield_model, glm::vec3(0.6f, 1.5f, 0.6f));
    m_shader.set_uniform("uModel", shield_model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_box_mesh.draw();
}

void ComponentRenderer::render_raspberry_pi(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.6f, scale.y * 0.05f, scale.z * 0.5f));

    // Green PCB
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.1f, 0.4f, 0.1f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // HDMI ports
    glm::mat4 hdmi_model = model;
    hdmi_model = glm::translate(hdmi_model, glm::vec3(-0.3f, 1.0f, 0.5f));
    hdmi_model = glm::scale(hdmi_model, glm::vec3(0.15f, 1.5f, 0.2f));
    m_shader.set_uniform("uModel", hdmi_model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.3f, 0.2f));
    m_box_mesh.draw();
}

// ============================================================================
// FLUID POWER
// ============================================================================

void ComponentRenderer::render_hydraulic_cylinder(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y, scale.z * 0.3f));

    // Cylinder body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.3f, 0.4f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Rod
    glm::mat4 rod_model = model;
    rod_model = glm::translate(rod_model, glm::vec3(0, 0.7f, 0));
    rod_model = glm::scale(rod_model, glm::vec3(0.3f, 0.4f, 0.3f));
    m_shader.set_uniform("uModel", rod_model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_hydraulic_pump(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.3f, scale.z * 0.4f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_pneumatic_valve(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_compressor(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.4f, scale.z * 0.4f));

    // Tank body
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_filter(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y * 0.3f, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.5f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_accumulator(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.5f, scale.z * 0.25f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_hose(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.1f, scale.y, scale.z * 0.1f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

// ============================================================================
// MULTIPHYSICS - THERMAL
// ============================================================================

void ComponentRenderer::render_heat_source(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.8f, 0.3f, 0.1f)); // Red for heat
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_heat_sink(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    // Base plate
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.6f, 0.6f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Fins
    for (int i = 0; i < 5; i++) {
        glm::mat4 fin_model = model;
        fin_model = glm::translate(fin_model, glm::vec3(-0.4f + i * 0.2f, 0.6f, 0));
        fin_model = glm::scale(fin_model, glm::vec3(0.1f, 0.5f, 1.0f));
        m_shader.set_uniform("uModel", fin_model);
        m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
        m_box_mesh.draw();
    }
}

void ComponentRenderer::render_thermal_resistance(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.1f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.6f, 0.4f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_fan(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.1f, scale.z * 0.4f));

    // Fan frame
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Blades
    glm::mat4 blade_model = model;
    blade_model = glm::translate(blade_model, glm::vec3(0, 0.6f, 0));
    blade_model = glm::scale(blade_model, glm::vec3(0.8f, 0.5f, 0.8f));
    m_shader.set_uniform("uModel", blade_model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.7f, 0.7f));
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_thermoelectric_cooler(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.25f, scale.y * 0.1f, scale.z * 0.25f));

    // Ceramic plates
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.9f, 0.9f, 0.9f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

// ============================================================================
// MULTIPHYSICS - MAGNETIC
// ============================================================================

void ComponentRenderer::render_permanent_magnet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.4f)); // Dark blue for magnet
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_electromagnet(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.3f, scale.z * 0.3f));

    // Copper coil
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.4f, 0.2f)); // Copper color
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();

    // Iron core
    glm::mat4 core_model = model;
    core_model = glm::scale(core_model, glm::vec3(0.4f, 1.1f, 0.4f));
    m_shader.set_uniform("uModel", core_model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f)); // Iron color
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_magnetic_core(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.4f, scale.y * 0.15f, scale.z * 0.4f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_solenoid_coil(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.3f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.7f, 0.4f, 0.2f)); // Copper color
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

// ============================================================================
// ROBOTICS
// ============================================================================

void ComponentRenderer::render_robot_link(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.2f, scale.y, scale.z * 0.2f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.4f, 0.4f, 0.4f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_cylinder_mesh.draw();
}

void ComponentRenderer::render_robot_joint(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_sphere_mesh.draw();
}

void ComponentRenderer::render_end_effector(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.2f, scale.z * 0.3f));

    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.5f, 0.5f, 0.5f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();
}

void ComponentRenderer::render_gripper(const Vec3& pos, const Vec3& scale, const Vec3& rotation, bool selected) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z));
    model = glm::scale(model, glm::vec3(scale.x * 0.3f, scale.y * 0.15f, scale.z * 0.3f));

    // Base
    m_shader.set_uniform("uModel", model);
    m_shader.set_uniform("uColor", glm::vec3(0.3f, 0.3f, 0.3f));
    m_shader.set_uniform("uSelected", selected ? 1 : 0);
    m_box_mesh.draw();

    // Left finger
    glm::mat4 left_model = model;
    left_model = glm::translate(left_model, glm::vec3(-0.4f, 0, 0));
    left_model = glm::scale(left_model, glm::vec3(0.3f, 0.5f, 0.3f));
    m_shader.set_uniform("uModel", left_model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_box_mesh.draw();

    // Right finger
    glm::mat4 right_model = model;
    right_model = glm::translate(right_model, glm::vec3(0.4f, 0, 0));
    right_model = glm::scale(right_model, glm::vec3(0.3f, 0.5f, 0.3f));
    m_shader.set_uniform("uModel", right_model);
    m_shader.set_uniform("uColor", glm::vec3(0.2f, 0.2f, 0.2f));
    m_box_mesh.draw();
}

} // namespace mechatron
