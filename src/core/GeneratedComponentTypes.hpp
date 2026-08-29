#pragma once

// Generated from backend/catalog/component_catalog.json by tools/generate_type_constants.py.
// Do not edit by hand; update the catalog and regenerate.

#include <string_view>

namespace mechatron::Type {
    constexpr std::string_view RESISTOR = "resistor";
    constexpr std::string_view CAPACITOR = "capacitor";
    constexpr std::string_view INDUCTOR = "inductor";
    constexpr std::string_view GROUND = "ground";
    constexpr std::string_view DIODE = "diode";
    constexpr std::string_view ZENER_DIODE = "zener_diode";
    constexpr std::string_view LED = "led";
    constexpr std::string_view BJT_NPN = "bjt_npn";
    constexpr std::string_view BJT_PNP = "bjt_pnp";
    constexpr std::string_view MOSFET_N = "mosfet_n";
    constexpr std::string_view MOSFET_P = "mosfet_p";
    constexpr std::string_view DC_VOLTAGE = "dc_voltage";
    constexpr std::string_view H_BRIDGE = "h_bridge";
    constexpr std::string_view BUCK_CONVERTER = "buck_converter";
    constexpr std::string_view BOOST_CONVERTER = "boost_converter";
    constexpr std::string_view MOTOR_DRIVER = "motor_driver";
    constexpr std::string_view SOLENOID_ACTUATOR = "solenoid_actuator";
    constexpr std::string_view DC_MOTOR = "dc_motor";
    constexpr std::string_view SERVO_MOTOR = "servo_motor";
    constexpr std::string_view PROXIMITY_SENSOR = "proximity_sensor";
    constexpr std::string_view LIMIT_SWITCH = "limit_switch";
    constexpr std::string_view ROTARY_ENCODER = "rotary_encoder";
    constexpr std::string_view POTENTIOMETER = "potentiometer";
    constexpr std::string_view LOAD_CELL = "load_cell";
    constexpr std::string_view ACCELEROMETER = "accelerometer";
    constexpr std::string_view GYROSCOPE = "gyroscope";
    constexpr std::string_view TEMPERATURE_SENSOR = "temperature_sensor";
    constexpr std::string_view PRESSURE_SENSOR = "pressure_sensor";
    constexpr std::string_view FLOW_SENSOR = "flow_sensor";
    constexpr std::string_view ATMEGA328P = "atmega328p";
    constexpr std::string_view ATMEGA2560 = "atmega2560";
    constexpr std::string_view ATTINY85 = "attiny85";
    constexpr std::string_view PID_CONTROLLER = "pid_controller";
    constexpr std::string_view PI_CONTROLLER = "pi_controller";
    constexpr std::string_view LEAD_LAG = "lead_lag";
    constexpr std::string_view STATE_SPACE = "state_space";
    constexpr std::string_view FEEDFORWARD = "feedforward";
    constexpr std::string_view CASCADE = "cascade";
    constexpr std::string_view DEADBEAT = "deadbeat";
    constexpr std::string_view KALMAN_FILTER = "kalman_filter";
    constexpr std::string_view MPC = "mpc";
    constexpr std::string_view TRAJECTORY = "trajectory";
    constexpr std::string_view HEAT_SOURCE = "heat_source";
    constexpr std::string_view HEAT_SINK = "heat_sink";
    constexpr std::string_view THERMAL_RESISTANCE = "thermal_resistance";
    constexpr std::string_view CONVECTION = "convection";
    constexpr std::string_view RADIATION = "radiation";
    constexpr std::string_view SOLENOID_FIELD = "solenoid_field";
    constexpr std::string_view PERMANENT_MAGNET = "permanent_magnet";
    constexpr std::string_view MAGNETIC_CIRCUIT = "magnetic_circuit";
    constexpr std::string_view EDDY_CURRENT = "eddy_current";
    constexpr std::string_view OSCILLOSCOPE = "oscilloscope";

    constexpr std::string_view CATEGORY_PASSIVE = "passive";
    constexpr std::string_view CATEGORY_SEMICONDUCTOR = "semiconductor";
    constexpr std::string_view CATEGORY_OPTOELECTRONIC = "optoelectronic";
    constexpr std::string_view CATEGORY_POWER = "power";
    constexpr std::string_view CATEGORY_MCU = "mcu";
    constexpr std::string_view CATEGORY_SENSOR = "sensor";
    constexpr std::string_view CATEGORY_ACTUATOR = "actuator";
    constexpr std::string_view CATEGORY_CONTROL = "control";
    constexpr std::string_view CATEGORY_ESTIMATOR = "estimator";
    constexpr std::string_view CATEGORY_INSTRUMENT = "instrument";
    constexpr std::string_view CATEGORY_THERMAL = "thermal";
    constexpr std::string_view CATEGORY_MAGNETIC = "magnetic";

    // Compatibility aliases for historical names. Prefer catalog IDs above for new code.
    constexpr std::string_view HBRIDGE = H_BRIDGE;
    constexpr std::string_view SERVO = SERVO_MOTOR;
    constexpr std::string_view ENCODER = ROTARY_ENCODER;
}
