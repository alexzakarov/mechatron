#include "Actuator.hpp"
#include "core/ComponentFactory.hpp"

namespace mechatron {

// Register actuator types
REGISTER_COMPONENT(SolenoidActuator, "Solenoid Actuator", "Electromechanical linear actuator", "actuator");
REGISTER_COMPONENT(DCMotor, "DC Motor", "Rotational DC motor with encoder support", "actuator");
REGISTER_COMPONENT(ServoMotor, "Servo Motor", "Position-controlled servo motor", "actuator");

} // namespace mechatron
