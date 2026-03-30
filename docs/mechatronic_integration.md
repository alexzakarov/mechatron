# Mechatronic Integration Implementation

## Overview
Tam mekatronik entegrasyon demo'su tamamlandı. Bu demo, MCU → Devre → Aktüatör → Fizik → Sensör → MCU geri besleme döngüsünü gösterir.

## Simulation Loop

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MECHATRONIC SIMULATION LOOP                       │
│                                                                      │
│  1. MCU Step: Digital output (D13 controls solenoid)                │
│     └──> Arduino pin state                                         │
│                                                                      │
│  2. Circuit Step: Voltage across solenoid coil                      │
│     └──> V_coil = V_in * (R_load / (R_load + R_internal))          │
│                                                                      │
│  3. Actuator Step: Solenoid generates force, moves plunger          │
│     └──> F = (N*I)² / (2*g²) * magnetic_constant                   │
│     └──> Plunger position dynamics (mass, spring, damping)          │
│                                                                      │
│  4. Physics Step: Plunger moves metal piece                         │
│     └──> F_total = F_solenoid + F_spring - F_damping               │
│     └──> a = F / m, v = v + a*dt, x = x + v*dt                     │
│                                                                      │
│  5. Sensor Step: Proximity sensor measures distance                │
│     └──> V_out = (1 - distance/max_range) * 5V                     │
│     └──> Closer = higher voltage, farther = lower voltage          │
│                                                                      │
│  6. Feedback: ADC value written to MCU memory                       │
│     └──> ADC = (distance / max_range) * 1023                       │
│     └──> Arduino can read with analogRead(A0)                      │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Demo Components

### 1. MechatronicSystem Class
Main class that orchestrates the simulation:

```cpp
class MechatronicSystem {
    QEMUInterface m_mcu;                    // Arduino simulation
    std::unique_ptr<SolenoidActuator> m_solenoid;
    std::unique_ptr<ProximitySensor> m_sensor;
    DemoPhysicsBody m_body;                 // Metal piece
};
```

### 2. Tests

#### Manual Control Test
Tests solenoid response at different input levels (0-100%):
- Input: 0.0 → 0.00V → 0.00% position
- Input: 0.5 → 2.50V → 10.55% position
- Input: 1.0 → 5.00V → 21.91% position

#### Sensor Feedback Test
Tests proximity sensor at different distances:
- 0mm → 5.0V → ADC 1023
- 10mm → 2.5V → ADC 511
- 20mm → 0.0V → ADC 0

#### Closed-Loop Control Test
Demonstrates position control using sensor feedback:
- Target: 10mm position
- Start: 18mm position
- Control: On/off with hysteresis (±1mm tolerance)
- Result: System converges to ~9.4mm

#### Full Simulation
5-second simulation with Arduino blink pattern:
- D13 toggles every 500ms
- Solenoid activates when D13 is HIGH
- Metal piece moves between positions
- Sensor continuously measures distance

## Key Improvements Made

### ProximitySensor
Added voltage conversion to `read()` method:
```cpp
float read() const override {
    float ratio = 1.0f - (m_distance / m_max_range);
    return ratio * m_max_value;  // 0-5V
}
```

### Physics Model
Added spring force and proper dynamics:
```cpp
// Solenoid pulls inward (negative z)
float solenoid_force = -m_solenoid->get_position() * 8.0f;

// Spring pushes outward to rest position
float spring_force = spring_k * (rest_position - m_body.position.z);

// Total force with damping
float total_force = solenoid_force + spring_force;
m_body.velocity.z += (total_force / m_body.mass) * TIME_STEP;
m_body.velocity.z *= 0.85f;  // Damping
```

## Sample Output

```
=== Closed-Loop Control Test ===
Goal: Maintain position at 10mm using sensor feedback

   Time(s)     Target(mm)     Actual(mm)      Error(mm)       D13
----------------------------------------------------------------------
     0.000         10.000         18.000         -8.000        ON
     0.500         10.000         10.639         -0.639       OFF
     1.000         10.000         10.297         -0.297       OFF
     1.500         10.000         10.023         -0.023       OFF
     2.000         10.000          9.740          0.260       OFF

Final position: 9.448 mm
```

## Build & Run

```bash
cmake --build build --target mechatronic_integration_demo
./build/bin/mechatronic_integration_demo
```

## Next Steps

1. **Better Control Algorithm**: Implement PID control instead of on/off
2. **QEMU Real Mode**: Complete QEMU subprocess integration
3. **CAD Visualization**: 3D rendering of solenoid and metal piece
4. **Circuit Simulation**: Add ngspice for accurate electrical simulation
5. **More Actuators**: Add DC motor, servo motor, stepper motor demos
6. **More Sensors**: Add limit switch, rotary encoder, potentiometer demos
