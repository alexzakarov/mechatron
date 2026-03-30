# MECHATRON - Evrensel Mekatronik Simülasyon Platformu

## Context

MECHATRON, mekanik mühendislik, elektronik mühendisliği ve gömülü yazılım alanlarının **tamamını** kapsayan, modüler ve genişletilebilir bir simülasyon platformudur. Solenoid örneği sadece bir kullanım senaryosudur — platform; bir CNC tezgahından robot koluna, fren sisteminden güç elektroniğine, PLC kontrolünden gömülü işletim sistemine kadar her türlü mekatronik sistemi simüle edebilmelidir.

**Temel Felsefe**: Plugin tabanlı modüler mimari. Her alt-disiplin bir plugin olarak implement edilir. Çekirdek sadece zaman senkronizasyonu, event sistemi ve plugin yönetimini bilir.

---

## Teknoloji Stack

| Katman | Teknoloji |
|--------|-----------|
| Dil | C++20 |
| Build | CMake + vcpkg |
| Platform | Cross-platform (Win/Linux/macOS) |
| Fizik | Jolt Physics |
| CAD Kernel | OpenCASCADE |
| MCU Emülatör | QEMU (AVR/ARM/RISC-V) |
| Devre Simülasyonu | ngspice |
| Render | OpenGL 3.3+ |
| UI | Dear ImGui |
| Pencere | GLFW |
| Lisans | GPL-3.0 |

---

## 1. Mimari Felsefe: Plugin-First

Platform üç temel dayanak üzerine inşa edilir:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MECHATRON CORE (Çekirdek)                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │  Time     │  │  Event   │  │  Plugin  │  │  Simulation       │  │
│  │  Manager  │  │  Bus     │  │  Host    │  │  Orchestrator     │  │
│  └──────────┘  └──────────┘  └──────────┘  └───────────────────┘  │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────────────────────┐ │
│  │  Entity  │  │  Project │  │  Subsystem Manager               │ │
│  │  Registry│  │  File    │  │  (Component Grouping)             │ │
│  └──────────┘  └──────────┘  └──────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
         │              │              │              │
         ▼              ▼              ▼              ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│  MECHANICS   │ │  ELECTRONICS │ │  SOFTWARE    │ │  MULTIPHYSICS│
│  PLUGIN      │ │  PLUGIN      │ │  PLUGIN      │ │  PLUGINS     │
│              │ │              │ │              │ │              │
│ ┌──────────┐ │ │ ┌──────────┐ │ │ ┌──────────┐ │ │ ┌──────────┐ │
│ │ Makine   │ │ │ │ Analog   │ │ │ │ MCU      │ │ │ │ Termal   │ │
│ │ Eleman.  │ │ │ │ Dijital  │ │ │ │ Emülatör │ │ │ │ Manyetik │ │
│ ├──────────┤ │ │ ├──────────┤ │ │ ├──────────┤ │ │ ├──────────┤ │
│ │ Akışkan  │ │ │ │ Güç      │ │ │ │ PLC      │ │ │ │ Akustik  │ │
│ │ Gücü     │ │ │ │ Elektr.  │ │ │ │ Simülatör│ │ │ │ Yapısal  │ │
│ ├──────────┤ │ │ ├──────────┤ │ │ ├──────────┤ │ │ ├──────────┤ │
│ │ Robotik  │ │ │ │ Protokol │ │ │ │ Gömülü   │ │ │ │ Optimiz. │ │
│ │ Kinemat. │ │ │ │ Simülatör│ │ │ │ OS       │ │ │ │          │ │
│ ├──────────┤ │ │ ├──────────┤ │ │ ├──────────┤ │ │ └──────────┘ │
│ │ FEM/Day. │ │ │ │ PCB      │ │ │ │ Kontrol  │ │ │              │
│ └──────────┘ │ │ └──────────┘ │ │ │ Algort.  │ │ │              │
│              │ │              │ │ └──────────┘ │ │              │
└──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘
         │              │              │              │
         └──────────────┴──────────────┴──────────────┘
                              │
                    ┌─────────▼──────────┐
                    │  SYSTEM TEMPLATES  │
                    │  (Ön tanımlı       │
                    │   sistem şablonları│
                    │   + Subsystem      │
                    │   paketleri)       │
                    └────────────────────┘
```

---

## 2. Plugin Sistemi Detayları

Her plugin `IMechatronPlugin` arayüzünü implement eder:

```cpp
// Plugin arayüzü
class IMechatronPlugin {
public:
    virtual ~IMechatronPlugin() = default;
    virtual std::string_view name() const = 0;
    virtual std::string_view version() const = 0;
    virtual std::vector<ComponentDescriptor> components() const = 0;
    virtual std::unique_ptr<Component> create(std::string_view type) = 0;
    virtual void on_register(PluginHost& host) = 0;
    virtual void on_unregister() = 0;
};
```

### Plugin Dağılımı (Modüler Büyüme)

#### A. Mechanics Plugin Grubu

| Plugin | Kapsam | Modüller |
|--------|--------|----------|
| **mech_machine_elements** | Makine elemanları | Gear (spur, helical, bevel, worm), Shaft, Bearing, Belt/Pulley, Chain/Sprocket, Cam, Linkage, Spring, Damper, Screw/Lead-screw |
| **mech_fluid_power** | Akışkan gücü | HydraulicCylinder, HydraulicPump, PneumaticValve, Compressor, Filter, Accumulator, Hose |
| **mech_robotics** | Robotik/kinematik | RobotArm (6-DOF), DeltaRobot, SCARA, CartesianPlatform, InverseKinematics solver, PathPlanner |
| **mech_fem** | Sonlu elemanlar | StressAnalysis, ThermalExpansion, ModalAnalysis, FatigueEstimation |

#### B. Electronics Plugin Grubu

| Plugin | Kapsam | Modüller |
|--------|--------|----------|
| **elec_passive** | Pasif elemanlar | Resistor, Capacitor, Inductor, Transformer |
| **elec_semiconductor** | Yarı iletkenler | Diode, Zener, LED, BJT, MOSFET, IGBT, Thyristor, TRIAC, DIAC |
| **elec_active** | Aktif devreler | OpAmp, Comparator, Oscillator, ADC, DAC, PLL |
| **elec_power** | Güç elektroniği | H-Bridge, BuckConverter, BoostConverter, Inverter, Rectifier, MotorDriver |
| **elec_digital** | Sayısal lojik | LogicGate, FlipFlop, Counter, ShiftRegister, Multiplexer, FSM |
| **elec_protocol** | İletişim protokolleri | I2CBus, SPIBus, UART, CANBus, OneWire, PWM, Ethernet |
| **elec_pcb** | PCB entegrasyonu | NetlistImporter, GerberViewer, FootprintLibrary |

#### C. Software Plugin Grubu

| Plugin | Kapsam | Modüller |
|--------|--------|----------|
| **soft_mcu_avr** | AVR mikrodenetleyiciler | ATmega328P (Uno), ATmega2560 (Mega), ATtiny serisi - QEMU backend |
| **soft_mcu_arm** | ARM Cortex-M | STM32F103, STM32F407, RP2040 - QEMU backend |
| **soft_mcu_riscv** | RISC-V | ESP32-C3, GD32VF103 - QEMU backend |
| **soft_plc** | PLC simülasyonu | LadderLogic engine, FunctionBlockDiagram, StructuredText parser |
| **soft_rtos** | Gömülü işletim sistemi | FreeRTOS task scheduler, Mutex/Semaphore, Interrupt controller |
| **soft_control** | Kontrol algoritmaları | PIDController, StateMachine, FuzzyLogic, KalmanFilter, MotionPlanner |

#### D. Multiphysics Plugin Grubu

| Plugin | Kapsam | Modüller |
|--------|--------|----------|
| **multi_thermal** | Termal analiz | HeatSource, HeatSink, ThermalResistance, Convection, Radiation |
| **multi_magnetic** | Manyetik analiz | SolenoidField, PermanentMagnet, MagneticCircuit, EddyCurrent |
| **multi_acoustic** | Akustik/titreşim | VibrationSource, AcousticEmitter, NoiseMeter |
| **multi_structural** | Yapısal analiz | Beam, Plate, Shell, ContactStress |

#### E. System Templates (Ön Tanımlı Şablonlar)

| Şablon | İçerik |
|--------|--------|
| **CNC 3-Eksen Tezgah** | Stepper motorlar + lead screw + spindle motor + limit switch + GRBL firmware |
| **Robot Kol (6-DOF)** | Servo motorlar + linkaj + inverse kinematik + ROS bridge |
| **Fren Sistemi** | Hidrolik silindir + disk + kaliper + ABS kontrol + sensör |
| **Konveyör Hattı** | DC motor + kayış-kasnak + proximity sensör + PLC ladder + HMI |
| **Drone** | BLDC motorlar × 4 + ESC + IMU + PID kontrol + batarya modeli |
| **Solenoit Vana Sistemi** | Solenoid + yay + sıvı akış modeli + Arduino kontrol |
| **Dijital Fabrika** | Çoklu konveyör + robot kol + PLC network + SCADA |

---

## 3. Subsystem (Alt-Sistem) Kavramı

Bileşenler tek tek kullanılabildiği gibi, mantıksal alt-sistemler halinde gruplanabilir:

```json
{
  "subsystem": {
    "id": "motor_drive_subsystem",
    "type": "dc_motor_drive",
    "components": [
      {"ref": "dc_motor_1", "role": "actuator"},
      {"ref": "h_bridge_1", "role": "driver"},
      {"ref": "encoder_1", "role": "feedback"},
      {"ref": "pid_ctrl_1", "role": "controller"},
      {"ref": "arduino_1", "role": "processor"}
    ],
    "internal_connections": [
      "arduino_1.d9 -> h_bridge_1.pwm_in",
      "h_bridge_1.motor_out -> dc_motor_1.power",
      "encoder_1.a_out -> arduino_1.d2",
      "encoder_1.b_out -> arduino_1.d3"
    ],
    "exposed_ports": [
      {"name": "power_supply", "type": "electrical", "pins": ["VCC", "GND"]},
      {"name": "setpoint", "type": "signal", "pin": "arduino_1.a0"},
      {"name": "mechanical_output", "type": "shaft", "ref": "dc_motor_1.shaft"}
    ]
  }
}
```

Alt-sistemler tek bir siyah kutu (black box) olarak daha büyük sistemlerde kullanılabilir.

---

## 4. Çekirdek Mimari (Core)

### 4.1 Dizin Yapısı

```
mechatron/
├── CMakeLists.txt
├── vcpkg.json
├── LICENSE                          # GPL-3.0
├── README.md
├── PLAN.md
│
├── src/
│   ├── core/                        # Çekirdek (plugin host dahil)
│   │   ├── CMakeLists.txt
│   │   ├── TimeManager.hpp/cpp      # Deterministik master clock
│   │   ├── EventBus.hpp/cpp         # Cross-domain event sistemi
│   │   ├── PluginHost.hpp/cpp       # Plugin yükleme/yönetim
│   │   ├── SimulationOrchestrator.hpp/cpp  # Simülasyon lifecycle
│   │   ├── Component.hpp            # Temel bileşen arayüzü
│   │   ├── Subsystem.hpp/cpp        # Alt-sistem gruplama
│   │   ├── Registry.hpp             # Entity/component kayıt
│   │   ├── ProjectFile.hpp/cpp      # .mtrx proje formatı
│   │   └── Transform.hpp            # 3D transform
│   │
│   ├── renderer/                    # 3D render katmanı
│   │   ├── CMakeLists.txt
│   │   ├── Renderer.hpp/cpp         # Ana renderer (shader yönetimi)
│   │   ├── Shader.hpp/cpp           # Shader derleme ve yönetim
│   │   ├── Mesh.hpp/cpp             # 3D mesh (box, sphere, grid)
│   │   ├── Camera.hpp/cpp           # Orbit kamera (pan, zoom, rotate)
│   │   ├── LineRenderer.hpp/cpp     # Devre bağlantıları için çizgi
│   │   ├── GizmoRenderer.hpp/cpp    # Transform gizmo (translate/rotate/scale)
│   │   ├── ComponentRenderer.hpp/cpp# Bileşen görselleştirme sistemi
│   │   └── GridRenderer.hpp/cpp     # Grid ve axes görselleştirme
│   │
│   ├── ui/                          # UI katmanı
│   │   ├── CMakeLists.txt
│   │   ├── UIApplication.hpp/cpp    # Ana uygulama döngüsü
│   │   ├── Viewport3D.hpp/cpp       # 3D sahne görüntüleme
│   │   ├── PropertiesPanel.hpp/cpp  # Bileşen özellikleri düzenleme
│   │   ├── TimelinePanel.hpp/cpp    # Simülasyon zaman çizelgesi
│   │   ├── ComponentPanel.hpp/cpp   # Bileşen listesi ve ekleme
│   │   ├── PluginManagerUI.hpp/cpp  # Plugin yükleme/yönetim
│   │   ├── SubsystemBrowser.hpp/cpp # Alt-sistem ağaç görünümü
│   │   ├── SerialMonitor.hpp/cpp    # UART serial monitor
│   │   ├── CircuitEditor.hpp/cpp    # Devre şeması editörü
│   │   └── CodeEditor.hpp/cpp       # Arduino sketch editörü
│   │
│   ├── app/
│   │   ├── CMakeLists.txt
│   │   └── main.cpp
│   │
│   └── plugins/                     # Tüm plugin'ler burada
│       ├── mechanics/
│       │   ├── CMakeLists.txt
│       │   ├── machine_elements/    # Dişli, şaft, rulman...
│       │   ├── fluid_power/         # Hidrolik, pnömatik...
│       │   ├── robotics/            # Robot kol, kinematik...
│       │   └── fem/                 # Sonlu elemanlar...
│       │
│       ├── electronics/
│       │   ├── CMakeLists.txt
│       │   ├── passive/             # R, C, L, transformatör
│       │   ├── semiconductor/       # BJT, MOSFET, IGBT...
│       │   ├── active/              # OpAmp, ADC, DAC...
│       │   ├── power/               # H-Bridge, converter...
│       │   ├── digital/             # Logic gate, FF, counter
│       │   ├── protocol/            # I2C, SPI, UART, CAN
│       │   └── pcb/                 # Netlist, Gerber
│       │
│       ├── software/
│       │   ├── CMakeLists.txt
│       │   ├── mcu_avr/             # Arduino Uno/Mega
│       │   ├── mcu_arm/             # STM32, RP2040
│       │   ├── mcu_riscv/           # ESP32-C3
│       │   ├── plc/                 # Ladder, FBD, ST
│       │   ├── rtos/                # FreeRTOS simülasyonu
│       │   └── control/             # PID, StateMachine, Fuzzy
│       │
│       ├── multiphysics/
│       │   ├── CMakeLists.txt
│       │   ├── thermal/             # Isı transferi
│       │   ├── magnetic/            # Manyetik alan
│       │   ├── acoustic/            # Titreşim/ses
│       │   └── structural/          # Yapısal analiz
│       │
│       └── templates/               # Sistem şablonları
│           ├── CMakeLists.txt
│           ├── cnc_3axis/
│           ├── robot_arm_6dof/
│           ├── braking_system/
│           ├── conveyor_line/
│           ├── drone_quadcopter/
│           ├── solenoid_valve/
│           └── digital_factory/
│
├── assets/
│   ├── shaders/
│   ├── fonts/
│   └── icons/
│
├── tests/
│   ├── CMakeLists.txt
│   ├── unit/
│   │   ├── test_time_manager.cpp
│   │   ├── test_event_bus.cpp
│   │   ├── test_plugin_host.cpp
│   │   ├── test_subsystem.cpp
│   │   └── test_project_file.cpp
│   └── integration/
│       ├── test_gear_train.cpp
│       ├── test_solenoid_system.cpp
│       ├── test_dc_motor_drive.cpp
│       ├── test_plc_conveyor.cpp
│       └── test_robot_arm.cpp
│
└── docs/
    ├── architecture.md
    ├── plugin_development_guide.md
    └── api_reference.md
```

### 4.2 Temel Arayüzler

```cpp
// Component.hpp
class Component {
public:
    virtual ~Component() = default;

    // Kimlik
    virtual std::string_view plugin_type() const = 0;  // Hangi plugin
    virtual std::string_view component_type() const = 0;
    virtual std::string_view category() const = 0;     // mechanical/electronic/software/multiphysics

    // Lifecycle
    virtual void on_register(Registry& reg) = 0;
    virtual void update(double dt) = 0;
    virtual void on_unregister() = 0;

    // Serileştirme
    virtual void serialize(nlohmann::json& out) const = 0;
    virtual void deserialize(const nlohmann::json& in) = 0;

    // Pin/Port sistemi (tüm domain'ler için)
    virtual std::vector<Port*> get_ports() = 0;

    // Transform (mekanik bileşenler için)
    virtual Transform& transform() { return m_transform; }

    const std::string& id() const { return m_id; }
};
```

```cpp
// Port.hpp - Tüm domain'lerdeki bağlantı noktaları
enum class PortDomain {
    Mechanical,    // Shaft, joint, surface
    Electrical,    // Pin, terminal
    Digital,       // Logic signal
    Analog,        // Voltage/current
    Fluid,         // Hydraulic/pneumatic port
    Thermal,       // Heat transfer point
    Data           // Software bus (I2C, SPI, UART, CAN)
};

class Port {
    std::string m_name;
    PortDomain m_domain;
    PinDirection m_direction;   // Input, Output, Bidirectional
    std::any m_value;           // Type-safe değer deposu
    std::vector<Connection*> m_connections;
};
```

---

## 5. Zaman Senkronizasyonu

```
SimulationOrchestrator - Master Step (1ms):

  ┌─────────────────────────────────────────────────────┐
  │  1. SOFTWARE DOMAIN                                  │
  │     MCU emulators (QEMU) → instruction budget        │
  │     PLC ladder scan cycle                             │
  │     RTOS task scheduling                              │
  │     Control algorithm execution                       │
  │     → Output: Pin states, digital/analog values       │
  ├─────────────────────────────────────────────────────┤
  │  2. ELECTRONICS DOMAIN                               │
  │     ngspice transient step                            │
  │     Protocol simulation (I2C/SPI frames)              │
  │     Power electronics switching                       │
  │     → Output: Voltages, currents, power states        │
  ├─────────────────────────────────────────────────────┤
  │  3. MULTIPHYSICS DOMAIN                               │
  │     Thermal update (heat generation from electronics)  │
  │     Magnetic field calculation (motors, solenoids)     │
  │     → Output: Forces, temperatures, field values       │
  ├─────────────────────────────────────────────────────┤
  │  4. MECHANICS DOMAIN                                 │
  │     Jolt Physics step (forces from all domains)       │
  │     Constraint solving (gears, linkages)              │
  │     Fluid power update                                │
  │     → Output: Positions, velocities, contacts          │
  ├─────────────────────────────────────────────────────┤
  │  5. FEEDBACK (Sensors)                               │
  │     Position → encoder pulses                         │
  │     Force → load cell signal                          │
  │     Temperature → thermistor resistance               │
  │     → Output: Sensor readings → Software domain input │
  ├─────────────────────────────────────────────────────┤
  │  6. RENDER                                           │
  │     Viewport update                                   │
  └─────────────────────────────────────────────────────┘

  Tekrarla → Deterministik, replay edilebilir
```

---

## 6. Event Bus - Cross-Domain İletişim

```cpp
// Event tipleri
enum class EventDomain {
    Software,       // PIN_CHANGED, FIRMWARE_LOADED, UART_DATA
    Electronics,    // VOLTAGE_CHANGED, CURRENT_CHANGED, PROTOCOL_FRAME
    Mechanics,      // FORCE_APPLIED, POSITION_CHANGED, COLLISION
    Multiphysics,   // TEMPERATURE_CHANGED, MAGNETIC_FLUX, PRESSURE
    System          // SIM_START, SIM_PAUSE, SIM_RESET, PLUGIN_LOADED
};

class EventBus {
    // Herhangi bir domain'den event yayınla
    void publish(const Event& event);

    // Belirli bir event tipine abone ol
    Subscription subscribe(EventType type, EventHandler handler);

    // Belirli bir source component'ten gelen event'leri dinle
    Subscription subscribe(std::string_view source_id, EventHandler handler);

    // Domain bazlı filtreleme
    Subscription subscribe_domain(EventDomain domain, EventHandler handler);
};
```

---

## 7. .mtrx Proje Formatı (Genişletilmiş)

```json
{
  "version": "2.0",
  "name": "CNC 3-Eksen Tezgah",
  "plugins": [
    "mech_machine_elements",
    "mech_robotics",
    "elec_power",
    "elec_semiconductor",
    "elec_protocol",
    "soft_mcu_avr",
    "soft_control",
    "multi_thermal"
  ],
  "components": [
    {
      "id": "stepper_x",
      "plugin": "mech_machine_elements",
      "type": "lead_screw",
      "params": { "pitch_mm": 5, "length_mm": 300, "diameter_mm": 12 }
    },
    {
      "id": "stepper_motor_x",
      "plugin": "elec_power",
      "type": "stepper_driver",
      "params": { "steps_per_rev": 200, "max_current_a": 2.0 }
    },
    {
      "id": "grbl_controller",
      "plugin": "soft_mcu_avr",
      "type": "arduino_uno",
      "firmware": "grbl_hex/grbl_v1.1.hex"
    },
    {
      "id": "pid_x",
      "plugin": "soft_control",
      "type": "pid_controller",
      "params": { "kp": 1.2, "ki": 0.5, "kd": 0.1 }
    }
  ],
  "connections": [
    {
      "source": "grbl_controller.d2",
      "target": "stepper_motor_x.step",
      "domain": "digital"
    },
    {
      "source": "stepper_motor_x.shaft",
      "target": "stepper_x.input_shaft",
      "domain": "mechanical"
    },
    {
      "source": "stepper_x.linear_output",
      "target": "pid_x.feedback",
      "domain": "analog"
    }
  ],
  "subsystems": [
    {
      "id": "x_axis",
      "components": ["stepper_x", "stepper_motor_x", "pid_x"],
      "exposed_ports": [
        {"name": "step_input", "ref": "stepper_motor_x.step"},
        {"name": "linear_position", "ref": "stepper_x.linear_output"}
      ]
    }
  ],
  "simulation": {
    "time_step_us": 1000,
    "duration_s": 60.0,
    "realtime_factor": 1.0
  }
}
```

---

## 8. Fazlara Göre Uygulama Yol Haritası

### Faz 1: Çekirdek Altyapı (Hafta 1-10)

| # | Görev | Detay |
|---|-------|-------|
| 1.1 | CMake + vcpkg kurulumu | Multi-module build, dependency management |
| 1.2 | Plugin sistemi | PluginHost, dynamic loading (.dll/.so), plugin API |
| 1.3 | GLFW + OpenGL context | Window management, input handling |
| 1.4 | Dear ImGui | Dockspace, panel framework, plugin manager UI |
| 1.5 | Jolt Physics wrapper | PhysicsWorld, RigidBody, Constraint basics |
| 1.6 | TimeManager | Deterministik timestep, pause/resume/step |
| 1.7 | EventBus | Cross-domain publish/subscribe |
| 1.8 | Component + Port sistemi | Base class, Port, Connection |
| 1.9 | Subsystem framework | Component grouping, black-box abstraction |
| 1.10 | Renderer basics | Mesh, Shader, Camera, grid, gizmo |
| 1.11 | Test altyapısı | Google Test, unit/integration framework |

**Çıktı**: Plugin yükleyebilen, 3D viewport'ta fizik simülasyonu yapabilen uygulama.

### Faz 2: İlk Plugin Seti - Temel Mekatronik (Hafta 11-20)

| # | Görev | Detay |
|---|-------|-------|
| 2.1 | **mech_machine_elements** plugin | RigidBody, Spring, Damper, Gear, Shaft, LeadScrew |
| 2.2 | **elec_passive** plugin | Resistor, Capacitor, Inductor |
| 2.3 | **elec_semiconductor** plugin | Diode, BJT, MOSFET |
| 2.4 | **elec_power** plugin | H-Bridge, MotorDriver, SolenoidDriver |
| 2.5 | **soft_mcu_avr** plugin | QEMU AVR backend, Arduino Uno profili, pin I/O |
| 2.6 | **soft_control** plugin | PID Controller, State Machine |
| 2.7 | **multi_magnetic** plugin | SolenoidField,电磁 kuvvet hesaplama |
| 2.8 | ngspice entegrasyonu | Subprocess, SPICE netlist generation |
| 2.9 | Pin/Port bridge sistemi | MCU pin ↔ Circuit ↔ Physics köprüsü |
| 2.10 | Sensör modelleri | LimitSwitch, RotaryEncoder, Potentiometer |

**Çıktı**: Solenoid + DC motor demo çalışır. Arduino → H-Bridge → DC Motor → Encoder → PID.

### Faz 3: Makine Elemanları ve Akışkan Gücü (Hafta 21-30)

| # | Görev |
|---|-------|
| 3.1 | **mech_machine_elements** genişletme: Dişli tipleri (spur, helical, bevel, worm), Kayış-Kasnak, Zincir, Kam, Linkaj |
| 3.2 | **mech_fluid_power** plugin: HydraulicCylinder, Pump, Valve, Pneumatic |
| 3.3 | **multi_thermal** plugin: HeatSource, HeatSink, Convection |
| 3.4 | **elec_active** plugin: OpAmp, ADC, DAC |
| 3.5 | **elec_digital** plugin: Logic gates, FlipFlop, Counter |
| 3.6 | **soft_mcu_arm** plugin: STM32, RP2040 desteği |

**Çıktı**: Dişli kutusu, hidrolik silindir, op-amp devresi simüle edilebilir.

### Faz 4: Robotik, PLC ve İletişim (Hafta 31-40)

| # | Görev |
|---|-------|
| 4.1 | **mech_robotics** plugin: RobotArm, Delta, SCARA, IK solver, Path planner |
| 4.2 | **soft_plc** plugin: Ladder logic engine, FBD, Structured Text |
| 4.3 | **elec_protocol** plugin: I2C, SPI, UART, CAN bus simülasyonu |
| 4.4 | **soft_rtos** plugin: FreeRTOS task scheduling, interrupt |
| 4.5 | **elec_pcb** plugin: Netlist import, Gerber viewer |
| 4.6 | **soft_mcu_riscv** plugin: ESP32-C3 desteği |

**Çıktı**: Robot kol kontrolü, PLC ladder programlama, CAN bus simülasyonu.

### Faz 5: Sistem Şablonları ve Mühendislik Çıktıları (Hafta 41-50)

| # | Görev |
|---|-------|
| 5.1 | Sistem şablonları: CNC, Robot Kol, Fren, Konveyör, Drone |
| 5.2 | **multi_structural** plugin: FEM basics, beam/stress analysis |
| 5.3 | **multi_acoustic** plugin: Vibration, noise |
| 5.4 | Mühendislik çıktıları: BOM, maliyet analizi, tolerans raporu |
| 5.5 | OpenCASCADE entegrasyonu: STEP/IGES/STL import/export |
| 5.6 | Parametrik modelleme DSL |
| 5.7 | .mtrx proje formatı finalize |
| 5.8 | Subsystem browser ve template gallery UI |

**Çıktı**: Tam özellikli platform. Şablondan proje oluşturma, CAD import, BOM export.

### Faz 6: İleri Özellikler (Sürekli)

- **Digital Factory**: Çoklu sistem senkronizasyonu, SCADA benzeri HMI
- ROS/ROS2 bridge
- ML-assisted optimizasyon (motor boyutlandırma, PID tuning)
- Collaborative simulation (multi-user)
- Hardware-in-the-Loop (HIL) desteği
- Görsel programlama (node-based system design)
- Augmented Reality (AR) görselleştirme

---

## 9. Test Stratejisi

| Seviye | Kapsam |
|--------|--------|
| Unit | Her component izole, plugin API, EventBus, TimeManager |
| Integration | Cross-domain: MCU↔Devre↔Fizik, protokol simülasyonu |
| System | Tam sistem senaryoları: CNC tezgah, robot kol |
| Plugin Compatibility | Her plugin çekirdek API'ye uyumlu mu |
| Performance | Fizik step < 1ms, MCU step < 0.5ms, 100+ body @ 60 FPS |
| Determinism | Same input → same output (replay test) |

---

## 10. Doğrulama Kriterleri

### Faz 1
- [ ] CMake build başarılı (Win + Linux)
- [ ] Plugin .dll/.so yükleme çalışıyor
- [ ] 3D viewport açılıyor, rigid body düşüyor
- [ ] Unit testler geçiyor

### Faz 2
- [x] Arduino blink .hex → D13 pin HIGH/LOW izlenebiliyor ✅
  - ATmegaInterpreter ile 105+ instruction implement edildi
  - Intel HEX firmware yükleme ve çalıştırma çalışıyor
  - Gerçek Arduino blink firmware'i test edildi
- [x] H-Bridge → DC Motor → Encoder → PID closed-loop çalışıyor ✅
  - HBridge sınıfı PWM ve yön kontrolü ile implement edildi
  - DC Motor encoder coupling desteği eklendi
  - RotaryEncoder angular_velocity girişi ile güncellendi
  - motor_pid_demo.cpp ile tam kapalı döngü kontrolü çalışıyor
  - Step response ve disturbance rejection testleri geçti
  - Stabil PID parametreleri (Kp=0.15, Ki=0.001, Kd=0.02)
- [x] Solenoid aktüatör fiziksel hareket simüle ediliyor ✅

### Faz 3
- [ ] Dişli çark sistemi (3 aşamalı redüktör) çalışıyor
- [ ] Hidrolik silindir + valf sistemi simüle ediliyor
- [ ] Op-amp devresi ngspice ile doğru sonuç veriyor

### Faz 4
- [ ] 6-DOF robot kol inverse kinematik ile hedefe ulaşıyor
- [ ] PLC ladder programı konveyör hattını kontrol ediyor
- [ ] I2C bus üzerinde sensör verisi okunuyor

### Faz 5
- [ ] CNC şablonundan proje oluşturuluyor, G-code çalışıyor
- [ ] STEP dosyası import edilip fizik simülasyonuna ekleniyor
- [ ] BOM ve maliyet raporu export ediliyor
