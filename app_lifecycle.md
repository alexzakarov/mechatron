# MECHATRON Application Lifecycle

```
================================================================================
                        MECHATRON v0.1.0 APPLICATION LIFECYCLE
================================================================================

    [START]
       |
       v
+--------------------------------------------------------------+
|  main()                                    [src/app/main.cpp]|
|  1. spdlog::set_level(debug)                                 |
|  2. UIApplication app                                        |
|  3. app.init()  -----> (basarili mi?)                        |
|       |EVET                        |HAYIR                     |
|       v                            v                          |
|  4. app.run()                  return 1 (hata mesaji)        |
|  5. app.shutdown()                                            |
|  6. return 0                                                  |
+--------------------------------------------------------------+
       |
       v
    [BITIS]


================================================================================
1. INIT (BASLATMA ASAMASI)                [UIApplication::init()]
================================================================================

  +---------------------+
  | 1. GLFW Init        |  glfwInit() -> OpenGL 3.3 Core Profile
  |                     |  Window: 1280x720 "MECHATRON - Simulation Engine"
  +--------+------------+
           |
           v
  +---------------------+
  | 2. OpenGL Context   |  glfwMakeContextCurrent()
  |                     |  VSync ON (SwapInterval=1)
  |                     |  glewInit() -> GLEW hazir
  +--------+------------+
           |
           v
  +---------------------+
  | 3. ImGui Init       |  ImGui::CreateContext()
  |                     |  Keyboard navigation ON
  |                     |  Dark theme
  |                     |  ImGui_ImplGlfw_InitForOpenGL()
  |                     |  ImGui_ImplOpenGL3_Init("#version 330")
  +--------+------------+
           |
           v
  +---------------------+
  | 4. Renderer Init    |  std::make_unique<Renderer>()
  |                     |  renderer->init()
  |                     |    -> Shader'lar yuklenir
  |                     |    -> GridRenderer olusturulur
  |                     |    -> ComponentRenderer olusturulur
  |                     |    -> GizmoRenderer olusturulur
  |                     |    -> Camera: pos(0,5,10), target(0,0,0)
  +--------+------------+
           |
           v
  +---------------------+
  | 5. Simulation Init  |  std::make_unique<SimulationOrchestrator>()
  |                     |  orchestrator->load_all_plugins()
  |                     |  viewport.init()
  +--------+------------+
           |
           v
  +---------------------+
  | 6. Hazir            |  m_running = true
  +---------------------+


================================================================================
1.5 PLUGIN YUKLEME                      [SimulationOrchestrator::load_all_plugins()]
================================================================================

  load_all_plugins()
       |
       v
  +---------------------------------------------------------------+
  | Kosullu derleme (#ifdef) ile plugin'ler kaydedilir:           |
  |                                                               |
  |  [Mechanics]                                                  |
  |    MechMachineElementsPlugin                                  |
  |      -> dc_motor, servo_motor, solenoid_actuator,            |
  |         stepper_motor, linear_actuator, gear, shaft,          |
  |         bearing, spring, damper, lead_screw, pulley, belt,    |
  |         cam, proximity_sensor, limit_switch,                  |
  |         rotary_encoder, potentiometer, load_cell,             |
  |         accelerometer, gyroscope                              |
  |                                                               |
  |  [Electronics - Passive]        (ELEC_PASSIVE_ENABLED)        |
  |    ElecPassivePlugin                                           |
  |      -> resistor, capacitor, inductor, ground                 |
  |                                                               |
  |  [Electronics - Semiconductor]  (ELEC_SEMICONDUCTOR_ENABLED)  |
  |    ElecSemiconductorPlugin                                     |
  |      -> diode, zener_diode, led, bjt_npn, bjt_pnp,           |
  |         mosfet_n, mosfet_p, igbt, thyristor                   |
  |                                                               |
  |  [Electronics - Power]          (ELEC_POWER_ENABLED)          |
  |    ElecPowerPlugin                                             |
  |      -> dc_voltage, h_bridge, buck_converter, boost_converter,|
  |         motor_driver, relay                                    |
  |                                                               |
  |  [Software - MCU]               (SOFT_MCU_AVR_ENABLED)        |
  |    MCUMcuAvrPlugin                                              |
  |      -> arduino_uno, arduino_mega, stm32, esp32, rpi          |
  |                                                               |
  |  [Software - Control]           (SOFT_CONTROL_ENABLED)        |
  |    SoftControlPlugin                                            |
  |      -> pid_controller, kalman_filter, low_pass_filter,       |
  |         high_pass_filter, state_machine                       |
  |                                                               |
  |  [Multiphysics - Thermal]       (MULTI_THERMAL_ENABLED)       |
  |    MultiThermalPlugin                                           |
  |      -> heat_source, heat_sink, thermal_resistance,           |
  |         fan, thermoelectric_cooler                             |
  |                                                               |
  |  [Multiphysics - Magnetic]      (MULTI_MAGNETIC_ENABLED)      |
  |    MultiMagneticPlugin                                          |
  |      -> permanent_magnet, electromagnet, magnetic_core,       |
  |         solenoid_coil                                          |
  +---------------------------------------------------------------+
       |
       v
  Her plugin icin:
    register_plugin(plugin)
       |
       v
    PluginHost::register_plugin()
       -> plugin->on_register(host)    [lifecycle hook]
       -> m_plugins[name] = plugin     [kayit]


================================================================================
2. MAIN LOOP (ANA DONGU)                  [UIApplication::run()]
================================================================================

  while (!glfwWindowShouldClose && m_running)
       |
       |
       |========================== FRAME BASLANGICI ==========================
       |
       v
  +----------------------------+
  | 2.1 GLFW Events           |  glfwPollEvents()
  |                            |  Klavye, mouse, window olaylari
  +------------+---------------+
               |
               v
  +----------------------------+
  | 2.2 Simulation Update     |  orchestrator->update()
  |     (Detay asagida)       |
  +------------+---------------+
               |
               v
  +----------------------------+
  | 2.3 ImGui New Frame       |  ImGui_ImplOpenGL3_NewFrame()
  |                            |  ImGui_ImplGlfw_NewFrame()
  |                            |  ImGui::NewFrame()
  +------------+---------------+
               |
               v
  +----------------------------+
  | 2.4 Menu Bar              |  [File] -> Exit (m_running=false)
  |                            |  [Simulation] -> Start/Pause/Resume/Stop/Step
  |                            |  [View] -> Panel toggle'lari
  +------------+---------------+
               |
               v
  +----------------------------+
  | 2.5 UI Panel Render       |  (Detay asagida)
  +------------+---------------+
               |
               v
  +----------------------------+
  | 2.6 Finalize & Present    |  ImGui::Render()
  |                            |  glClear (dark background)
  |                            |  ImGui_ImplOpenGL3_RenderDrawData()
  |                            |  glfwSwapBuffers()
  +----------------------------+
       |
       +-----> [dongu basi] veya [BITIS]


================================================================================
2.2 SIMULATION UPDATE                   [SimulationOrchestrator::update()]
================================================================================

  update()
       |
       v
  +------------------------------------------+
  | Step 0: Time Manager                     |
  |   m_time.update()                        |
  |   dt = m_time.physics_step_size()        |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 1: Component Update                 |
  |   registry.for_each: comp.update(dt)     |
  |                                          |
  |   Her component kendi cikis port'larina  |
  |   deger yazar:                           |
  |     DCVoltageSource -> V+ = 5V           |
  |     Sensor -> OUT = distance*voltage     |
  |     MotorDriver -> OUT+ = V*duty         |
  |     CircuitComponentAdapter:             |
  |       sync_ports_to_pins()               |
  |       circuit_comp->update(dt)           |
  |       sync_pins_to_ports()               |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 2: Net Propagation                  |
  |   propagate_nets()                       |
  |                                          |
  |   Union-Find ile elektriksel aglar:      |
  |     1. Her bagli Port -> index           |
  |     2. Union-Find: bagli port'lari birlestir |
  |     3. Output port'lardan voltaji oku    |
  |     4. Ayni net'teki tum port'lara yay   |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 3: Circuit Simulation (MNA)         |
  |   step_circuit(dt)                       |
  |                                          |
  |   1. Circuit simulator lazy-init         |
  |   2. CircuitComponentAdapter'lari        |
  |      simulator'a external olarak ekle    |
  |   3. Wire baglantilarini senkronize et   |
  |   4. circuit_simulator->step(dt):        |
  |      a. build_nets() -> pin->node haritasi|
  |      b. MNA matris olustur               |
  |      c. Her component stamp() ile        |
  |         matrise katki yapar              |
  |         (conductance, voltage source,    |
  |          current source)                 |
  |      d. Newton-Raphson ile coz           |
  |      e. Gerilim ve akimlari guncelle     |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 4: Circuit-Physics Bridge           |
  |   circuit_bridge.update(dt)              |
  |                                          |
  |   Voltajlari aktuator girislerine cevir: |
  |     Elektrik -> Mekanik kuvvet/tork      |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Durum Kontrolu                           |
  |   state == Stopped?                      |
  |     |EVET -> return (fizik calismaz)     |
  |     |HAYIR -> devam                      |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 5: Component -> Physics             |
  |   registry.for_each:                     |
  |     body->position = comp.position       |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 6: Physics Step                     |
  |   physics->step(dt)                      |
  |   [Jolt Physics veya basit fizik motoru] |
  +------------------------------------------+
       |
       v
  +------------------------------------------+
  | Step 7: Physics -> Component             |
  |   registry.for_each:                     |
  |     comp.position = body->position       |
  |     comp.on_physics_update(dt)           |
  |     Aktuator: apply_to_physics(body)     |
  |       DCMotor -> tork uygula             |
  |       Solenoid -> linear kuvvet uygula   |
  |       ServoMotor -> aci uygula           |
  +------------------------------------------+


================================================================================
2.5 UI PANEL RENDER
================================================================================

  +-----------------------+
  | 2.5.1 3D Viewport     |  viewport.render(*renderer, *orchestrator)
  |                       |
  |  a. Toolbar           |  [Reset Camera] [Grid] [Axes] [Wireframe]
  |                       |  [Gizmo: Translate/Rotate/Scale]
  |                       |  [View: Perspective/Top/Front/Right]
  |                       |
  |  b. FBO olustur       |  renderer.create_framebuffer(w,h)
  |                       |  renderer.bind_framebuffer()
  |                       |
  |  c. 3D Sahne Render   |  renderer.begin_frame()
  |     - Kamera kontrol  |    handle_camera_input()
  |       Orbit: Sag tik  |    Zoom: Scroll      Pan: Orta tik
  |       WASD: Hareket   |    F: Odaklan        Home: Sifirla
  |     - Grid/Axes       |    grid->render()
  |     - Component'ler   |    comp_renderer->render_all()
  |       (Her tur icin   |      resistor: kutu + renk bantlari
  |        ozel 3D model) |      capacitor: silindir
  |                       |      dc_motor: silindir + shaft
  |                       |      led: kure + isik efekti
  |     - Gizmo           |    render_gizmo()
  |                       |
  |  d. FBO -> ImGui      |  ImGui::Image(fbo_texture)
  |                       |
  |  e. Gizmo Input       |  handle_gizmo_input_after_image()
  |     Tiklama -> ray    |    Ray-sphere intersection ile secim
  |     Surukleme -> delta|    Translate/Rotate/Scale transform
  |                       |
  |  f. Secim             |  handle_selection()
  |     Sol tik: ray-pick |    -> orchestrator->set_selected_component()
  |     Sag tik: menu     |    -> [Delete] [Focus] [Deselect]
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.2 Timeline        |  timeline.render(*orchestrator)
  |                       |  Simulasyon zaman kontrolu
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.3 Properties      |  properties.render(*orchestrator)
  |                       |  Secili component'in:
  |                       |    - Transform (pos, rot, scale)
  |                       |    - Parametreler (resistance, voltage...)
  |                       |    - Port degerleri
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.4 Scene Tree      |  render_component_tree()
  |                       |
  |  [Add Component] butonu:
  |    -> Plugin'lerden kategorile:
  |       Actuators | Sensors | Electronics-Passive
  |       Electronics-Semiconductor | Electronics-Power
  |       Software-Control | Software-MCU
  |       Multiphysics-Thermal | Multiphysics-Magnetic
  |    -> Secim: create_component(plugin, type, id)
  |       -> PluginHost::create_component()
  |       -> Registry::add()
  |       -> Physics body olustur (pozisyon varsa)
  |
  |  Component Listesi:
  |    registry.all_components()
  |    Tikla -> properties.set_selected()
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.5 Circuit Editor  |  (opsiyonel: m_show_circuit_editor)
  |  circuit_editor.render|
  |                       |  2D sematik gosterim
  |                       |  Bilesen surukle-birak
  |                       |  Wire baglantilari
  |                       |  Pin voltaj/akim gosterimi
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.6 Code Editor     |  (opsiyonel: m_show_code_editor)
  |  code_editor.render   |  MCU firmware duzenleme
  +-----------+-----------+
              |
              v
  +-----------------------+
  | 2.5.7 Serial Monitor  |  (opsiyonel: m_show_serial_monitor)
  |  serial_monitor.render|  Seri haberlesme ciktisi
  +-----------------------+


================================================================================
3. SIMULATION STATE MACHINE                   [SimulationState enum class]
================================================================================

                    +----------+
                    | STOPPED  | <----- (baslangic durumu)
                    +----+-----+
                         |
                    start()
                         |
                         v
                    +----------+
              +-----| RUNNING  |
      pause() |     +----+-----+
              |          |
              v          | resume()
        +----------+     |
        |  PAUSED  | <---+
        +----+-----+
             |
        resume()
             |
             v
        +----------+
        | RUNNING  |
        +----+-----+
             |
        stop()
             |
             v
        +----------+
        | STOPPED  |
        +----------+

  Gecis Metotlari:
    start()  -> PhysicsWorld olustur, body'leri ekle, TimeManager.start(), Event:SimStart
    pause()  -> TimeManager.pause(), Event:SimPause
    resume() -> TimeManager.resume(), Event:SimResume
    stop()   -> TimeManager.stop(), Event:SimStop
    step()   -> TimeManager.step() (manuel ilerletme)

  Menu'den kontrol:
    Start  -> sadece Stopped durumunda aktif
    Pause  -> sadece Running durumunda aktif
    Resume -> sadece Paused durumunda aktif
    Stop   -> Stopped disinda her durumda aktif


================================================================================
4. COMPONENT OLUSTURMA AKISI       [UIApplication::render_component_tree()]
================================================================================

  Kullanici: [Add Component] -> [kategori] -> [bilesen adi]
       |
       v
  orchestrator->create_component(plugin_name, type, id)
       |
       v
  PluginHost::create_component(plugin_name, type)
       |
       v
  Plugin->create(type)  [Factory pattern]
       |
       +-- ElecPassivePlugin:       ResistorComponent(make_unique<Resistor>())
       +-- ElecSemiconductorPlugin: DiodeComponent(make_unique<Diode>())
       +-- ElecPowerPlugin:         DCVoltageComponent(make_unique<DCVoltageSource>())
       +-- SoftControlPlugin:       ControlAlgorithmAdapter<PIDController>()
       +-- MCUMcuAvrPlugin:         MCUComponent(make_unique<ATmegaInterpreter>())
       +-- ...
       |
       v
  Component* ptr = Registry::add(unique_ptr<Component>, id)
       |
       v
  Physics body olustur (physics world varsa)
       |
       v
  Sahneye eklenir -> Scene Tree'de gorunur
                  -> 3D Viewport'ta render edilir


================================================================================
5. PORT ve BAGLANTI SISTEMI                       [Port, Connection]
================================================================================

  Port Domain'leri:              Port Direction'lari:
    Mechanical                     Input
    Electrical                     Output
    Digital                        Bidirectional
    Analog
    Fluid                    PortValue = variant<bool, float,
    Thermal                           double, int32, uint32, string>
    Data

  Baglanti Akisi:
    orchestrator->connect(source_port, target_port)
       |
       v
    Connection olusturulur
       |
       v
    propagate_nets() -> Union-Find ile voltage yayilimi
       |
       v
    step_circuit() -> MNA solver ile akim hesabi

  Ornek: DC Motor Surucusu Baglantisi

    [DC Voltage Source]                [Motor Driver]              [DC Motor]
     V+ (Output) -----> VCC (Input)     OUT+ (Output) --> V+ (Input)
     GND (Output) -----> GND (Input)     OUT- (Output) --> GND (Input)
                       PWM (Input) <-- [PWM kaynagi]

    Akis: Voltage -> Net propagation -> Circuit sim -> Physics torque


================================================================================
6. CIRCUIT SIMULATION DETAYI                 [CircuitSimulator::step()]
================================================================================

  step(dt)
       |
       v
  +----------------------------+
  | 1. build_nets()            |  Union-Find ile pin->node haritasi
  |                            |  Elektriksel aglari tanimla
  +----------------------------+
       |
       v
  +----------------------------+
  | 2. MNA Matrix Olustur      |  Her bilesen stamp() ile katki:
  |                            |
  |  [G  B] [v]   [i]         |    Resistor: add_conductance(n1,n2, 1/R)
  |  [C  D] [j] = [e]         |    Capacitor: add_conductance + current_source
  |                            |    Inductor: add_conductance(dt/L) + I_prev
  |  G = iletkenlik matrisi   |    Diode/LED: linearize (G + I_offset)
  |  B,C = voltage source     |    MOSFET: Newton-Raphson linearizasyon
  |  v = node voltajlari      |    DC Source: add_voltage_source(n+,n-,V)
  |  j = branch akimlari      |    BJT: beta * Ib -> Ic
  |  i = current kaynaklari   |
  |  e = voltage kaynaklari   |
  +----------------------------+
       |
       v
  +----------------------------+
  | 3. Newton-Raphson Cozumu   |  Iteratif cozum:
  |                            |    x_new = x - J^(-1) * F(x)
  |                            |  Yakinsama kontrolu
  +----------------------------+
       |
       v
  +----------------------------+
  | 4. Sonuclari Yaz           |  Node voltajlari -> pin.voltage
  |                            |  Branch akimlari -> pin.current
  |                            |  (Ayni instance, kopya yok)
  +----------------------------+


================================================================================
7. SHUTDOWN (KAPANMA)                       [UIApplication::shutdown()]
================================================================================

  shutdown()
       |
       v
  +----------------------------+
  | 1. Renderer Shutdown       |  OpenGL kaynaklari temizle
  |                            |  Shader, VAO, VBO, FBO sil
  +----------------------------+
       |
       v
  +----------------------------+
  | 2. ImGui Shutdown          |  ImGui_ImplOpenGL3_Shutdown()
  |                            |  ImGui_ImplGlfw_Shutdown()
  |                            |  ImGui::DestroyContext()
  +----------------------------+
       |
       v
  +----------------------------+
  | 3. GLFW Shutdown           |  glfwDestroyWindow(m_window)
  |                            |  glfwTerminate()
  +----------------------------+
       |
       v
  +----------------------------+
  | 4. Smart Pointer Temizlik  |  m_renderer (unique_ptr -> auto delete)
  |                            |  m_orchestrator (unique_ptr -> auto delete)
  |                            |    -> Registry temizlenir
  |                            |    -> PluginHost temizlenir
  |                            |    -> PhysicsWorld temizlenir
  |                            |    -> CircuitSimulator temizlenir
  +----------------------------+
       |
       v
    [BITIS]


================================================================================
TAM AKIS OZETI (TEK FRAME)
================================================================================

  glfwPollEvents()
       |
       v
  orchestrator->update()
    |-- TimeManager.update()
    |-- Component.update(dt)           [kaynaklar voltage set eder]
    |-- propagate_nets()               [Union-Find voltage yayilimi]
    |-- step_circuit(dt)               [MNA solver]
    |     |-- build_nets()
    |     |-- stamp() x N              [matris olustur]
    |     |-- Newton-Raphson solve     [gerilim/akim hesapla]
    |-- circuit_bridge.update(dt)      [voltaj -> kuvvet/tork]
    |-- [Running only:]
    |     Component -> Physics sync
    |     Physics->step(dt)
    |     Physics -> Component sync
    |     Aktuator->apply_to_physics()
       |
       v
  ImGui::NewFrame()
       |
       v
  UI Render
    |-- Menu Bar (Start/Pause/Stop/Step)
    |-- 3D Viewport (FBO -> 3D Scene -> ImGui Image)
    |     |-- Camera Input (orbit/zoom/pan/WASD)
    |     |-- Grid + Axes
    |     |-- Component Render (type-based 3D models)
    |     |-- Gizmo (Translate/Rotate/Scale)
    |     |-- Selection (ray-pick)
    |-- Timeline
    |-- Properties Panel
    |-- Scene Tree (Add Component / List)
    |-- [Circuit Editor]
    |-- [Code Editor]
    |-- [Serial Monitor]
       |
       v
  ImGui::Render()
  glClear()
  ImGui_ImplOpenGL3_RenderDrawData()
  glfwSwapBuffers()
       |
       v
  [Sonraki Frame]
```
