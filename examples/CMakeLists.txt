add_executable(physics_demo
    physics_demo.cpp
)

target_link_libraries(physics_demo
    PRIVATE
        mechatron_core
        mechatron_physics
        mechatron_renderer
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(physics_debug
    physics_debug.cpp
)

target_link_libraries(physics_debug
    PRIVATE
        mechatron_core
        mechatron_physics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(physics_ground_test
    physics_ground_test.cpp
)

target_link_libraries(physics_ground_test
    PRIVATE
        mechatron_core
        mechatron_physics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(mechatronics_demo
    mechatronics_demo.cpp
)

target_link_libraries(mechatronics_demo
    PRIVATE
        mechatron_core
        mechatron_physics
        mechatron_actuators
        mechatron_sensors
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(circuit_physics_demo
    circuit_physics_demo.cpp
)

target_link_libraries(circuit_physics_demo
    PRIVATE
        mechatron_core
        mechatron_physics
        mechatron_actuators
        mechatron_sensors
        mechatron_electronics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(firmware_demo
    firmware_demo.cpp
)

target_link_libraries(firmware_demo
    PRIVATE
        mechatron_core
        mechatron_mcu
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(mcu_sim_demo
    mcu_sim_demo.cpp
)

target_link_libraries(mcu_sim_demo
    PRIVATE
        mechatron_core
        mechatron_mcu
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(cad_demo
    cad_demo.cpp
)

target_link_libraries(cad_demo
    PRIVATE
        mechatron_core
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(mechatronic_integration_demo
    mechatronic_integration_demo.cpp
)

target_link_libraries(mechatronic_integration_demo
    PRIVATE
        mechatron_core
        mechatron_mcu
        mechatron_actuators
        mechatron_sensors
        mechatron_electronics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(cad_advanced_demo
    cad_advanced_demo.cpp
)

target_link_libraries(cad_advanced_demo
    PRIVATE
        mechatron_core
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(control_algorithms_demo
    control_algorithms_demo.cpp
)

target_link_libraries(control_algorithms_demo
    PRIVATE
        mechatron_core
        soft_control_lib
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(mcu_advanced_demo
    mcu_advanced_demo.cpp
)

target_link_libraries(mcu_advanced_demo
    PRIVATE
        mechatron_core
        mechatron_mcu
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(motor_pid_demo
    motor_pid_demo.cpp
)

target_link_libraries(motor_pid_demo
    PRIVATE
        mechatron_core
        mechatron_actuators
        mechatron_sensors
        mechatron_electronics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

add_executable(ngspice_hbridge_demo
    ngspice_hbridge_demo.cpp
)

target_link_libraries(ngspice_hbridge_demo
    PRIVATE
        mechatron_core
        mechatron_electronics
        mech_machine_elements_lib
        elec_passive_lib
        elec_semiconductor_lib
        elec_power_lib
        soft_mcu_avr_lib
        soft_control_lib
        multi_thermal_lib
        multi_magnetic_lib
)

# MinGW pthread fix
if(MINGW)
    target_link_options(physics_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(physics_debug PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(physics_ground_test PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(mechatronics_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(circuit_physics_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(firmware_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(mcu_sim_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(cad_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(mechatronic_integration_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(cad_advanced_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(control_algorithms_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(mcu_advanced_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(motor_pid_demo PRIVATE -Wl,--allow-multiple-definition)
    target_link_options(ngspice_hbridge_demo PRIVATE -Wl,--allow-multiple-definition)
endif()
