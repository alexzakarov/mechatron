add_executable(mechatron_tests
    unit/test_time_manager.cpp
    unit/test_event_bus.cpp
    unit/test_subsystem.cpp
    unit/test_project_file.cpp
    unit/test_physical_mcu_protocol.cpp
    unit/test_atmega_interpreter.cpp
    unit/test_cad_modifiers.cpp
    unit/test_net_propagation.cpp
)

target_link_libraries(mechatron_tests PRIVATE
    mechatron_core
    mechatron_mcu
    mechatron::elec_power
    mechatron::elec_passive
    mechatron::elec_semiconductor
    mechatron::soft_mcu_avr
    mechatron::soft_control
    mechatron::multi_thermal
    mechatron::multi_magnetic
    mechatron::instrument
    mechatron::mech_machine_elements
    GTest::gtest
    GTest::gtest_main
)

target_compile_features(mechatron_tests PRIVATE cxx_std_20)

# MinGW: Don't link pthread explicitly (it's in libstdc++)
if(MINGW)
    target_link_options(mechatron_tests PRIVATE -Wl,--allow-multiple-definition)
endif()

# Disable test discovery during build (run manually)
# include(GoogleTest)
# gtest_discover_tests(mechatron_tests)

# Circuit port tests (standalone, no GTest)
add_executable(test_circuit_ports
    test_circuit_ports.cpp
)

target_link_libraries(test_circuit_ports PRIVATE
    mechatron_electronics
    mechatron_core
    spdlog::spdlog
)

target_compile_features(test_circuit_ports PRIVATE cxx_std_20)
