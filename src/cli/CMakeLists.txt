add_library(mechatron_cli_lib
    MechatronCLI.cpp
)

target_include_directories(mechatron_cli_lib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_cli_lib
    PUBLIC
    mechatron_core
    mechatron_electronics
    elec_passive_lib
    elec_semiconductor_lib
    elec_power_lib
    mech_machine_elements_lib
    soft_mcu_avr_lib
    soft_control_lib
    multi_thermal_lib
    multi_magnetic_lib
    instrument_plugin_lib
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    imgui::imgui
)

target_compile_features(mechatron_cli_lib PUBLIC cxx_std_20)

# Mechatron CLI Tool
add_executable(mechatron-cli
    main.cpp
)

target_link_libraries(mechatron-cli
    PRIVATE
    mechatron_cli_lib
)

# Platform-specific libraries
if(WIN32)
    target_link_libraries(mechatron-cli PRIVATE ws2_32)
endif()

if(APPLE)
    find_library(READLINE_LIB readline)
    find_library(CURSES_LIB curses)
    target_link_libraries(mechatron-cli PRIVATE ${READLINE_LIB} ${CURSES_LIB})
elseif(UNIX)
    target_link_libraries(mechatron-cli PRIVATE readline curses)
endif()

target_compile_features(mechatron-cli PRIVATE cxx_std_20)

# Installation
install(TARGETS mechatron-cli mechatron_cli_lib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
