add_executable(mechatron_app
    main.cpp
)

# Core libraries
target_link_libraries(mechatron_app PRIVATE
    mechatron_core
    mechatron_renderer
    mechatron_ui
    imgui::imgui
    glfw
    GLEW::GLEW
    OpenGL::GL
    spdlog::spdlog
)

# Plugin libraries (conditionally linked based on options)
if(MECHATRON_PLUGIN_MECH_MACHINE_ELEMENTS)
    target_link_libraries(mechatron_app PRIVATE mech_machine_elements_lib)
    target_compile_definitions(mechatron_app PRIVATE MECH_MACHINE_ELEMENTS_ENABLED)
endif()
if(MECHATRON_PLUGIN_ELEC_PASSIVE)
    target_link_libraries(mechatron_app PRIVATE elec_passive_lib)
    target_compile_definitions(mechatron_app PRIVATE ELEC_PASSIVE_ENABLED)
endif()
if(MECHATRON_PLUGIN_ELEC_SEMICONDUCTOR)
    target_link_libraries(mechatron_app PRIVATE elec_semiconductor_lib)
    target_compile_definitions(mechatron_app PRIVATE ELEC_SEMICONDUCTOR_ENABLED)
endif()
if(MECHATRON_PLUGIN_ELEC_POWER)
    target_link_libraries(mechatron_app PRIVATE elec_power_lib)
    target_compile_definitions(mechatron_app PRIVATE ELEC_POWER_ENABLED)
endif()
if(MECHATRON_PLUGIN_SOFT_MCU_AVR)
    target_link_libraries(mechatron_app PRIVATE soft_mcu_avr_lib)
    target_compile_definitions(mechatron_app PRIVATE SOFT_MCU_AVR_ENABLED)
endif()
if(MECHATRON_PLUGIN_SOFT_CONTROL)
    target_link_libraries(mechatron_app PRIVATE soft_control_lib)
    target_compile_definitions(mechatron_app PRIVATE SOFT_CONTROL_ENABLED)
endif()
if(MECHATRON_PLUGIN_MULTI_THERMAL)
    target_link_libraries(mechatron_app PRIVATE multi_thermal_lib)
    target_compile_definitions(mechatron_app PRIVATE MULTI_THERMAL_ENABLED)
endif()
if(MECHATRON_PLUGIN_MULTI_MAGNETIC)
    target_link_libraries(mechatron_app PRIVATE multi_magnetic_lib)
    target_compile_definitions(mechatron_app PRIVATE MULTI_MAGNETIC_ENABLED)
endif()

target_compile_features(mechatron_app PRIVATE cxx_std_20)

# MinGW: Don't link pthread explicitly (it's in libstdc++)
if(MINGW)
    target_link_options(mechatron_app PRIVATE -Wl,--allow-multiple-definition)
endif()

# Set output name
set_target_properties(mechatron_app PROPERTIES
    OUTPUT_NAME "mechatron"
)

# Copy assets to build directory
add_custom_command(TARGET mechatron_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        ${CMAKE_BINARY_DIR}/bin/assets
    COMMENT "Copying assets to build directory"
)
