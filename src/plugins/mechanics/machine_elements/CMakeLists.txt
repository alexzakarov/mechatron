# Mechanics - Machine Elements Plugin
# Provides: Gears, Shafts, Bearings, Springs, Dampers, Actuators, Sensors

add_library(mech_machine_elements INTERFACE)
target_include_directories(mech_machine_elements INTERFACE
    ${CMAKE_SOURCE_DIR}/src
)

# Source files
set(MECH_MACHINE_ELEMENTS_SOURCES
    MechMachineElementsPlugin.cpp
)

set(MECH_MACHINE_ELEMENTS_HEADERS
    MechMachineElementsPlugin.hpp
)

# Create static library for the plugin
add_library(mech_machine_elements_lib ${MECH_MACHINE_ELEMENTS_SOURCES} ${MECH_MACHINE_ELEMENTS_HEADERS})
target_link_libraries(mech_machine_elements_lib
    PUBLIC
    mechatron_core
    mechatron_physics
    mechatron_actuators
    mechatron_sensors
)

# Alias for consistency
add_library(mechatron::mech_machine_elements ALIAS mech_machine_elements_lib)
