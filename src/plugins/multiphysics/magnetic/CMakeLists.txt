# Magnetic Physics Plugin (Solenoid Field, Permanent Magnet, Eddy Current)

add_library(multi_magnetic_lib
    MultiMagneticPlugin.cpp
    MultiMagneticPlugin.hpp
)

target_link_libraries(multi_magnetic_lib
    PUBLIC
    mechatron_core
)

target_include_directories(multi_magnetic_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_library(mechatron::multi_magnetic ALIAS multi_magnetic_lib)
