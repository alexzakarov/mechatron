add_library(mechatron_io STATIC
    SerialPort.cpp
    SerialPort.hpp
)

target_include_directories(mechatron_io
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_io
    PUBLIC
        spdlog::spdlog
)

target_compile_features(mechatron_io PUBLIC cxx_std_20)

