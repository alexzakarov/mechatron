add_library(mechatron_electronics STATIC
    CircuitSimulator.cpp
    CircuitSimulator.hpp
    NgspiceWrapper.cpp
    NgspiceWrapper.hpp
)

target_include_directories(mechatron_electronics
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/include"
)

target_link_libraries(mechatron_electronics
    PUBLIC
        mechatron_core
        spdlog::spdlog
)

# Windows-specific: copy ngspice DLL to bin directory
if(WIN32)
    # Copy required DLLs to output directory
    add_custom_command(TARGET mechatron_electronics POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/dll-mingw/libngspice-0.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/dll-mingw/libgcc_s_seh-1.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/dll-mingw/libwinpthread-1.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMENT "Copying ngspice DLLs to output directory"
    )
endif()
