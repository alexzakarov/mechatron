add_library(mechatron_electronics STATIC
    CircuitSimulator.cpp
    CircuitSimulator.hpp
    MNASolver.hpp
    NgspiceWrapper.cpp
    NgspiceWrapper.hpp
    SpiceModelLibrary.cpp
    SpiceModelLibrary.hpp
    CircuitToSpiceConverter.cpp
    CircuitToSpiceConverter.hpp
)

# Add ngspice support if available
if(DEFINED NGSPICE_INCLUDE_DIR AND DEFINED NGSPICE_LIBRARY)
    target_include_directories(mechatron_electronics PRIVATE ${NGSPICE_INCLUDE_DIR})
    target_link_libraries(mechatron_electronics PUBLIC ${NGSPICE_LIBRARY})
    target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
elseif(APPLE)
    # Try to find ngspice via Homebrew on macOS
    # Check if library exists
    if(EXISTS "/opt/homebrew/lib/libngspice.dylib" AND EXISTS "/opt/homebrew/include/ngspice/sharedspice.h")
        target_include_directories(mechatron_electronics PRIVATE /opt/homebrew/include)
        target_link_libraries(mechatron_electronics PUBLIC /opt/homebrew/lib/libngspice.dylib)
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        message(STATUS "Found ngspice: /opt/homebrew/lib/libngspice.dylib")
    elseif(EXISTS "/usr/local/lib/libngspice.dylib" AND EXISTS "/usr/local/include/ngspice/sharedspice.h")
        target_include_directories(mechatron_electronics PRIVATE /usr/local/include)
        target_link_libraries(mechatron_electronics PUBLIC /usr/local/lib/libngspice.dylib)
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        message(STATUS "Found ngspice: /usr/local/lib/libngspice.dylib")
    else()
        message(WARNING "ngspice not found. Circuit simulation will be disabled.")
    endif()
elseif(WIN32)
    # Try to find ngspice in standard Windows locations
    find_library(NGSPICE_LIBRARY ngspice-0 HINTS
        "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/dll-mingw"
        "C:/ngspice/bin"
        "$ENV{NGSPICE_ROOT}/bin"
    )
    find_path(NGSPICE_INCLUDE_DIR sharedspice.h HINTS
        "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/include"
        "C:/ngspice/include"
        "$ENV{NGSPICE_ROOT}/include"
    )

    if(NGSPICE_LIBRARY AND NGSPICE_INCLUDE_DIR)
        target_include_directories(mechatron_electronics PRIVATE ${NGSPICE_INCLUDE_DIR})
        target_link_libraries(mechatron_electronics PUBLIC ${NGSPICE_LIBRARY})
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        message(STATUS "Found ngspice: ${NGSPICE_LIBRARY}")
    else()
        message(WARNING "ngspice not found. Circuit simulation will be disabled.")
    endif()
else()
    message(WARNING "ngspice not configured. Circuit simulation will be disabled.")
endif()

target_include_directories(mechatron_electronics
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

# Windows-specific ngspice include path (fallback if not using NGSPICE_INCLUDE_DIR)
if(WIN32)
    target_include_directories(mechatron_electronics PUBLIC
        "C:/Users/Muhammed/Downloads/ngspice-38_dll_64/Spice64_dll/include"
    )
endif()

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
