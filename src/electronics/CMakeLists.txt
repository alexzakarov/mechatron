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
set(NGSPICE_FOUND FALSE)
set(_NGSPICE_ROOT "${CMAKE_SOURCE_DIR}/third_party/ngspice")

if(APPLE)
    if(EXISTS "/opt/homebrew/lib/libngspice.dylib" AND EXISTS "/opt/homebrew/include/ngspice/sharedspice.h")
        target_include_directories(mechatron_electronics PRIVATE /opt/homebrew/include)
        target_link_libraries(mechatron_electronics PUBLIC /opt/homebrew/lib/libngspice.dylib)
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        set(NGSPICE_FOUND TRUE)
        message(STATUS "Found ngspice: /opt/homebrew/lib/libngspice.dylib")
    elseif(EXISTS "/usr/local/lib/libngspice.dylib" AND EXISTS "/usr/local/include/ngspice/sharedspice.h")
        target_include_directories(mechatron_electronics PRIVATE /usr/local/include)
        target_link_libraries(mechatron_electronics PUBLIC /usr/local/lib/libngspice.dylib)
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        set(NGSPICE_FOUND TRUE)
        message(STATUS "Found ngspice: /usr/local/lib/libngspice.dylib")
    endif()
elseif(WIN32)
    set(_NGSPICE_DLL "${_NGSPICE_ROOT}/dll/libngspice-0.dll")
    set(_NGSPICE_INC "${_NGSPICE_ROOT}/include")

    if(EXISTS "${_NGSPICE_DLL}")
        target_include_directories(mechatron_electronics PUBLIC ${_NGSPICE_INC})
        set(_NGSPICE_IMPLIB "${_NGSPICE_ROOT}/dll/ngspice.lib")
        if(EXISTS "${_NGSPICE_IMPLIB}")
            target_link_libraries(mechatron_electronics PUBLIC ${_NGSPICE_IMPLIB})
        else()
            target_link_libraries(mechatron_electronics PUBLIC ${_NGSPICE_DLL})
        endif()
        target_compile_definitions(mechatron_electronics PUBLIC MECHATRON_HAVE_NGSPICE=1)
        set(NGSPICE_FOUND TRUE)
        message(STATUS "Found ngspice: ${_NGSPICE_DLL}")
    endif()
endif()

if(NOT NGSPICE_FOUND)
    message(WARNING "ngspice not found. Circuit simulation will be disabled.")
endif()

target_include_directories(mechatron_electronics
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(mechatron_electronics
    PUBLIC
        mechatron_core
        spdlog::spdlog
)

# Copy ngspice DLLs to output directory
if(WIN32 AND NGSPICE_FOUND)
    add_custom_command(TARGET mechatron_electronics POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_NGSPICE_ROOT}/dll/libngspice-0.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_NGSPICE_ROOT}/dll/libgcc_s_seh-1.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_NGSPICE_ROOT}/dll/libwinpthread-1.dll"
            $<TARGET_FILE_DIR:mechatron_electronics>/../bin
        COMMENT "Copying ngspice DLLs to output directory"
    )
endif()
