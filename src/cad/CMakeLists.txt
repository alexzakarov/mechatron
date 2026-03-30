# CAD Kernel Module
# Provides CAD geometry operations, STL/OBJ I/O, and mesh primitives

target_sources(mechatron_core
    PRIVATE
        CADKernel.cpp
        OpenCascadeWrapper.cpp
)

target_include_directories(mechatron_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Optional OpenCASCADE support
option(MECHATRON_USE_OPENCASCADE "Enable OpenCASCADE for STEP/IGES import and CSG operations" ON)

if(MECHATRON_USE_OPENCASCADE)
    find_package(OpenCASCADE REQUIRED)

    target_compile_definitions(mechatron_core PRIVATE MECHATRON_USE_OPENCASCADE)

    target_include_directories(mechatron_core
        SYSTEM PUBLIC
            ${OpenCASCADE_INCLUDE_DIR}
    )

    target_link_libraries(mechatron_core
        PUBLIC
            ${OpenCASCADE_LIBRARIES}
    )

    message(STATUS "OpenCASCADE found: ${OpenCASCADE_VERSION}")
    message(STATUS "OpenCASCADE libraries: ${OpenCASCADE_LIBRARIES}")
else()
    message(STATUS "OpenCASCADE disabled - STEP/IGES import and CSG operations will not be available")
    message(STATUS "To enable: -DMECHATRON_USE_OPENCASCADE=ON")
endif()
