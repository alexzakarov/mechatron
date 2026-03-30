add_library(mechatron_ui STATIC
    UIApplication.cpp
    Viewport3D.cpp
    PropertiesPanel.cpp
    TimelinePanel.cpp
    CircuitEditor.cpp
    CodeEditor.cpp
    SerialMonitor.cpp
)

target_include_directories(mechatron_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mechatron_ui PUBLIC
    mechatron_core
    mechatron_renderer
    imgui::imgui
    glfw
)

target_compile_features(mechatron_ui PUBLIC cxx_std_20)
