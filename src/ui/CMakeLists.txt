add_library(mechatron_ui STATIC
    UIApplication.cpp
    Viewport3D.cpp
    PropertiesPanel.cpp
    TimelinePanel.cpp
    CircuitEditor.cpp
    SchematicSymbol.cpp
    ModelEditor.cpp
    CodeEditor.cpp
    SerialMonitor.cpp
    OscilloscopePanel.cpp
)

target_include_directories(mechatron_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mechatron_ui PUBLIC
    mechatron_core
    mechatron_io
    mechatron_renderer
    instrument_plugin_lib
    imgui::imgui
    glfw
)

if(WIN32)
    target_link_libraries(mechatron_ui PUBLIC shell32 ole32 comdlg32)
endif()

target_compile_features(mechatron_ui PUBLIC cxx_std_20)
