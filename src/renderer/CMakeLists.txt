add_library(mechatron_renderer STATIC
    Renderer.cpp
    Shader.cpp
    Mesh.cpp
    Camera.cpp
    LineRenderer.cpp
    GridRenderer.cpp
    ComponentRenderer.cpp
    GizmoRenderer.cpp
)

target_include_directories(mechatron_renderer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mechatron_renderer PUBLIC
    mechatron_core
    glfw
    GLEW::GLEW
    OpenGL::GL
    imgui::imgui
    glm::glm
)

target_compile_features(mechatron_renderer PUBLIC cxx_std_20)
