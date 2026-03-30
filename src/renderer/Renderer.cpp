#include "Renderer.hpp"
#include "GridRenderer.hpp"
#include "ComponentRenderer.hpp"
#include "GizmoRenderer.hpp"
#include <GL/glew.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>
#include <imgui.h>

namespace mechatron {

static const char* main_vertex_src = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_projection;

out vec3 v_normal;
out vec3 v_frag_pos;

void main() {
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    v_frag_pos = world_pos.xyz;
    v_normal = mat3(transpose(inverse(u_model))) * a_normal;
    gl_Position = u_projection * u_view * world_pos;
}
)";

static const char* main_fragment_src = R"(
#version 330 core
in vec3 v_normal;
in vec3 v_frag_pos;

uniform vec3 u_color;
uniform vec3 u_light_dir;

out vec4 frag_color;

void main() {
    vec3 norm = normalize(v_normal);
    float diff = max(dot(norm, u_light_dir), 0.0);
    vec3 ambient = 0.3 * u_color;
    vec3 diffuse = 0.7 * diff * u_color;
    frag_color = vec4(ambient + diffuse, 1.0);
}
)";

static const char* line_vertex_src = R"(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_color;

uniform mat4 u_view_proj;

out vec4 v_color;

void main() {
    v_color = a_color;
    gl_Position = u_view_proj * vec4(a_position, 1.0);
}
)";

static const char* line_fragment_src = R"(
#version 330 core
in vec4 v_color;
out vec4 frag_color;

void main() {
    frag_color = v_color;
}
)";

bool Renderer::init() {
    if (!m_main_shader.load_from_source(main_vertex_src, main_fragment_src)) {
        spdlog::error("Failed to compile main shader");
        return false;
    }
    if (!m_line_shader.load_from_source(line_vertex_src, line_fragment_src)) {
        spdlog::error("Failed to compile line shader");
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // Initialize renderers
    if (!m_grid_renderer.init()) {
        spdlog::error("Failed to initialize GridRenderer");
        return false;
    }
    if (!m_component_renderer.init()) {
        spdlog::error("Failed to initialize ComponentRenderer");
        return false;
    }
    if (!m_gizmo_renderer.init()) {
        spdlog::error("Failed to initialize GizmoRenderer");
        return false;
    }

    m_grid = Mesh::create_grid(20.0f, 20);

    spdlog::info("Renderer initialized");
    return true;
}

void Renderer::shutdown() {
    for (auto& [_, obj] : m_objects) {
        obj->mesh.cleanup();
    }
    m_objects.clear();
    m_grid.cleanup();

    delete_framebuffer();

    // Shutdown renderers
    m_grid_renderer.shutdown();
    m_component_renderer.shutdown();
    m_gizmo_renderer.shutdown();
}

void Renderer::begin_frame(float r, float g, float b, float a) {
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::end_frame() {
    float aspect = static_cast<float>(m_width) / static_cast<float>(m_height);
    glm::mat4 view = camera().view_matrix();
    glm::mat4 proj = camera().projection_matrix(aspect);

    // Draw grid
    m_line_shader.bind();
    glm::mat4 vp = proj * view;
    m_line_shader.set_uniform("u_view_proj", vp);
    m_line_shader.unbind();

    m_line_renderer.begin();
    // Grid lines
    float half = 10.0f;
    uint32_t grid_color = 0xFF444444;
    for (int i = 0; i <= 20; ++i) {
        float pos = -half + i * 1.0f;
        m_line_renderer.draw_line({pos, 0, -half}, {pos, 0, half}, grid_color);
        m_line_renderer.draw_line({-half, 0, pos}, {half, 0, pos}, grid_color);
    }
    m_line_renderer.end(m_line_shader.program_id(), glm::value_ptr(vp));

    // Draw objects
    m_main_shader.bind();
    m_main_shader.set_uniform("u_view", view);
    m_main_shader.set_uniform("u_projection", proj);
    m_main_shader.set_uniform("u_light_dir", glm::vec3(0.5f, 1.0f, 0.3f));

    for (auto& [_, obj] : m_objects) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f),
            glm::vec3(obj->position.x, obj->position.y, obj->position.z));
        model = glm::scale(model,
            glm::vec3(obj->scale.x, obj->scale.y, obj->scale.z));

        m_main_shader.set_uniform("u_model", model);
        m_main_shader.set_uniform("u_color", glm::vec3(obj->color.x, obj->color.y, obj->color.z));
        obj->mesh.draw();
    }

    m_main_shader.unbind();
}

void Renderer::resize(int width, int height) {
    m_width = width;
    m_height = height;
    glViewport(0, 0, width, height);
}

RenderObject* Renderer::add_object(std::string id, Mesh mesh, Vec3 position) {
    auto obj = std::make_unique<RenderObject>();
    obj->id = id;
    obj->mesh = std::move(mesh);
    obj->position = position;
    RenderObject* ptr = obj.get();
    m_objects[id] = std::move(obj);
    return ptr;
}

void Renderer::remove_object(std::string_view id) {
    auto it = m_objects.find(std::string(id));
    if (it != m_objects.end()) {
        it->second->mesh.cleanup();
        m_objects.erase(it);
    }
}

RenderObject* Renderer::get_object(std::string_view id) {
    auto it = m_objects.find(std::string(id));
    return it != m_objects.end() ? it->second.get() : nullptr;
}

void Renderer::create_framebuffer(int width, int height) {
    if (width <= 0 || height <= 0) return;

    // Recreate if size changed
    if (m_fbo && m_fbo_width == width && m_fbo_height == height) return;

    delete_framebuffer();

    m_fbo_width = width;
    m_fbo_height = height;

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    // Color texture
    glGenTextures(1, &m_fbo_color_texture);
    glBindTexture(GL_TEXTURE_2D, m_fbo_color_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_fbo_color_texture, 0);

    // Depth renderbuffer
    glGenRenderbuffers(1, &m_fbo_depth_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_fbo_depth_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_fbo_depth_rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        spdlog::error("Framebuffer is not complete!");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::delete_framebuffer() {
    if (m_fbo_color_texture) { glDeleteTextures(1, &m_fbo_color_texture); m_fbo_color_texture = 0; }
    if (m_fbo_depth_rbo) { glDeleteRenderbuffers(1, &m_fbo_depth_rbo); m_fbo_depth_rbo = 0; }
    if (m_fbo) { glDeleteFramebuffers(1, &m_fbo); m_fbo = 0; }
    m_fbo_width = 0;
    m_fbo_height = 0;
}

void Renderer::bind_framebuffer() {
    if (!m_fbo) return;
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_fbo_width, m_fbo_height);
}

void Renderer::unbind_framebuffer() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace mechatron
