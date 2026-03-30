#include "Viewport3D.hpp"
#include "renderer/Renderer.hpp"
#include "renderer/GridRenderer.hpp"
#include "renderer/ComponentRenderer.hpp"
#include "renderer/GizmoRenderer.hpp"
#include "SimulationOrchestrator.hpp"
#include <imgui.h>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace mechatron {

Viewport3D::Viewport3D() = default;

Viewport3D::~Viewport3D() {
    shutdown();
}

void Viewport3D::init() {
    // Initialization handled by renderer components
}

void Viewport3D::shutdown() {
    // Cleanup handled by renderer components
}

void Viewport3D::render(Renderer& renderer, SimulationOrchestrator& orchestrator) {
    // Store pointers for toolbar/input handlers
    m_renderer = &renderer;
    m_orchestrator = &orchestrator;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("3D Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 size = ImGui::GetContentRegionAvail();
    m_width = static_cast<int>(size.x);
    m_height = static_cast<int>(size.y);
    m_hovered = ImGui::IsWindowHovered();
    m_focused = ImGui::IsWindowFocused();

    // Render toolbar at top
    render_toolbar();

    if (m_width > 0 && m_height > 0) {
        // Create/recreate FBO if needed
        renderer.create_framebuffer(m_width, m_height);

        // Bind FBO and render 3D scene to texture
        renderer.bind_framebuffer();

        renderer.begin_frame();

        // Wireframe mode
        if (m_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }

        // Handle camera input
        handle_camera_input();

        float aspect = static_cast<float>(m_width) / std::max(1, m_height);

        // Render grid and axes
        if (auto* grid = renderer.get_grid_renderer()) {
            grid->set_grid_visible(m_grid_visible);
            grid->set_axes_visible(m_axes_visible);
            grid->render(renderer.camera().view_matrix(), renderer.projection_matrix());
        }

        // Render components
        if (auto* comp_renderer = renderer.get_component_renderer()) {
            comp_renderer->set_registry(&orchestrator.registry());
            comp_renderer->set_selected(orchestrator.get_selected_component());
            comp_renderer->render_all(renderer.camera(), aspect);
        }

        // Render gizmo for selected component
        handle_gizmo_input(renderer, orchestrator);

        // Restore fill mode before unbinding
        if (m_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // Unbind FBO - back to default framebuffer
        renderer.unbind_framebuffer();

        // Display the rendered texture in ImGui
        ImVec2 fbo_size((float)renderer.fbo_width(), (float)renderer.fbo_height());
        ImGui::Image((ImTextureID)(intptr_t)renderer.get_color_texture(), fbo_size,
                     ImVec2(0, 1), ImVec2(1, 0));

        // Handle selection after image (so ImGui item is active)
        handle_selection();
    }

    // Context menu
    render_context_menu();

    ImGui::End();
    ImGui::PopStyleVar();
}

void Viewport3D::render_toolbar() {
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 4);
    ImGui::SameLine(8);

    // Reset camera
    if (ImGui::Button("Reset Camera")) {
        if (m_renderer) {
            m_renderer->camera().reset();
        }
    }

    ImGui::SameLine();

    // Grid toggle
    ImGui::Checkbox("Grid", &m_grid_visible);

    ImGui::SameLine();

    // Axes toggle
    ImGui::Checkbox("Axes", &m_axes_visible);

    ImGui::SameLine();

    // Wireframe toggle
    ImGui::Checkbox("Wireframe", &m_wireframe);

    ImGui::SameLine();

    // Gizmo mode selector
    const char* modes[] = {"Translate", "Rotate", "Scale"};
    ImGui::SetNextItemWidth(100);
    ImGui::Combo("##GizmoMode", &m_gizmo_mode, modes, 3);

    ImGui::SameLine();

    // View presets
    const char* views[] = {"Perspective", "Top", "Front", "Right"};
    static int current_view = 0;
    ImGui::SetNextItemWidth(100);
    if (ImGui::Combo("##ViewPreset", &current_view, views, 4)) {
        if (m_renderer) {
            auto& cam = m_renderer->camera();
            switch (current_view) {
            case 0: // Perspective
                cam.reset();
                break;
            case 1: // Top
                cam.set_position({0, 10, 0});
                cam.set_target({0, 0, 0});
                break;
            case 2: // Front
                cam.set_position({0, 0, 10});
                cam.set_target({0, 0, 0});
                break;
            case 3: // Right
                cam.set_position({10, 0, 0});
                cam.set_target({0, 0, 0});
                break;
            }
        }
    }

    ImGui::Spacing();
}

void Viewport3D::handle_camera_input() {
    if (!m_hovered || !m_renderer) return;

    auto& cam = m_renderer->camera();
    ImGuiIO& io = ImGui::GetIO();
    float dt = io.DeltaTime;

    // Orbit (right mouse drag) - look around
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        float dx = io.MouseDelta.x * 0.3f;
        float dy = io.MouseDelta.y * 0.3f;
        cam.orbit(dx, -dy);
    }

    // Zoom (scroll wheel)
    if (io.MouseWheel != 0.0f) {
        cam.zoom(io.MouseWheel * 1.0f);
    }

    // Pan (middle mouse drag)
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        float dx = io.MouseDelta.x * 0.01f;
        float dy = io.MouseDelta.y * 0.01f;
        cam.pan(-dx, dy);
    }

    // WASD movement (only when viewport focused)
    if (m_focused) {
        float forward = 0, right = 0, up = 0;

        if (ImGui::IsKeyDown(ImGuiKey_W)) forward -= 1.0f;
        if (ImGui::IsKeyDown(ImGuiKey_S)) forward += 1.0f;
        if (ImGui::IsKeyDown(ImGuiKey_D)) right -= 1.0f;
        if (ImGui::IsKeyDown(ImGuiKey_A)) right += 1.0f;
        if (ImGui::IsKeyDown(ImGuiKey_E) || ImGui::IsKeyDown(ImGuiKey_Space)) up += 1.0f;
        if (ImGui::IsKeyDown(ImGuiKey_Q) || ImGui::IsKeyDown(ImGuiKey_LeftCtrl)) up -= 1.0f;

        if (forward != 0 || right != 0 || up != 0) {
            cam.move(forward, right, up, dt);
        }

        // F - Focus on selected component
        if (ImGui::IsKeyPressed(ImGuiKey_F)) {
            if (m_orchestrator) {
                const std::string& selected = m_orchestrator->get_selected_component();
                if (!selected.empty()) {
                    Component* comp = m_orchestrator->registry().get(selected);
                    if (comp) {
                        cam.set_target(comp->transform().position);
                    }
                }
            }
        }

        // Home - Reset camera
        if (ImGui::IsKeyPressed(ImGuiKey_Home)) {
            cam.reset();
        }
    }
}

void Viewport3D::handle_selection() {
    if (!m_renderer || !m_orchestrator || !m_hovered) return;

    // Left click to select
    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        ImVec2 rect_min = ImGui::GetItemRectMin();
        ImVec2 rect_max = ImGui::GetItemRectMax();

        // Mouse position relative to viewport
        float mx = mouse_pos.x - rect_min.x;
        float my = mouse_pos.y - rect_min.y;

        // Normalize to [0,1]
        float nx = mx / (rect_max.x - rect_min.x);
        float ny = my / (rect_max.y - rect_min.y);

        auto& cam = m_renderer->camera();
        float aspect = static_cast<float>(m_width) / std::max(1, m_height);
        glm::mat4 proj = cam.projection_matrix(aspect);
        glm::mat4 view = cam.view_matrix();
        glm::mat4 inv_vp = glm::inverse(proj * view);

        // NDC from mouse
        float ndc_x = nx * 2.0f - 1.0f;
        float ndc_y = 1.0f - ny * 2.0f;

        // Ray near and far points
        glm::vec4 ray_near(ndc_x, ndc_y, -1.0f, 1.0f);
        glm::vec4 ray_far(ndc_x, ndc_y, 1.0f, 1.0f);
        glm::vec4 near_world = inv_vp * ray_near;
        glm::vec4 far_world = inv_vp * ray_far;
        near_world /= near_world.w;
        far_world /= far_world.w;

        glm::vec3 ray_origin(near_world);
        glm::vec3 ray_dir = glm::normalize(glm::vec3(far_world) - ray_origin);

        // Find closest component hit
        std::string best_id;
        float best_dist = std::numeric_limits<float>::max();

        m_orchestrator->registry().for_each([&](Component& comp) {
            auto& t = comp.transform();
            glm::vec3 center(t.position.x, t.position.y, t.position.z);
            float radius = 1.0f; // bounding sphere radius

            // Ray-sphere intersection
            glm::vec3 oc = ray_origin - center;
            float b = 2.0f * glm::dot(oc, ray_dir);
            float c = glm::dot(oc, oc) - radius * radius;
            float discriminant = b * b - 4.0f * c;

            if (discriminant >= 0.0f) {
                float t_hit = (-b - std::sqrt(discriminant)) / 2.0f;
                if (t_hit > 0.0f && t_hit < best_dist) {
                    best_dist = t_hit;
                    best_id = comp.id();
                }
            }
        });

        m_orchestrator->set_selected_component(best_id);
    }

    // Right click to open context menu (only if something is selected and not dragging)
    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !ImGui::IsMouseDragging(ImGuiMouseButton_Right) && !m_orchestrator->get_selected_component().empty()) {
        m_show_context_menu = true;
        ImGui::OpenPopup("ViewportContextMenu");
    }
}

void Viewport3D::render_context_menu() {
    if (!m_show_context_menu) return;

    if (ImGui::BeginPopup("ViewportContextMenu")) {
        const std::string& selected = m_orchestrator->get_selected_component();

        if (!selected.empty()) {
            Component* comp = m_orchestrator->registry().get(selected);
            if (comp) {
                ImGui::Text("%s", comp->component_type().data());
                ImGui::Separator();

                if (ImGui::MenuItem("Delete")) {
                    m_orchestrator->remove_component(selected);
                }

                if (ImGui::MenuItem("Focus")) {
                    m_renderer->camera().set_target(comp->transform().position);
                }

                ImGui::Separator();
            }
        }

        if (ImGui::MenuItem("Deselect All")) {
            m_orchestrator->set_selected_component("");
        }

        ImGui::EndPopup();
    } else {
        m_show_context_menu = false;
    }
}

void Viewport3D::handle_gizmo_input(Renderer& renderer, SimulationOrchestrator& orchestrator) {
    // Get selected component
    const std::string& selected = orchestrator.get_selected_component();
    if (selected.empty()) return;

    Component* comp = orchestrator.registry().get(selected);
    if (!comp) return;

    // Render gizmo at component position
    // Gizmo rendering and input handling
}

} // namespace mechatron
