#include "Viewport3D.hpp"
#include "renderer/Renderer.hpp"
#include "renderer/GridRenderer.hpp"
#include "renderer/ComponentRenderer.hpp"
#include "renderer/GizmoRenderer.hpp"
#include "core/Component.hpp"
#include "core/Registry.hpp"
#include "core/SimulationOrchestrator.hpp"
#include <imgui.h>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <spdlog/spdlog.h>

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

    // Dock viewport to bottom-right corner - fixed position
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoScrollbar |
                                    ImGuiWindowFlags_NoScrollWithMouse |
                                    ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoCollapse;

    // Set window to bottom-right corner
    ImGuiIO& io = ImGui::GetIO();
    float window_width = 600.0f;
    float window_height = 500.0f;
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - window_width, io.DisplaySize.y - window_height - 20), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_Always);

    ImGui::Begin("3D Viewport", nullptr, window_flags);

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

        // Store the FBO size early for gizmo rendering
        m_image_size_x = static_cast<float>(m_width);
        m_image_size_y = static_cast<float>(m_height);

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
        render_gizmo(renderer, orchestrator);

        // Restore fill mode before unbinding
        if (m_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // Unbind FBO - back to default framebuffer
        renderer.unbind_framebuffer();

        // Display the rendered texture in ImGui
        ImVec2 fbo_size((float)renderer.fbo_width(), (float)renderer.fbo_height());

        // Render the FBO texture
        ImGui::Image((ImTextureID)(intptr_t)renderer.get_color_texture(), fbo_size,
                     ImVec2(0, 1), ImVec2(1, 0));

        // Get actual image position after rendering (more accurate than CursorScreenPos)
        ImVec2 image_rect_min = ImGui::GetItemRectMin();
        m_image_pos_x = image_rect_min.x;
        m_image_pos_y = image_rect_min.y;

        // Handle gizmo input after image is rendered (ImGui item is now active)
        handle_gizmo_input_after_image(renderer, orchestrator);

        // Handle selection after gizmo (so gizmo gets first chance at clicks)
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

    // Don't select if gizmo is active (gizmo gets priority)
    if (m_gizmo_active) return;

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

void Viewport3D::render_gizmo(Renderer& renderer, SimulationOrchestrator& orchestrator) {
    // Get selected component
    const std::string& selected = orchestrator.get_selected_component();
    if (selected.empty()) return;

    Component* comp = orchestrator.registry().get(selected);
    if (!comp) return;

    // Get gizmo renderer
    auto* gizmo = renderer.get_gizmo_renderer();
    if (!gizmo) return;

    // Determine gizmo mode based on Shift key modifier
    // Default = Translate, Shift+Click = Rotate, Ctrl+Click = Scale
    ImGuiIO& io = ImGui::GetIO();

    GizmoMode mode = GizmoMode::Translate;
    if (io.KeyShift) {
        mode = GizmoMode::Rotate;
    } else if (io.KeyCtrl) {
        mode = GizmoMode::Scale;
    } else {
        // Use toolbar selection if no modifier
        if (m_gizmo_mode == 0) mode = GizmoMode::Translate;
        else if (m_gizmo_mode == 1) mode = GizmoMode::Rotate;
        else if (m_gizmo_mode == 2) mode = GizmoMode::Scale;
    }

    gizmo->set_mode(mode);

    // Get component position
    const Vec3& pos = comp->transform().position;

    // Render gizmo (use actual image size for proper mouse interaction)
    float aspect = static_cast<float>(m_width) / std::max(1, m_height);
    gizmo->render(renderer.camera(), pos, mode, aspect, m_image_size_x, m_image_size_y);
}

void Viewport3D::handle_gizmo_input_after_image(Renderer& renderer, SimulationOrchestrator& orchestrator) {
    // Get selected component
    const std::string& selected = orchestrator.get_selected_component();
    if (selected.empty()) return;

    Component* comp = orchestrator.registry().get(selected);
    if (!comp) return;

    // Get gizmo renderer
    auto* gizmo = renderer.get_gizmo_renderer();
    if (!gizmo) return;

    // Determine gizmo mode
    ImGuiIO& io = ImGui::GetIO();
    GizmoMode mode = gizmo->get_mode();
    const Vec3& pos = comp->transform().position;

    // Handle mouse input - check if mouse is over viewport content area
    ImVec2 mouse_pos = ImGui::GetMousePos();

    // Calculate mouse position relative to the rendered image
    float rel_x = mouse_pos.x - m_image_pos_x;
    float rel_y = mouse_pos.y - m_image_pos_y;

    // Check if mouse is within actual rendered image bounds (not viewport bounds)
    bool mouse_in_viewport = rel_x >= 0 && rel_x < m_image_size_x && rel_y >= 0 && rel_y < m_image_size_y;

    if (mouse_in_viewport) {
        spdlog::debug("Viewport: mouse_in_viewport TRUE, rel_x={:.1f}, rel_y={:.1f}, image_size={:.0f}x{:.0f}, gizmo_pos=({:.2f},{:.2f},{:.2f})",
                     rel_x, rel_y, m_image_size_x, m_image_size_y, pos.x, pos.y, pos.z);

        // Handle mouse buttons - only interact with gizmo if not dragging camera
        bool is_camera_drag = ImGui::IsMouseDragging(ImGuiMouseButton_Right) ||
                             ImGui::IsMouseDragging(ImGuiMouseButton_Middle);

        if (!is_camera_drag) {
            // Always update hover state first (for every frame)
            if (!m_gizmo_active) {
                gizmo->on_mouse_move(rel_x, rel_y, renderer.camera(), pos);
            }

            if (!m_gizmo_active && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                // Check if gizmo was clicked (uses hover state from on_mouse_move above)
                if (gizmo->on_mouse_down(rel_x, rel_y, renderer.camera(), pos)) {
                    m_gizmo_active = true;
                    // Store initial transform values
                    m_gizmo_start_position = comp->transform().position;
                    m_gizmo_start_scale = comp->transform().scale;
                    m_gizmo_start_rotation = comp->transform().rotation;
                }
            }

            if (m_gizmo_active && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                // During drag, update deltas directly without hover check
                // Use IsMouseDown instead of IsMouseDragging for immediate response
                gizmo->on_mouse_move(rel_x, rel_y, renderer.camera(), pos);

                // Apply transform delta to component
                if (mode == GizmoMode::Translate) {
                    Vec3 delta = gizmo->get_translation_delta();
                    comp->transform().position.x = m_gizmo_start_position.x + delta.x;
                    comp->transform().position.y = m_gizmo_start_position.y + delta.y;
                    comp->transform().position.z = m_gizmo_start_position.z + delta.z;
                } else if (mode == GizmoMode::Rotate) {
                    // Convert Euler delta (radians) to quaternion and compose with start rotation
                    Vec3 delta = gizmo->get_rotation_delta();
                    // from_euler takes (pitch, yaw, roll) which is (Y, Z, X) rotation
                    // Gizmo delta is (x, y, z) which is (roll, pitch, yaw)
                    Quat delta_rot = Quat::from_euler(delta.y, delta.z, delta.x);
                    comp->transform().rotation = m_gizmo_start_rotation * delta_rot;

                    // Debug: Log rotation delta and resulting quaternion
                    Vec3 euler = comp->transform().rotation.to_euler();
                    spdlog::debug("Gizmo ROTATION: delta=({:.3f},{:.3f},{:.3f}) -> quat=({:.3f},{:.3f},{:.3f},{:.3f}) -> euler=({:.3f},{:.3f},{:.3f}) deg",
                                  delta.x, delta.y, delta.z,
                                  comp->transform().rotation.x, comp->transform().rotation.y,
                                  comp->transform().rotation.z, comp->transform().rotation.w,
                                  euler.x * 180.0f / M_PI, euler.y * 180.0f / M_PI, euler.z * 180.0f / M_PI);
                } else if (mode == GizmoMode::Scale) {
                    Vec3 delta = gizmo->get_scale_delta();
                    comp->transform().scale.x = m_gizmo_start_scale.x * delta.x;
                    comp->transform().scale.y = m_gizmo_start_scale.y * delta.y;
                    comp->transform().scale.z = m_gizmo_start_scale.z * delta.z;
                }
            }

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                gizmo->on_mouse_up();
                m_gizmo_active = false;
            }
        }
    } else if (m_gizmo_active) {
        // Mouse left viewport while dragging - cancel gizmo
        gizmo->on_mouse_up();
        m_gizmo_active = false;
    }
}

} // namespace mechatron
