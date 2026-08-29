#include "UIApplication.hpp"
#include "Renderer.hpp"
#include "SimulationOrchestrator.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include "Theme.hpp"

namespace mechatron {

UIApplication::UIApplication() = default;
UIApplication::~UIApplication() = default;

bool UIApplication::init() {
    // GLFW init
    if (!glfwInit()) {
        spdlog::error("Failed to initialize GLFW");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(1280, 720, "MECHATRON - Simulation Engine", nullptr, nullptr);
    if (!m_window) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    // GLEW init
    if (glewInit() != GLEW_OK) {
        spdlog::error("Failed to initialize GLEW");
        return false;
    }
    spdlog::info("OpenGL {}", reinterpret_cast<const char*>(glGetString(GL_VERSION)));

    // ImGui init
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    Theme::ApplyModernDark();

    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Renderer
    m_renderer = std::make_unique<Renderer>();
    if (!m_renderer->init()) {
        spdlog::error("Failed to initialize renderer");
        return false;
    }

    // Simulation
    m_orchestrator = std::make_unique<SimulationOrchestrator>();
    m_orchestrator->load_all_plugins();

    m_viewport.init();
    m_viewport.set_circuit_editor(&m_circuit_editor);
    m_viewport.set_code_editor(&m_code_editor);
    m_viewport.set_model_editor(&m_model_editor);

    m_running = true;
    spdlog::info("MECHATRON initialized successfully");
    return true;
}

void UIApplication::shutdown() {
    m_renderer->shutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void UIApplication::run() {
    while (!glfwWindowShouldClose(m_window) && m_running) {
        glfwPollEvents();

        // Update simulation
        m_orchestrator->update();

        // ImGui new frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Menu bar in main window
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Exit")) { m_running = false; }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Simulation")) {
                auto state = m_orchestrator->time_manager().state();
                if (ImGui::MenuItem("Start", nullptr, false, state == SimulationState::Stopped)) {
                    m_orchestrator->start();
                }
                if (ImGui::MenuItem("Pause", nullptr, false, state == SimulationState::Running)) {
                    m_orchestrator->pause();
                }
                if (ImGui::MenuItem("Resume", nullptr, false, state == SimulationState::Paused)) {
                    m_orchestrator->resume();
                }
                if (ImGui::MenuItem("Stop", nullptr, false, state != SimulationState::Stopped)) {
                    m_orchestrator->stop();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Step Forward")) {
                    if (state == SimulationState::Stopped) m_orchestrator->start();
                    m_orchestrator->step();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("ImGui Demo", nullptr, &m_show_demo);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        // Main workspace window that fills the entire viewport (below menu bar)
        ImVec2 viewport_size = ImGui::GetIO().DisplaySize;
        float menu_height = 20.0f;

        ImGui::SetNextWindowPos(ImVec2(0, menu_height), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(viewport_size.x, viewport_size.y - menu_height), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(2, 2));
        ImGui::Begin("MainWorkspace", nullptr,
                     ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImVec2 content_region = ImGui::GetContentRegionAvail();

        // === TOP AREA (Left + Center + Right panels) ===
        // Calculate top height based on bottom panel height (remaining space)
        float splitter_height = 4.0f;
        float top_height = content_region.y - m_bottom_panel_height - splitter_height;

        // === LEFT PANEL (Scene Outliner) ===
        ImGui::BeginChild("SceneOutlinerPanel",
                          ImVec2(m_left_panel_width, top_height),
                          true,
                          ImGuiWindowFlags_None);
        render_component_tree();
        ImGui::EndChild();

        ImGui::SameLine();

        // Splitter between left and center
        ImGui::PushID(&m_left_panel_width);
        if (render_splitter_v(&m_left_panel_width, 200.0f, 500.0f, top_height)) {
            // Splitter changed
        }
        ImGui::PopID();

        ImGui::SameLine();

        // === CENTER PANEL (3D Viewport) ===
        float center_width = content_region.x - m_left_panel_width - m_right_panel_width - 20;
        ImGui::BeginChild("CenterViewport",
                          ImVec2(center_width, top_height),
                          true,
                          ImGuiWindowFlags_None);
        m_viewport.render(*m_renderer, *m_orchestrator);
        ImGui::EndChild();

        ImGui::SameLine();

        // Splitter between center and right
        ImGui::PushID(&m_right_panel_width);
        if (render_splitter_v(&m_right_panel_width, 200.0f, 500.0f, top_height)) {
            // Splitter changed
        }
        ImGui::PopID();

        ImGui::SameLine();

        // === RIGHT PANEL (Properties) ===
        ImGui::BeginChild("PropertiesPanel",
                          ImVec2(m_right_panel_width, top_height),
                          true,
                          ImGuiWindowFlags_None);
        m_properties.set_selected(m_orchestrator->get_selected_component());
        m_properties.render(*m_orchestrator);
        ImGui::EndChild();

        // Store splitter position for rendering later (after bottom panel)
        ImVec2 splitter_pos = ImGui::GetCursorScreenPos();

        // === BOTTOM PANELS (Tab bar - full width) ===
        // Calculate bottom panel position - it should start right after the horizontal splitter
        float bottom_y_start = top_height + splitter_height;
        ImVec2 bottom_panel_pos(0.0f, bottom_y_start);
        ImVec2 panel_size(content_region.x, m_bottom_panel_height);

        // Manually position the cursor to where the bottom panel should start
        ImGui::SetCursorPos(bottom_panel_pos);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
        ImGui::BeginChild("BottomPanelArea", panel_size, false, ImGuiWindowFlags_None);
        if (ImGui::BeginTabBar("BottomTabBar", ImGuiTabBarFlags_None)) {
            if (ImGui::BeginTabItem("Timeline")) {
                m_timeline.render(*m_orchestrator);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Serial Monitor")) {
                m_serial_monitor.render(*m_orchestrator);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();

        // === HORIZONTAL SPLITTER (render AFTER bottom panel so it appears on top) ===
        ImGui::SetCursorPos(ImVec2(0.0f, top_height));
        ImGui::PushID(&m_bottom_panel_height);
        if (render_splitter_h(&m_bottom_panel_height, 150.0f, content_region.y - 150.0f, content_region.x)) {
            // Splitter changed
        }
        ImGui::PopID();

        ImGui::End(); // MainWorkspace
        ImGui::PopStyleVar(3);

        // Demo window (optional)
        if (m_show_demo) {
            ImGui::ShowDemoWindow(&m_show_demo);
        }

        // Render
        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(m_window, &display_w, &display_h);

        // Main viewport clear (behind ImGui)
        glViewport(0, 0, display_w, display_h);
        {
            const auto& bg = Theme::CurrentPalette().bg;
            glClearColor(bg.x, bg.y, bg.z, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }
}

void UIApplication::render_component_tree() {
    // Scene panel content (no Begin/End - we're inside a tab)

    // Add Component Menu (dynamically populated from all loaded plugins)
    if (ImGui::Button("Add Component")) {
        ImGui::OpenPopup("AddComponentPopup");
    }

    if (ImGui::BeginPopup("AddComponentPopup")) {
        // Group descriptors by plugin name
        auto* plugin_host = &m_orchestrator->plugin_host();
        auto plugins = plugin_host->get_all_plugins();

        // Map: category -> vector of (plugin_name, descriptor)
        std::map<std::string, std::vector<std::pair<std::string, ComponentDescriptor>>> grouped;
        for (auto* plugin : plugins) {
            auto descs = plugin->components();
            for (auto& desc : descs) {
                grouped[desc.category].push_back({std::string(plugin->name()), desc});
            }
        }

        // Friendly category names
        static const std::map<std::string, std::string> category_labels = {
            {"actuator", "Actuators"},
            {"sensor", "Sensors"},
            {"electronic", "Electronics - Passive"},
            {"semiconductor", "Electronics - Semiconductor"},
            {"optoelectronic", "Electronics - Optoelectronic"},
            {"power", "Electronics - Power"},
            {"control", "Software - Control"},
            {"estimator", "Software - Estimator"},
            {"mcu", "Software - MCU"},
            {"thermal", "Multiphysics - Thermal"},
            {"magnetic", "Multiphysics - Magnetic"},
            {"mechanical", "Mechanics"},
            {"rendering", "Rendering"},
            {"instrument", "Instruments"}
        };

        // Counters for unique IDs
        static std::map<std::string, int> component_counters;

        // Render each category as a section
        for (auto& [cat, items] : grouped) {
            std::string label = cat;
            auto it = category_labels.find(cat);
            if (it != category_labels.end()) label = it->second;

            ImGui::TextDisabled("%s", label.c_str());
            for (auto& [plugin_name, desc] : items) {
                if (ImGui::MenuItem(desc.display_name.c_str())) {
                    std::string key = plugin_name + "/" + desc.type;
                    component_counters[key]++;
                    std::string id = desc.type + "_" + std::to_string(component_counters[key]);
                    if (auto* comp = m_orchestrator->create_component(plugin_name, desc.type, id)) {
                        comp->transform().position = {component_counters[key] * 2.0f, 0, 0};
                        spdlog::info("Added {} ({})", desc.display_name, id);
                    } else {
                        spdlog::warn("Failed to create component: {}/{}", plugin_name, desc.type);
                    }
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", desc.description.c_str());
                }
            }
            ImGui::Separator();
        }

        ImGui::EndPopup();
    }

    ImGui::Separator();

    // List all components from registry
    ImGui::Text("Components in Scene:");
    ImGui::BeginChild("ObjectsList", ImVec2(0, 0), true);

    auto& registry = m_orchestrator->registry();
    bool has_components = false;

    // Iterate through all registered components
    for (auto& [id, comp] : registry.all_components()) {
        has_components = true;
        bool selected = (m_properties.selected() == id);

        ImGui::PushID(id.c_str());
        if (ImGui::Selectable(comp->component_type().data(), selected)) {
            m_properties.set_selected(id);
        }
        ImGui::PopID();

        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s\nCategory: %s", id.c_str(), comp->category().data());
        }
    }

    if (!has_components) {
        ImGui::TextDisabled("No components yet.\nClick 'Add Component' to begin.");
    }

    ImGui::EndChild();
}

bool UIApplication::render_splitter_v(float* size, float min_size, float max_size, float height) {
    // InvisibleButton is better for splitters - it doesn't affect layout
    ImVec2 cursor_pos = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();

    float button_width = 4.0f;
    float button_height = height > 0.0f ? height : avail.y;

    ImGui::SetCursorScreenPos(ImVec2(cursor_pos.x, cursor_pos.y));
    ImGui::InvisibleButton("##SplitterV", ImVec2(button_width, button_height));

    bool hovered = ImGui::IsItemHovered();
    bool active = ImGui::IsItemActive();

    // Draw visible splitter line
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImU32 split_color = ImGui::GetColorU32(hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator);
    draw_list->AddLine(
        ImVec2(cursor_pos.x + 2.0f, cursor_pos.y),
        ImVec2(cursor_pos.x + 2.0f, cursor_pos.y + button_height),
        split_color,
        2.0f
    );

    ImGui::SetCursorScreenPos(ImVec2(cursor_pos.x, cursor_pos.y + button_height));

    if (active) {
        float delta = ImGui::GetIO().MouseDelta.x;
        // For right panel, dragging right should increase size (invert delta)
        if (size == &m_right_panel_width) {
            delta = -delta;
        }
        *size += delta;
        if (*size < min_size) *size = min_size;
        if (*size > max_size) *size = max_size;
    }

    if (hovered || active) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    return active;
}

bool UIApplication::render_splitter_h(float* size, float min_size, float max_size, float width) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Theme::CurrentPalette().surfaceHover);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, Theme::CurrentPalette().primaryActive);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));

    float button_height = 4.0f;
    float button_width = width > 0.0f ? width : ImGui::GetContentRegionAvail().x;

    // Regular Button for automatic layout
    ImGui::Button("##SplitterH", ImVec2(button_width, button_height));

    bool hovered = ImGui::IsItemHovered();
    bool active = ImGui::IsItemActive();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);

    if (active) {
        float delta = ImGui::GetIO().MouseDelta.y;
        // For bottom panel height: dragging up (negative) should increase, dragging down (positive) should decrease
        delta = -delta;
        *size += delta;
        if (*size < min_size) *size = min_size;
        if (*size > max_size) *size = max_size;
    }

    if (hovered || active) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }

    return active;
}

} // namespace mechatron
