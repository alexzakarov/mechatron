#include "UIApplication.hpp"
#include "Renderer.hpp"
#include "SimulationOrchestrator.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>

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
                ImGui::MenuItem("Circuit Editor", nullptr, &m_show_circuit_editor);
                ImGui::MenuItem("Code Editor", nullptr, &m_show_code_editor);
                ImGui::MenuItem("Serial Monitor", nullptr, &m_show_serial_monitor);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        // Panels
        m_viewport.render(*m_renderer, *m_orchestrator);
        m_timeline.render(*m_orchestrator);

        // Sync viewport selection to properties panel
        m_properties.set_selected(m_orchestrator->get_selected_component());

        m_properties.render(*m_orchestrator);
        render_component_tree();

        if (m_show_demo) {
            ImGui::ShowDemoWindow(&m_show_demo);
        }

        if (m_show_circuit_editor) {
            m_circuit_editor.render(*m_orchestrator);
        }

        if (m_show_code_editor) {
            m_code_editor.render(*m_orchestrator);
        }

        if (m_show_serial_monitor) {
            m_serial_monitor.render(*m_orchestrator);
        }

        // Render
        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(m_window, &display_w, &display_h);

        // Main viewport clear (behind ImGui)
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.05f, 0.05f, 0.07f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }
}

void UIApplication::render_component_tree() {
    ImGui::Begin("Scene");

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
            {"rendering", "Rendering"}
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
    ImGui::End();
}

} // namespace mechatron
