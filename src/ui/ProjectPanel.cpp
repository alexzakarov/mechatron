#include "ProjectPanel.hpp"
#include <imgui.h>
#include "Theme.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <sstream>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#endif

namespace mechatron {

namespace fs = std::filesystem;

namespace {

std::string trim_trailing_newlines(std::string text) {
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
        text.pop_back();
    }
    return text;
}

#ifndef _WIN32
std::string run_dialog_command(const char* command) {
    std::array<char, 512> buffer{};
    std::string result;
    FILE* pipe = popen(command, "r");
    if (!pipe) return {};
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
        result += buffer.data();
    }
    pclose(pipe);
    return trim_trailing_newlines(result);
}
#endif

std::string choose_project_location_folder() {
#ifdef _WIN32
    BROWSEINFOA bi{};
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    bi.lpszTitle = "Select project location";
    PIDLIST_ABSOLUTE pidl = SHBrowseForFolderA(&bi);
    if (!pidl) return {};

    char path[MAX_PATH] = {};
    const bool ok = SHGetPathFromIDListA(pidl, path);
    CoTaskMemFree(pidl);
    return ok ? std::string(path) : std::string();
#elif defined(__APPLE__)
    return run_dialog_command("osascript -e 'POSIX path of (choose folder with prompt \"Select project location\")' 2>/dev/null");
#else
    std::string folder = run_dialog_command("zenity --file-selection --directory --title='Select project location' 2>/dev/null");
    if (!folder.empty()) return folder;
    return run_dialog_command("kdialog --getexistingdirectory . 'Select project location' 2>/dev/null");
#endif
}

std::string choose_project_file() {
#ifdef _WIN32
    char path[MAX_PATH] = {};
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFile = path;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = "Open Mechatron project";
    ofn.lpstrFilter = "Mechatron Project\0*.mepro\0All Files\0*.*\0";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    return GetOpenFileNameA(&ofn) ? std::string(path) : std::string();
#elif defined(__APPLE__)
    return run_dialog_command("osascript -e 'POSIX path of (choose file with prompt \"Open Mechatron project\" of type {\"mepro\"})' 2>/dev/null");
#else
    std::string file = run_dialog_command("zenity --file-selection --title='Open Mechatron project' --file-filter='Mechatron project | *.mepro' 2>/dev/null");
    if (!file.empty()) return file;
    return run_dialog_command("kdialog --getopenfilename . 'Mechatron project (*.mepro)' 2>/dev/null");
#endif
}

void prepare_centered_modal(const ImVec2& size) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    if (!viewport) {
        return;
    }

    ImGui::SetNextWindowPos(viewport->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size, ImGuiCond_Appearing);
}

} // namespace

ProjectPanel::ProjectPanel(ProjectManager& manager)
    : m_manager(manager)
{
    // Initialize templates
    m_new_project_data.templates = ProjectManager::get_available_templates();

    // Initialize default location
    fs::path default_path = fs::current_path();
    strncpy(m_new_project_data.location, default_path.string().c_str(), sizeof(m_new_project_data.location) - 1);

    // Setup callbacks
    m_manager.set_project_changed_callback([this](const std::string& path) {
        // Update UI when project changes
    });

    m_manager.set_modification_changed_callback([this](bool modified) {
        // Update title bar with modified indicator
    });

    m_manager.set_autosave_callback([this](const std::string& autosave_path) {
        // Show notification about autosave
    });
}

void ProjectPanel::render() {
    if (!ImGui::Begin(PANEL_TITLE, nullptr, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    // Project status section
    if (m_manager.has_project()) {
        // Project name and status
        std::string display_name = get_project_display_name();
        ImGui::Text("Project: %s", display_name.c_str());

        // Modified indicator
        std::string modified = get_modified_indicator();
        if (!modified.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(Theme::CurrentPalette().warning, "%s", modified.c_str());
        }

        ImGui::Separator();

        // Project actions
        if (ImGui::Button("Save")) {
            m_manager.save_project();
        }
        ImGui::SameLine();
        if (ImGui::Button("Save As...")) {
            show_save_as_dialog();
        }
        ImGui::SameLine();
        if (ImGui::Button("Close")) {
            request_close_project();
        }

        ImGui::Separator();

        // Project info
        if (ImGui::CollapsingHeader("Project Info", ImGuiTreeNodeFlags_DefaultOpen)) {
            render_project_info();
        }

        // Statistics
        if (ImGui::CollapsingHeader("Statistics")) {
            render_project_stats();
        }

        // Autosave settings
        if (ImGui::CollapsingHeader("Autosave")) {
            render_autosave_settings();
        }

        // Properties button
        ImGui::Separator();
        if (ImGui::Button("Properties...")) {
            show_project_properties();
        }

    } else {
        // No project open
        ImGui::Text("No project open");
        ImGui::Separator();

        if (ImGui::Button("New Project...")) {
            show_new_project_dialog();
        }
        ImGui::SameLine();
        if (ImGui::Button("Open Project...")) {
            show_open_project_dialog();
        }

        // Recent projects
        if (ImGui::CollapsingHeader("Recent Projects")) {
            render_recent_projects();
        }
    }

    ImGui::End();
    render_dialogs();
}

void ProjectPanel::render_menu_items() {
    if (ImGui::BeginMenu("Project")) {
        render_project_menu();
        ImGui::EndMenu();
    }
}

void ProjectPanel::render_file_menu() {
    render_project_menu();
}

void ProjectPanel::render_project_menu() {
    // New Project
    if (ImGui::MenuItem("New Project...", "Ctrl+Shift+N")) {
        show_new_project_dialog();
    }

    // Open Project
    if (ImGui::MenuItem("Open Project...", "Ctrl+O")) {
        show_open_project_dialog();
    }

    ImGui::Separator();

    // Save/Save As (only if project open)
    if (m_manager.has_project()) {
        if (ImGui::MenuItem("Save", "Ctrl+S")) {
            m_manager.save_project();
        }

        if (ImGui::MenuItem("Save As...", "Ctrl+Shift+S")) {
            show_save_as_dialog();
        }

        ImGui::Separator();

        // Close Project
        if (ImGui::MenuItem("Close Project")) {
            request_close_project();
        }
    }

    ImGui::Separator();

    // Recent Projects
    if (ImGui::BeginMenu("Recent Projects")) {
        render_recent_projects();
        ImGui::EndMenu();
    }
}

void ProjectPanel::render_dialogs() {
    if (m_popup_open_requested) {
        switch (m_current_dialog) {
            case DialogType::NewProject: ImGui::OpenPopup("New Project"); break;
            case DialogType::OpenProject: ImGui::OpenPopup("Open Project"); break;
            case DialogType::SaveAs: ImGui::OpenPopup("Save Project As"); break;
            case DialogType::ProjectProperties: ImGui::OpenPopup("Project Properties"); break;
            case DialogType::AutosaveRecovery: ImGui::OpenPopup("Autosave Recovery"); break;
            case DialogType::None: break;
        }
        m_popup_open_requested = false;
    }

    if (m_dialog_open) {
        switch (m_current_dialog) {
            case DialogType::NewProject: render_new_project_dialog(); break;
            case DialogType::OpenProject: render_open_project_dialog(); break;
            case DialogType::SaveAs: render_save_as_dialog(); break;
            case DialogType::ProjectProperties: render_project_properties_dialog(); break;
            case DialogType::AutosaveRecovery: render_autosave_recovery_dialog(); break;
            case DialogType::None: break;
        }
    }
    render_confirm_close_dialog();
}

void ProjectPanel::show_new_project_dialog() {
    m_current_dialog = DialogType::NewProject;
    m_dialog_open = true;
    m_popup_open_requested = true;
    m_status_message.clear();

    // Reset dialog data
    memset(m_new_project_data.name, 0, sizeof(m_new_project_data.name));
    m_new_project_data.selected_template = 0;
}

void ProjectPanel::show_open_project_dialog() {
    m_current_dialog = DialogType::OpenProject;
    m_dialog_open = true;
    m_popup_open_requested = true;
    m_status_message.clear();

    // Reset dialog data
    m_open_project_data.selected_recent = -1;
    memset(m_open_project_data.custom_path, 0, sizeof(m_open_project_data.custom_path));
}

void ProjectPanel::show_save_as_dialog() {
    m_current_dialog = DialogType::SaveAs;
    m_dialog_open = true;
    m_popup_open_requested = true;
    m_status_message.clear();
    const std::string current = m_manager.project_path();
    std::snprintf(m_save_as_data.path, sizeof(m_save_as_data.path), "%s", current.c_str());
}

void ProjectPanel::show_project_properties() {
    m_current_dialog = DialogType::ProjectProperties;
    m_dialog_open = true;
    m_popup_open_requested = true;
    m_status_message.clear();

    // Load current metadata
    const auto& meta = m_manager.metadata();
    strncpy(m_properties_data.name, meta.name.c_str(), sizeof(m_properties_data.name) - 1);
    strncpy(m_properties_data.description, meta.description.c_str(), sizeof(m_properties_data.description) - 1);
    strncpy(m_properties_data.author, meta.author.c_str(), sizeof(m_properties_data.author) - 1);

    // Convert tags to comma-separated string
    std::string tags_str;
    for (size_t i = 0; i < meta.tags.size(); ++i) {
        if (i > 0) tags_str += ", ";
        tags_str += meta.tags[i];
    }
    strncpy(m_properties_data.tags, tags_str.c_str(), sizeof(m_properties_data.tags) - 1);
}

void ProjectPanel::show_autosave_recovery_dialog() {
    m_current_dialog = DialogType::AutosaveRecovery;
    m_dialog_open = true;
    m_popup_open_requested = true;
    m_status_message.clear();
}

void ProjectPanel::render_status_bar() {
    if (m_manager.has_project()) {
        // Project name
        std::string display_name = get_project_display_name();
        ImGui::Text("%s", display_name.c_str());

        // Modified indicator
        std::string modified = get_modified_indicator();
        if (!modified.empty()) {
            ImGui::SameLine();
            ImGui::Text("%s", modified.c_str());
        }

        // Autosave status
        if (m_manager.autosave_config().enabled) {
            ImGui::SameLine();
            ImGui::Text("| Autosave: On");
        }
    } else {
        ImGui::Text("No project");
    }
}

void ProjectPanel::render_project_info() {
    const auto& meta = m_manager.metadata();

    ImGui::Text("Name: %s", meta.name.c_str());
    ImGui::Text("Created: %s", meta.created.c_str());
    ImGui::Text("Modified: %s", meta.modified.c_str());

    if (!meta.author.empty()) {
        ImGui::Text("Author: %s", meta.author.c_str());
    }

    if (!meta.description.empty()) {
        ImGui::TextWrapped("Description: %s", meta.description.c_str());
    }

    if (!meta.tags.empty()) {
        ImGui::Text("Tags:");
        ImGui::SameLine();
        for (size_t i = 0; i < meta.tags.size(); ++i) {
            if (i > 0) ImGui::SameLine();
            ImGui::TextColored(Theme::CurrentPalette().primaryHover, "[%s]", meta.tags[i].c_str());
        }
    }
}

void ProjectPanel::render_project_stats() {
    const auto& stats = m_manager.stats();

    ImGui::Text("Components: %zu", stats.component_count);
    ImGui::Text("Connections: %zu", stats.connection_count);
    ImGui::Text("Assets: %zu", stats.asset_count);

    if (!stats.last_autosave_time.empty()) {
        ImGui::Text("Last Autosave: %s", stats.last_autosave_time.c_str());
    }
}

void ProjectPanel::render_autosave_settings() {
    auto& config = const_cast<ProjectManager::AutosaveConfig&>(m_manager.autosave_config());

    // Enabled
    bool enabled = config.enabled;
    if (ImGui::Checkbox("Enable Autosave", &enabled)) {
        m_manager.enable_autosave(enabled);
    }

    if (enabled) {
        // Interval
        constexpr double min_interval = 5.0;
        constexpr double max_interval = 3600.0;
        ImGui::SliderScalar("Interval (s)", ImGuiDataType_Double, &config.interval_seconds, &min_interval, &max_interval, "%.1f");

        // Max autosaves
        ImGui::SliderInt("Max Autosaves", &config.max_autosaves, 1, 20);

        // Autosave status
        if (m_manager.has_autosave()) {
            ImGui::TextColored(Theme::CurrentPalette().warning, "Autosave available");
            if (ImGui::Button("Restore Autosave")) {
                if (m_manager.restore_from_autosave()) {
                    m_current_dialog = DialogType::None;
                }
            }
        } else {
            ImGui::Text("No autosave available");
        }
    }
}

void ProjectPanel::render_recent_projects() {
    if (m_open_project_data.recent_projects.empty()) {
        ImGui::TextDisabled("No recent projects");
        return;
    }
    for (int i = 0; i < static_cast<int>(m_open_project_data.recent_projects.size()); ++i) {
        const std::string& path = m_open_project_data.recent_projects[i];
        if (ImGui::MenuItem(path.c_str())) {
            if (m_manager.open_project(path)) {
                set_status("Project opened.");
            } else {
                set_status("Failed to open recent project.", true);
            }
        }
    }
}

void ProjectPanel::create_new_project() {
    std::string name(m_new_project_data.name);
    std::string location(m_new_project_data.location);

    if (name.empty() || location.empty()) {
        set_status("Project name and location are required.", true);
        return;
    }

    const std::string templ = (m_new_project_data.selected_template >= 0 &&
                              m_new_project_data.selected_template < static_cast<int>(m_new_project_data.templates.size()))
        ? m_new_project_data.templates[m_new_project_data.selected_template]
        : std::string{};
    if (templ.empty()) {
        set_status("No valid project template is available.", true);
        return;
    }
    if (m_manager.create_from_template(templ, name, location)) {
        m_current_dialog = DialogType::None;
        m_dialog_open = false;
        set_status("Project created.");
    } else {
        set_status("Failed to create project.", true);
    }
}

void ProjectPanel::open_selected_project() {
    if (m_open_project_data.selected_recent >= 0) {
        // Open from recent projects
        // TODO: Implement recent projects
    } else if (strlen(m_open_project_data.custom_path) > 0) {
        // Open from custom path
        std::string path(m_open_project_data.custom_path);
        if (m_manager.open_project(path)) {
            m_current_dialog = DialogType::None;
            m_dialog_open = false;
            set_status("Project opened.");
        } else {
            set_status("Failed to open project.", true);
        }
    }
}

void ProjectPanel::save_project_as() {
    std::string path(m_save_as_data.path);
    if (path.empty()) {
        set_status("Save path is required.", true);
        return;
    }
    if (fs::path(path).extension() != ProjectManager::PROJECT_EXTENSION) {
        path += ProjectManager::PROJECT_EXTENSION;
    }
    if (m_manager.save_project_as(path)) {
        m_current_dialog = DialogType::None;
        m_dialog_open = false;
        set_status("Project saved.");
    } else {
        set_status("Failed to save project.", true);
    }
}

std::string ProjectPanel::get_project_display_name() const {
    if (!m_manager.has_project()) {
        return "No Project";
    }

    const auto& meta = m_manager.metadata();
    return meta.name.empty() ? "Untitled" : meta.name;
}

std::string ProjectPanel::get_modified_indicator() const {
    return m_manager.is_modified() ? "*" : "";
}

void ProjectPanel::request_close_project() {
    if (!m_manager.has_project()) return;
    if (!m_manager.is_modified()) {
        m_manager.close_project();
        set_status("Project closed.");
        return;
    }
    m_confirm_close_open = true;
}

void ProjectPanel::render_confirm_close_dialog() {
    if (m_confirm_close_open) {
        ImGui::OpenPopup("Save Changes?");
        m_confirm_close_open = false;
    }
    if (ImGui::BeginPopupModal("Save Changes?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("The current project has unsaved changes.");
        ImGui::Text("Do you want to save them before closing?");
        ImGui::Separator();
        if (ImGui::Button("Save", ImVec2(120, 0))) {
            if (m_manager.save_project()) {
                m_manager.close_project();
                set_status("Project saved and closed.");
                ImGui::CloseCurrentPopup();
            } else {
                set_status("Failed to save project.", true);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Don't Save", ImVec2(120, 0))) {
            m_manager.close_project();
            set_status("Project closed.");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void ProjectPanel::render_new_project_dialog() {
    prepare_centered_modal(ImVec2(560.0f, 0.0f));
    bool open = true;
    if (ImGui::BeginPopupModal("New Project", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::InputText("Name", m_new_project_data.name, sizeof(m_new_project_data.name));
        ImGui::InputText("Location", m_new_project_data.location, sizeof(m_new_project_data.location));
        ImGui::SameLine();
        if (ImGui::Button("Browse...")) {
            const std::string folder = choose_project_location_folder();
            if (!folder.empty()) {
                std::snprintf(m_new_project_data.location, sizeof(m_new_project_data.location), "%s", folder.c_str());
                set_status("Project location selected.");
            }
        }
        if (!m_new_project_data.templates.empty()) {
            std::vector<const char*> items;
            items.reserve(m_new_project_data.templates.size());
            for (const auto& t : m_new_project_data.templates) items.push_back(t.c_str());
            ImGui::Combo("Template", &m_new_project_data.selected_template, items.data(), static_cast<int>(items.size()));
        }
        if (!m_status_message.empty()) {
            ImGui::TextColored(m_status_is_error ? Theme::CurrentPalette().error : Theme::CurrentPalette().success,
                               "%s", m_status_message.c_str());
        }
        ImGui::Separator();
        if (ImGui::Button("Create", ImVec2(120, 0))) {
            create_new_project();
            if (!m_dialog_open) ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) {
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    }
}

void ProjectPanel::render_open_project_dialog() {
    prepare_centered_modal(ImVec2(560.0f, 0.0f));
    bool open = true;
    if (ImGui::BeginPopupModal("Open Project", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::InputText("Path", m_open_project_data.custom_path, sizeof(m_open_project_data.custom_path));
        ImGui::SameLine();
        if (ImGui::Button("Browse...")) {
            const std::string file = choose_project_file();
            if (!file.empty()) {
                std::snprintf(m_open_project_data.custom_path, sizeof(m_open_project_data.custom_path), "%s", file.c_str());
                set_status("Project file selected.");
            }
        }
        ImGui::TextDisabled("Enter a .mepro path.");
        if (!m_status_message.empty()) {
            ImGui::TextColored(m_status_is_error ? Theme::CurrentPalette().error : Theme::CurrentPalette().success,
                               "%s", m_status_message.c_str());
        }
        ImGui::Separator();
        if (ImGui::Button("Open", ImVec2(120, 0))) {
            open_selected_project();
            if (!m_dialog_open) ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) {
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    }
}

void ProjectPanel::render_save_as_dialog() {
    prepare_centered_modal(ImVec2(560.0f, 0.0f));
    bool open = true;
    if (ImGui::BeginPopupModal("Save Project As", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::InputText("Path", m_save_as_data.path, sizeof(m_save_as_data.path));
        ImGui::TextDisabled("The .mepro extension is added automatically when omitted.");
        if (!m_status_message.empty()) {
            ImGui::TextColored(m_status_is_error ? Theme::CurrentPalette().error : Theme::CurrentPalette().success,
                               "%s", m_status_message.c_str());
        }
        ImGui::Separator();
        if (ImGui::Button("Save", ImVec2(120, 0))) {
            save_project_as();
            if (!m_dialog_open) ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) {
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    }
}

void ProjectPanel::render_project_properties_dialog() {
    prepare_centered_modal(ImVec2(620.0f, 0.0f));
    bool open = true;
    if (ImGui::BeginPopupModal("Project Properties", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::InputText("Name", m_properties_data.name, sizeof(m_properties_data.name));
        ImGui::InputText("Author", m_properties_data.author, sizeof(m_properties_data.author));
        ImGui::InputTextMultiline("Description", m_properties_data.description, sizeof(m_properties_data.description), ImVec2(420, 90));
        ImGui::InputText("Tags", m_properties_data.tags, sizeof(m_properties_data.tags));
        if (!m_status_message.empty()) {
            ImGui::TextColored(m_status_is_error ? Theme::CurrentPalette().error : Theme::CurrentPalette().success,
                               "%s", m_status_message.c_str());
        }
        ImGui::Separator();
        if (ImGui::Button("Apply", ImVec2(120, 0))) {
            apply_project_properties();
            if (!m_dialog_open) ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) {
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    }
}

void ProjectPanel::render_autosave_recovery_dialog() {
    prepare_centered_modal(ImVec2(520.0f, 0.0f));
    bool open = true;
    if (ImGui::BeginPopupModal("Autosave Recovery", &open, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextWrapped("An autosave is available for the current project.");
        if (ImGui::Button("Restore", ImVec2(120, 0))) {
            if (m_manager.restore_from_autosave()) {
                set_status("Autosave restored.");
                m_dialog_open = false;
                m_current_dialog = DialogType::None;
                ImGui::CloseCurrentPopup();
            } else {
                set_status("Failed to restore autosave.", true);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(120, 0))) {
            m_manager.clear_autosaves();
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Later", ImVec2(120, 0))) {
            m_dialog_open = false;
            m_current_dialog = DialogType::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (!open) {
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    }
}

void ProjectPanel::set_status(const std::string& text, bool error) {
    m_status_message = text;
    m_status_is_error = error;
}

void ProjectPanel::apply_project_properties() {
    if (!m_manager.has_project()) {
        set_status("No project open.", true);
        return;
    }
    auto& meta = m_manager.metadata();
    meta.name = m_properties_data.name;
    meta.author = m_properties_data.author;
    meta.description = m_properties_data.description;
    meta.tags = split_tags(m_properties_data.tags);
    if (m_manager.save_project()) {
        set_status("Project properties saved.");
        m_dialog_open = false;
        m_current_dialog = DialogType::None;
    } else {
        set_status("Failed to save project properties.", true);
    }
}

std::vector<std::string> ProjectPanel::split_tags(const std::string& tags_csv) {
    std::vector<std::string> tags;
    std::stringstream ss(tags_csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        auto first = item.find_first_not_of(" \t\r\n");
        auto last = item.find_last_not_of(" \t\r\n");
        if (first == std::string::npos || last == std::string::npos) continue;
        tags.push_back(item.substr(first, last - first + 1));
    }
    return tags;
}

} // namespace mechatron
