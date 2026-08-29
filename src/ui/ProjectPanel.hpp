#pragma once

#include "core/ProjectManager.hpp"
#include <string>
#include <vector>

namespace mechatron {

/**
 * ProjectPanel - ImGui UI panel for project management
 *
 * Provides user interface for:
 * - Creating new projects
 * - Opening/saving projects
 * - Project properties
 * - Autosave management
 * - Project statistics
 */
class ProjectPanel {
public:
    ProjectPanel(ProjectManager& manager);
    ~ProjectPanel() = default;

    // Main render function
    void render();

    // Menu integration
    void render_menu_items();
    void render_file_menu();
    void render_project_menu();

    // Dialog management
    void render_dialogs();
    void show_new_project_dialog();
    void show_open_project_dialog();
    void show_save_as_dialog();
    void show_project_properties();
    void show_autosave_recovery_dialog();

    // Status bar content
    void render_status_bar();

    // Dialog state
    bool is_dialog_open() const { return m_dialog_open; }

private:
    void render_project_info();
    void render_project_stats();
    void render_autosave_settings();
    void render_recent_projects();

    void create_new_project();
    void open_selected_project();
    void save_project_as();
    void request_close_project();
    void render_confirm_close_dialog();
    void render_new_project_dialog();
    void render_open_project_dialog();
    void render_save_as_dialog();
    void render_project_properties_dialog();
    void render_autosave_recovery_dialog();

    // Helper functions
    std::string get_project_display_name() const;
    std::string get_modified_indicator() const;
    void set_status(const std::string& text, bool error = false);
    void apply_project_properties();
    static std::vector<std::string> split_tags(const std::string& tags_csv);

private:
    ProjectManager& m_manager;

    // Dialog state
    enum class DialogType {
        None,
        NewProject,
        OpenProject,
        SaveAs,
        ProjectProperties,
        AutosaveRecovery
    };
    DialogType m_current_dialog = DialogType::None;
    bool m_dialog_open = false;
    bool m_popup_open_requested = false;
    bool m_confirm_close_open = false;
    bool m_status_is_error = false;
    std::string m_status_message;

    // New project dialog data
    struct NewProjectData {
        char name[256] = "";
        char location[512] = "";
        int selected_template = 0;
        std::vector<std::string> templates;
    } m_new_project_data;

    // Open project dialog data
    struct OpenProjectData {
        std::vector<std::string> recent_projects;
        int selected_recent = -1;
        char custom_path[512] = "";
    } m_open_project_data;

    struct SaveAsData {
        char path[512] = "";
    } m_save_as_data;

    // Project properties dialog data
    struct ProjectPropertiesData {
        char name[256] = "";
        char description[1024] = "";
        char author[256] = "";
        char tags[512] = "";
    } m_properties_data;

    // Autosave settings
    bool m_autosave_enabled = true;
    double m_autosave_interval = 30.0;
    int m_max_autosaves = 5;

    // UI state
    bool m_show_stats = true;
    bool m_show_autosave_settings = false;
    bool m_show_recent_projects = true;

    // Constants
    static constexpr const char* PANEL_TITLE = "Project";
};

} // namespace mechatron
