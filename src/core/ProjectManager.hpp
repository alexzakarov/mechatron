#pragma once

#include "SimulationOrchestrator.hpp"
#include "MechatronProject.hpp"
#include <string>
#include <memory>
#include <functional>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace mechatron {

/**
 * ProjectManager - Mechatron Project (.mepro) Management System
 *
 * This class manages the complete project lifecycle including:
 * - Creating new projects
 * - Loading/saving projects
 * - Autosave functionality
 * - Session state persistence
 * - Project templates
 *
 * Similar to Visual Studio's .sln file management, but for mechatronics simulations.
 */
class ProjectManager {
public:
    // Project file extension
    static constexpr const char* PROJECT_EXTENSION = ".mepro";

    // Autosave configuration
    struct AutosaveConfig {
        bool enabled = true;
        double interval_seconds = 30.0;  // Autosave every 30 seconds
        int max_autosaves = 5;           // Keep last 5 autosaves
        std::string autosave_dir = ".mechatron_autosave";
    };

    // Project metadata
    struct ProjectMetadata {
        std::string name;
        std::string description;
        std::string author;
        std::string created;
        std::string modified;
        std::vector<std::string> tags;

        ProjectMetadata() {
            // Set current time as default
            auto now = std::time(nullptr);
            char buf[100];
            std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
            created = buf;
            modified = buf;
        }
    };

    // Session state (UI, camera, selection)
    struct SessionState {
        struct CameraState {
            Vec3 position;
            Vec3 target;
            Vec3 up;
        } camera;

        std::string selected_component;
        nlohmann::json ui_layout;

        // Viewport settings
        struct ViewportSettings {
            bool wireframe = false;
            int shading_mode = 2;  // 0=flat, 1=smooth, 2=material
            bool show_grid = true;
            float grid_size = 10.0f;
            Vec3 background_color{0.08f, 0.08f, 0.09f};
        } viewport;
    };

    // Project statistics
    struct ProjectStats {
        size_t component_count = 0;
        size_t connection_count = 0;
        size_t asset_count = 0;
        std::string last_autosave_time;
        std::string total_edit_time;
    };

    ProjectManager(SimulationOrchestrator& orchestrator);
    ~ProjectManager();

    // Project lifecycle
    bool create_new_project(const std::string& name, const std::string& location);
    bool open_project(const std::string& project_path);
    bool save_project();
    bool save_project_as(const std::string& project_path);
    void close_project();

    // Project state queries
    bool has_project() const { return m_project_open; }
    bool is_modified() const { return m_modified; }
    const std::string& project_path() const { return m_project_path; }
    const std::string& project_name() const { return m_metadata.name; }
    const ProjectMetadata& metadata() const { return m_metadata; }
    ProjectMetadata& metadata() { return m_metadata; }
    const ProjectStats& stats() const { return m_stats; }

    // Autosave management
    void set_autosave_config(const AutosaveConfig& config) { m_autosave_config = config; }
    const AutosaveConfig& autosave_config() const { return m_autosave_config; }
    void enable_autosave(bool enable) { m_autosave_config.enabled = enable; }
    void update_autosave(double current_time_seconds);
    bool has_autosave() const;
    bool restore_from_autosave();
    bool restore_from_autosave(size_t index);
    void clear_autosaves();
    std::vector<std::string> list_autosaves() const;

    // Session state
    const SessionState& session_state() const { return m_session; }
    SessionState& session_state() { return m_session; }
    void save_session_state();
    void restore_session_state();

    // Project templates
    static std::vector<std::string> get_available_templates();
    bool create_from_template(const std::string& template_name,
                             const std::string& project_name,
                             const std::string& location);

    // Export/Import
    bool export_project(const std::string& export_path);
    bool import_project(const std::string& import_path);

    // Project validation
    bool validate_project(std::string& error_message) const;
    std::vector<std::string> get_missing_assets() const;
    std::vector<std::string> get_missing_plugins() const;

    // Callbacks for UI integration
    using ProjectChangedCallback = std::function<void(const std::string& project_path)>;
    using ModificationChangedCallback = std::function<void(bool modified)>;
    using AutosaveCallback = std::function<void(const std::string& autosave_path)>;
    using ApplicationStateSerializeCallback = std::function<nlohmann::json()>;
    using ApplicationStateDeserializeCallback = std::function<void(const nlohmann::json& state)>;

    void set_project_changed_callback(ProjectChangedCallback cb) { m_on_project_changed = cb; }
    void set_modification_changed_callback(ModificationChangedCallback cb) { m_on_modification_changed = cb; }
    void set_autosave_callback(AutosaveCallback cb) { m_on_autosave = cb; }
    void set_application_state_callbacks(ApplicationStateSerializeCallback serialize_cb,
                                         ApplicationStateDeserializeCallback deserialize_cb) {
        m_on_serialize_application_state = std::move(serialize_cb);
        m_on_deserialize_application_state = std::move(deserialize_cb);
    }

private:
    // Serialization helpers
    bool serialize_to_json(nlohmann::json& j, bool split_project_files = false) const;
    bool deserialize_from_json(const nlohmann::json& j);

    // Component serialization
    nlohmann::json serialize_components() const;
    bool deserialize_components(const nlohmann::json& j);

    // Connection serialization
    nlohmann::json serialize_connections() const;
    bool deserialize_connections(const nlohmann::json& j);

    // Asset reference serialization
    nlohmann::json serialize_assets() const;
    bool deserialize_assets(const nlohmann::json& j);

    // Circuit mapping serialization
    nlohmann::json serialize_circuit_mappings() const;
    bool deserialize_circuit_mappings(const nlohmann::json& j);

    // Simulation settings serialization
    nlohmann::json serialize_simulation_settings() const;
    bool deserialize_simulation_settings(const nlohmann::json& j);

    // Autosave helpers
    void perform_autosave();
    std::filesystem::path get_autosave_directory() const;
    std::filesystem::path get_autosave_file_path(int index = 0) const;
    std::vector<std::filesystem::path> get_existing_autosaves() const;
    void cleanup_old_autosaves();
    bool load_autosave_metadata(const std::filesystem::path& autosave_path, nlohmann::json& metadata);

    // Hash calculation for change detection
    uint64_t calculate_project_hash() const;

    // Path utilities
    std::string get_project_directory() const;
    static std::string sanitize_filename(const std::string& name);

    // Mark project as modified
    void mark_modified(bool modified = true);

    // Update statistics
    void update_statistics();

private:
    SimulationOrchestrator& m_orchestrator;

    // Project state
    bool m_project_open = false;
    bool m_modified = false;
    std::string m_project_path;
    ProjectMetadata m_metadata;
    SessionState m_session;
    ProjectStats m_stats;

    // Autosave
    AutosaveConfig m_autosave_config;
    double m_last_autosave_time = 0.0;
    uint64_t m_last_autosave_hash = 0;

    // Callbacks
    ProjectChangedCallback m_on_project_changed;
    ModificationChangedCallback m_on_modification_changed;
    AutosaveCallback m_on_autosave;
    ApplicationStateSerializeCallback m_on_serialize_application_state;
    ApplicationStateDeserializeCallback m_on_deserialize_application_state;

    // Project format version
    static constexpr const char* PROJECT_FORMAT_VERSION = "1.0.0";
    static constexpr const char* PROJECT_FORMAT = "mechatron_project";
};

} // namespace mechatron
