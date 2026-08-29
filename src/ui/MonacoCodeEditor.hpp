#pragma once

#include "WebView.hpp"
#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <nlohmann/json.hpp>

namespace mechatron {

class SimulationOrchestrator;
class ArduinoLspClient;

/**
 * Monaco Editor integration for Mechatron
 * Uses WebView to embed Monaco Editor and communicates via JavaScript bridge
 */
class MonacoCodeEditor {
public:
    MonacoCodeEditor();
    ~MonacoCodeEditor();

    // Initialize the webview and load Monaco Editor
    bool initialize();

    // Attach to native window (must be called after initialize, before first render)
    void attach_to_window(void* window);
    void set_visible(bool visible);

    // Render the editor (call every frame)
    void render(SimulationOrchestrator& orchestrator);

    // File operations
    void load_file(const std::string& path);
    void save_file(const std::string& path);
    const std::string& current_file() const { return m_current_file; }

    // Content operations
    void set_content(const std::string& content);
    std::string get_content() const; // Returns cached content

    // Editor configuration
    void set_language(const std::string& language);  // cpp, javascript, arduino, etc.
    void set_theme(const std::string& theme);  // vs-dark, vs-light, hc-black
    void set_font_size(int size);

    // Arduino integration (from old CodeEditor)
    void set_arduino_cli_path(const std::string& path) { m_arduino_cli_path = path; }
    void set_fqbn(const std::string& fqbn) { m_fqbn = fqbn; }
    const std::string& arduino_cli_path() const { return m_arduino_cli_path; }
    const std::string& fqbn() const { return m_fqbn; }
    void set_baud_rate(int baud) { m_baud_rate = baud; }
    int baud_rate() const { return m_baud_rate; }

    // MCU selection for upload
    void set_selected_mcu(const std::string& mcu_id) { m_selected_mcu_id = mcu_id; }
    const std::string& selected_mcu() const { return m_selected_mcu_id; }

    // Compilation and upload
    bool compile_sketch();
    bool upload_to_mcu(SimulationOrchestrator& orchestrator);

    // Recent files
    static constexpr size_t MAX_RECENT_FILES = 10;
    void add_recent_file(const std::string& path);
    const std::vector<std::string>& recent_files() const { return m_recent_files; }

    // Auto-save
    void set_auto_save_enabled(bool enabled);
    bool is_auto_save_enabled() const;
    void set_auto_save_interval(double seconds);
    double auto_save_interval() const;
    void check_auto_save();

    // When a project is open, set the sketch path so auto-save can write
    // even when the user hasn't explicitly done File > Open.
    void set_project_sketch_path(const std::string& path) { m_project_sketch_path = path; }
    const std::string& project_sketch_path() const { return m_project_sketch_path; }

    // Project state serialization
    nlohmann::json serialize_project_state() const;
    void deserialize_project_state(const nlohmann::json& state);

    // Configuration persistence
    void save_config();
    void load_config();
    static const char* config_file_path() { return "mechatron_monaco_editor_config.json"; }

    // Callback fired after every save (manual or auto-save) so the
    // application can sync project sidecar files when appropriate.
    using SaveCallback = std::function<void(const std::string& saved_path)>;
    void set_on_save_callback(SaveCallback cb) { m_on_save_callback = std::move(cb); }

    // Check if editor is ready
    bool is_ready() const { return m_webview && m_webview->is_ready() && m_editor_ready; }

private:
    // WebView for Monaco Editor
    std::unique_ptr<WebView> m_webview;
    void* m_host_window = nullptr;
    bool m_webview_attached = false;
    bool m_visible = false;

    // Editor state
    std::string m_current_file;
    std::string m_project_sketch_path;  // e.g. /path/to/project/code/sketch.ino
    bool m_file_modified = false;
    bool m_editor_ready = false;  // Monaco Editor loaded and ready
    std::string m_load_error;

    // Content cache (synced with Monaco)
    std::string m_content_cache;

    // Arduino language server integration
    std::unique_ptr<ArduinoLspClient> m_lsp_client;
    std::mutex m_webview_message_mutex;
    std::vector<std::string> m_webview_message_queue;
    double m_last_lsp_ready_false_log_time = -1.0;
    double m_last_lsp_reconnect_attempt_time = -1.0;
    static constexpr double LSP_RECONNECT_INTERVAL = 5.0;  // Retry every 5 seconds

    // Arduino configuration
    std::string m_arduino_cli_path = "arduino-cli";
    std::string m_fqbn = "arduino:avr:uno";
    int m_baud_rate = 9600;
    std::string m_selected_mcu_id;

    // Recent files
    std::vector<std::string> m_recent_files;

    // Auto-save
    bool m_auto_save_enabled = true;
    double m_auto_save_interval = 60.0;
    double m_last_auto_save_time = 0.0;
    double m_last_save_time = 0.0;
    double m_last_content_change_time = 0.0;
    static constexpr double INSTANT_SAVE_DELAY = 0.3;  // seconds after last keystroke
    bool m_pending_instant_save = false;

    // Polling fallback: if contentChanged events never arrive from WebView,
    // periodically request content from Monaco and compare with cached version.
    double m_last_content_poll_time = 0.0;
    static constexpr double CONTENT_POLL_INTERVAL = 0.5;
    std::string m_last_saved_content;  // last content that was written to disk
    std::string m_modified_file;       // save target captured when the edit happened
    int m_editor_document_generation = 0;

    // UI state
    bool m_show_settings = false;
    bool m_show_open_dialog = false;
    bool m_show_save_as_dialog = false;
    bool m_initialized = false;
    std::string m_file_dialog_path;
    float m_output_panel_height = 170.0f;

    // Editor configuration
    std::string m_language = "arduino";
    std::string m_theme = "vs-dark";
    int m_font_size = 14;

    // Compile/upload state
    bool m_compiling = false;
    bool m_uploading = false;
    std::string m_compile_output;
    std::string m_compile_errors;
    std::string m_compiled_hex_path;

    // Callbacks
    SaveCallback m_on_save_callback;

    // Message handling from JavaScript
    void handle_webview_message(const std::string& message);

    // JavaScript commands
    void send_content_to_editor();
    void request_content_from_editor();
    void update_editor_config();
    void queue_webview_message(const nlohmann::json& message);
    void flush_webview_messages();
    void start_arduino_lsp();
    void handle_lsp_completion_request(const nlohmann::json& request);
    void log_lsp_not_ready_if_due();
    void attempt_lsp_reconnection_if_needed();
    std::string resolve_current_save_target() const;
    bool write_content_to_file(const std::string& path, const std::string& content, bool make_current);
    bool auto_save_current_content(const char* reason);
    void mark_content_as_clean(const std::string& content);
    void flush_pending_autosave_before_switch(const std::string& next_path);

    // Helper functions (from old CodeEditor)
    bool check_arduino_cli();
    std::string run_arduino_command(const std::vector<std::string>& args);
    std::string prepare_sketch_for_compile();
    std::string find_hex_file(const std::string& build_path, const std::string& sketch_name) const;

    // Render helpers
    void render_menu_bar(SimulationOrchestrator& orchestrator);
    void render_mcu_selector(SimulationOrchestrator& orchestrator);
    void render_status_bar();
    void render_compilation_output();
    void render_bottom_tabs(float height);
    void render_settings();
    void render_editor_host(float reserved_bottom_height);
    void render_file_dialogs();
};

} // namespace mechatron
