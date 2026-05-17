#pragma once

#include <string>
#include <vector>
#include <imgui.h>

namespace mechatron {

class SimulationOrchestrator;

class CodeEditor {
public:
    CodeEditor();
    void render(SimulationOrchestrator& orchestrator);

    void load_file(const std::string& path);
    void save_file(const std::string& path);
    const std::string& current_file() const { return m_current_file; }

    // Arduino CLI configuration
    void set_arduino_cli_path(const std::string& path) { m_arduino_cli_path = path; }
    void set_fqbn(const std::string& fqbn) { m_fqbn = fqbn; }
    const std::string& arduino_cli_path() const { return m_arduino_cli_path; }
    const std::string& fqbn() const { return m_fqbn; }

    // Serial baud rate for template
    void set_baud_rate(int baud) { m_baud_rate = baud; }
    int baud_rate() const { return m_baud_rate; }

    // Recent files history
    static constexpr size_t MAX_RECENT_FILES = 10;
    void add_recent_file(const std::string& path);
    const std::vector<std::string>& recent_files() const { return m_recent_files; }

    // Configuration persistence
    void save_config();
    void load_config();
    static const char* config_file_path() { return "mechatron_code_editor_config.json"; }

    // Auto-save functionality
    void set_auto_save_enabled(bool enabled);
    bool is_auto_save_enabled() const;
    void set_auto_save_interval(double seconds);
    double auto_save_interval() const;

    // Check and perform auto-save if needed
    void check_auto_save();

private:
    struct TextEditor {
        std::string content;
        int cursor_line = 1;
        int cursor_column = 1;
        int scroll_x = 0;
        int scroll_y = 0;

        bool readonly = false;
        bool show_line_numbers = true;
        bool show_whitespace = false;
    };

    TextEditor m_editor;
    std::string m_current_file;
    bool m_file_modified = false;

    // Arduino CLI configuration
    std::string m_arduino_cli_path = "arduino-cli";  // Default: assume it's in PATH
    std::string m_arduino_cli_config_path;
    std::string m_fqbn = "arduino:avr:uno";  // Default: Arduino Uno
    int m_baud_rate = 9600;  // Default serial baud rate

    // Recent files history
    std::vector<std::string> m_recent_files;

    // Auto-save state
    bool m_auto_save_enabled = false;
    double m_auto_save_interval = 60.0;  // seconds (default: 1 minute)
    double m_last_auto_save_time = 0.0;
    double m_last_save_time = 0.0;

    // UI State
    bool m_show_search = false;
    bool m_show_replace = false;
    bool m_show_settings = false;
    bool m_show_open_dialog = false;
    bool m_show_save_as_dialog = false;
    std::string m_file_dialog_path;
    char m_search_buffer[256] = "";
    char m_replace_buffer[256] = "";
    int m_current_match = 0;
    int m_total_matches = 0;

    // Syntax highlighting (simplified)
    struct HighlightRange {
        int start, end;
        ImU32 color;
    };
    std::vector<HighlightRange> m_highlight_ranges;

    // Compile/upload state
    bool m_compiling = false;
    bool m_uploading = false;
    std::string m_compile_output;
    std::string m_compile_errors;
    std::string m_compiled_hex_path;  // Path to compiled .hex file
    std::string m_selected_mcu_id;

    void render_menu_bar(SimulationOrchestrator& orchestrator);
    void render_mcu_selector(SimulationOrchestrator& orchestrator);
    void render_editor_area();
    void render_status_bar();
    void render_search_replace();
    void render_file_dialogs();

    void update_syntax_highlighting();
    ImU32 get_color_for_token(const std::string& token);
    void update_cursor_from_index(size_t cursor_index);

    void insert_text(const std::string& text);
    void delete_selection();
    void move_cursor(int delta_line, int delta_column);
    int get_line_count() const;
    std::string get_line(int line_index) const;

    bool compile_sketch();
    bool upload_to_mcu(SimulationOrchestrator& orchestrator);
    bool check_arduino_cli();
    std::string run_arduino_command(const std::vector<std::string>& args);
    std::string prepare_sketch_for_compile();
    std::string find_hex_file(const std::string& build_path, const std::string& sketch_name) const;
    void update_search_matches();

    // Arduino keywords for syntax highlighting
    static const char* s_keywords[];
    static const char* s_types[];
    static const char* s_constants[];
};

} // namespace mechatron
