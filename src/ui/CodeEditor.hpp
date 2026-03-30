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

    // UI State
    bool m_show_search = false;
    bool m_show_replace = false;
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

    void render_menu_bar();
    void render_editor_area();
    void render_status_bar();
    void render_search_replace();

    void update_syntax_highlighting();
    ImU32 get_color_for_token(const std::string& token);

    void insert_text(const std::string& text);
    void delete_selection();
    void move_cursor(int delta_line, int delta_column);
    int get_line_count() const;
    std::string get_line(int line_index) const;

    bool compile_sketch();
    void upload_to_mcu();

    // Arduino keywords for syntax highlighting
    static const char* s_keywords[];
    static const char* s_types[];
    static const char* s_constants[];
};

} // namespace mechatron
