#include "CodeEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace mechatron {

// Arduino syntax keywords
const char* CodeEditor::s_keywords[] = {
    "if", "else", "for", "while", "do", "switch", "case", "break", "continue",
    "return", "goto", "default", "sizeof", "typeof",
    "void", "int", "float", "double", "char", "unsigned", "signed", "long", "short",
    "bool", "byte", "word", "String", "array", "static", "const", "volatile",
    "class", "struct", "union", "enum", "public", "private", "protected",
    "virtual", "inline", "explicit", "friend", "template", "typename",
    nullptr
};

const char* CodeEditor::s_types[] = {
    "setup", "loop", "pinMode", "digitalWrite", "digitalRead",
    "analogWrite", "analogRead", "delay", "delayMicroseconds",
    "millis", "micros", "Serial", "begin", "print", "println",
    "available", "read", "write", "flush", nullptr
};

const char* CodeEditor::s_constants[] = {
    "HIGH", "LOW", "INPUT", "OUTPUT", "INPUT_PULLUP",
    "LED_BUILTIN", "true", "false", "nullptr", nullptr
};

CodeEditor::CodeEditor() {
    // Default Arduino sketch template
    m_editor.content =
R"(// Arduino Sketch
// MECHATRON Code Editor

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize pins
    pinMode(13, OUTPUT);
}

void loop() {
    // Your code here
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
}
)";
}

void CodeEditor::render(SimulationOrchestrator& orchestrator) {
    ImGui::Begin("Code Editor");

    render_menu_bar();

    // Editor area
    render_editor_area();

    // Status bar
    render_status_bar();

    // Search/Replace dialog
    if (m_show_search || m_show_replace) {
        render_search_replace();
    }

    ImGui::End();
}

void CodeEditor::render_menu_bar() {
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New", "Ctrl+N")) {
                m_editor.content = "// New Sketch\n\nvoid setup() {}\n\nvoid loop() {}\n";
                m_current_file.clear();
                m_file_modified = false;
            }
            if (ImGui::MenuItem("Open", "Ctrl+O")) {
                // File dialog would go here
                spdlog::info("Open file dialog (not implemented)");
            }
            if (ImGui::MenuItem("Save", "Ctrl+S", false, !m_current_file.empty())) {
                save_file(m_current_file);
            }
            if (ImGui::MenuItem("Save As...", "Ctrl+Shift+S")) {
                spdlog::info("Save As dialog (not implemented)");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Close", nullptr, false, !m_current_file.empty())) {
                m_current_file.clear();
                m_file_modified = false;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", "Ctrl+Z")) {}
            if (ImGui::MenuItem("Redo", "Ctrl+Y")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Cut", "Ctrl+X")) {}
            if (ImGui::MenuItem("Copy", "Ctrl+C")) {}
            if (ImGui::MenuItem("Paste", "Ctrl+V")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Find", "Ctrl+F", &m_show_search)) {}
            if (ImGui::MenuItem("Replace", "Ctrl+H", &m_show_replace)) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Line Numbers", nullptr, &m_editor.show_line_numbers);
            ImGui::MenuItem("Show Whitespace", nullptr, &m_editor.show_whitespace);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Sketch")) {
            if (ImGui::MenuItem("Verify/Compile", "Ctrl+R")) {
                compile_sketch();
            }
            if (ImGui::MenuItem("Upload", "Ctrl+U")) {
                upload_to_mcu();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Export Hex", nullptr)) {
                spdlog::info("Export hex (not implemented)");
            }
            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
}

void CodeEditor::render_editor_area() {
    ImGui::BeginChild("EditorArea", ImVec2(0, -20), true);

    // Simple multiline text editor
    ImVec2 size = ImGui::GetContentRegionAvail();
    if (m_editor.show_line_numbers) {
        // Reserve space for line numbers
        ImGui::SetCursorPosX(40);
        size.x -= 40;
    }

    // Get buffer pointer for ImGui
    int content_size = static_cast<int>(m_editor.content.size()) + 1024;
    m_editor.content.reserve(content_size);

    if (ImGui::InputTextMultiline("##CodeEditor", &m_editor.content[0], m_editor.content.capacity(), size)) {
        m_file_modified = true;
        update_syntax_highlighting();
    }

    ImGui::EndChild();
}

void CodeEditor::render_status_bar() {
    ImGui::Text("Ln %d, Col %d | %s | %s",
        m_editor.cursor_line,
        m_editor.cursor_column,
        m_file_modified ? "Modified" : "Saved",
        m_current_file.empty() ? "Untitled" : m_current_file.c_str());
}

void CodeEditor::render_search_replace() {
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetMainViewport()->Size.x * 0.5f, 100), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0));
    ImGui::SetNextWindowSize(ImVec2(400, 200), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Find & Replace", &m_show_search)) {
        if (ImGui::InputText("Find", m_search_buffer, 256)) {
            // Update search results
        }

        if (m_show_replace) {
            ImGui::InputText("Replace with", m_replace_buffer, 256);
        }

        if (m_total_matches > 0) {
            ImGui::Text("%d / %d matches", m_current_match + 1, m_total_matches);
        } else if (std::string_view(m_search_buffer).empty()) {
            ImGui::TextDisabled("Enter search term");
        } else {
            ImGui::Text("No matches found");
        }

        ImGui::Spacing();
        if (ImGui::Button("Find Next")) {}
        ImGui::SameLine();
        if (ImGui::Button("Find Previous")) {}

        if (m_show_replace) {
            ImGui::SameLine();
            if (ImGui::Button("Replace")) {}
            ImGui::SameLine();
            if (ImGui::Button("Replace All")) {}
        }

        ImGui::End();
    } else {
        m_show_search = false;
        m_show_replace = false;
    }
}

void CodeEditor::update_syntax_highlighting() {
    m_highlight_ranges.clear();

    // Very basic syntax highlighting (would need proper tokenizer for production)
    std::istringstream stream(m_editor.content);
    std::string line;
    int line_num = 0;

    while (std::getline(stream, line)) {
        int pos = 0;
        while (pos < static_cast<int>(line.length())) {
            // Skip whitespace
            while (pos < static_cast<int>(line.length()) && std::isspace(line[pos])) {
                pos++;
            }

            if (pos >= static_cast<int>(line.length())) break;

            // Find token end
            int token_start = pos;
            while (pos < static_cast<int>(line.length()) &&
                   (std::isalnum(line[pos]) || line[pos] == '_')) {
                pos++;
            }

            if (token_start < pos) {
                std::string token = line.substr(token_start, pos - token_start);

                // Check if keyword
                for (const char** kw = s_keywords; *kw; ++kw) {
                    if (token == *kw) {
                        HighlightRange hr;
                        hr.start = token_start;
                        hr.end = pos;
                        hr.color = IM_COL32(150, 150, 255, 255);
                        m_highlight_ranges.push_back(hr);
                        break;
                    }
                }

                // Check if type/function
                for (const char** kw = s_types; *kw; ++kw) {
                    if (token == *kw) {
                        HighlightRange hr;
                        hr.start = token_start;
                        hr.end = pos;
                        hr.color = IM_COL32(100, 200, 100, 255);
                        m_highlight_ranges.push_back(hr);
                        break;
                    }
                }

                // Check if constant
                for (const char** kw = s_constants; *kw; ++kw) {
                    if (token == *kw) {
                        HighlightRange hr;
                        hr.start = token_start;
                        hr.end = pos;
                        hr.color = IM_COL32(255, 100, 100, 255);
                        m_highlight_ranges.push_back(hr);
                        break;
                    }
                }
            } else {
                pos++;
            }
        }
        line_num++;
    }
}

ImU32 CodeEditor::get_color_for_token(const std::string& token) {
    // Keywords: blue/purple
    for (const char** kw = s_keywords; *kw; ++kw) {
        if (token == *kw) return IM_COL32(150, 150, 255, 255);
    }
    // Types/functions: green
    for (const char** kw = s_types; *kw; ++kw) {
        if (token == *kw) return IM_COL32(100, 200, 100, 255);
    }
    // Constants: red/orange
    for (const char** kw = s_constants; *kw; ++kw) {
        if (token == *kw) return IM_COL32(255, 100, 100, 255);
    }
    return IM_COL32(255, 255, 255, 255);
}

int CodeEditor::get_line_count() const {
    return std::count(m_editor.content.begin(), m_editor.content.end(), '\n') + 1;
}

std::string CodeEditor::get_line(int line_index) const {
    std::istringstream stream(m_editor.content);
    std::string line;
    for (int i = 0; i <= line_index && std::getline(stream, line); ++i) {
        // Skip to desired line
    }
    return line;
}

void CodeEditor::load_file(const std::string& path) {
    std::ifstream file(path);
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        m_editor.content = buffer.str();
        m_current_file = path;
        m_file_modified = false;
        spdlog::info("Loaded file: {}", path);
    } else {
        spdlog::error("Failed to open file: {}", path);
    }
}

void CodeEditor::save_file(const std::string& path) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << m_editor.content;
        m_current_file = path;
        m_file_modified = false;
        spdlog::info("Saved file: {}", path);
    } else {
        spdlog::error("Failed to save file: {}", path);
    }
}

bool CodeEditor::compile_sketch() {
    spdlog::info("Compiling sketch...");
    // TODO: Integrate with Arduino CLI or avr-gcc
    // For now, just check syntax
    spdlog::info("Compile: Verify (not yet implemented)");
    return true;
}

void CodeEditor::upload_to_mcu() {
    if (compile_sketch()) {
        spdlog::info("Uploading to MCU...");
        // TODO: Integrate with QEMU for upload
        spdlog::info("Upload: Complete (not yet implemented)");
    }
}

} // namespace mechatron
