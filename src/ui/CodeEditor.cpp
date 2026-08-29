#include "CodeEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include <imgui.h>
#include "Theme.hpp"
#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace mechatron {

namespace {
int text_resize_callback(ImGuiInputTextCallbackData* data) {
    auto* text = static_cast<std::string*>(data->UserData);
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        text->resize(static_cast<size_t>(data->BufTextLen));
        data->Buf = text->data();
    }
    return 0;
}

bool input_text_string(const char* label, std::string& value) {
    if (value.capacity() == 0) value.reserve(256);
    return ImGui::InputText(
        label,
        value.data(),
        value.capacity() + 1,
        ImGuiInputTextFlags_CallbackResize,
        text_resize_callback,
        &value
    );
}

bool input_text_multiline_string(const char* label, std::string& value, const ImVec2& size) {
    if (value.capacity() == 0) value.reserve(4096);
    return ImGui::InputTextMultiline(
        label,
        value.data(),
        value.capacity() + 1,
        size,
    ImGuiInputTextFlags_CallbackResize,
        text_resize_callback,
        &value
    );
}

std::string shell_quote(const std::string& value) {
#ifdef _WIN32
    std::string quoted = "\"";
    for (char ch : value) {
        if (ch == '"') {
            quoted += "\\\"";
        } else {
            quoted += ch;
        }
    }
    quoted += "\"";
#else
    std::string quoted = "'";
    for (char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
#endif
    return quoted;
}

std::string filename_without_extension(const std::string& path) {
    return std::filesystem::path(path).stem().string();
}

std::string default_sketch_path() {
    return (std::filesystem::current_path() / "sketch.ino").string();
}

std::string find_project_file(const std::filesystem::path& relative_path) {
    namespace fs = std::filesystem;

    std::error_code ec;
    fs::path current = fs::current_path(ec);
    if (ec) return {};

    for (int depth = 0; depth < 6; ++depth) {
        fs::path candidate = current / relative_path;
        if (fs::exists(candidate, ec)) {
            return candidate.lexically_normal().string();
        }
        if (!current.has_parent_path() || current == current.parent_path()) {
            break;
        }
        current = current.parent_path();
    }
    return {};
}

std::string bundled_arduino_cli_path() {
#ifdef _WIN32
    return find_project_file("tools/arduino-cli/bin/arduino-cli.exe");
#else
    return find_project_file("tools/arduino-cli/bin/arduino-cli");
#endif
}

std::string bundled_arduino_cli_config_path() {
    return find_project_file("tools/arduino-cli/arduino-cli.yaml");
}

std::filesystem::path project_root_from_cli_config(const std::string& config_path) {
    if (config_path.empty()) return {};

    std::filesystem::path config(config_path);
    if (config.filename() != "arduino-cli.yaml") return {};

    // <project>/tools/arduino-cli/arduino-cli.yaml
    auto cli_dir = config.parent_path();
    auto tools_dir = cli_dir.parent_path();
    return tools_dir.parent_path();
}
}

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
    // Load saved configuration
    load_config();

    std::string bundled_cli = bundled_arduino_cli_path();
    if (!bundled_cli.empty()) {
        m_arduino_cli_path = bundled_cli;
    }
    std::string bundled_config = bundled_arduino_cli_config_path();
    if (!bundled_config.empty()) {
        m_arduino_cli_config_path = bundled_config;
    }

    // Default Arduino sketch template
    char template_buffer[512];
    snprintf(template_buffer, sizeof(template_buffer),
R"(// Arduino Sketch
// MECHATRON Code Editor

void setup() {
    // Initialize serial communication
    Serial.begin(%d);

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
)", m_baud_rate);
    m_editor.content = template_buffer;
}

void CodeEditor::render(SimulationOrchestrator& orchestrator) {
    // No Begin/End - we're inside a tab

    // Check for auto-save before rendering
    check_auto_save();

    render_menu_bar(orchestrator);

    // Show compilation output/errors if available
    if (!m_compile_output.empty() || !m_compile_errors.empty()) {
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Compilation Output", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!m_compile_errors.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 1.0f)));
                ImGui::TextWrapped("%s", m_compile_errors.c_str());
                ImGui::PopStyleColor();
            }
            if (!m_compile_output.empty()) {
                ImGui::TextWrapped("%s", m_compile_output.c_str());
            }
            if (ImGui::Button("Clear Output")) {
                m_compile_output.clear();
                m_compile_errors.clear();
            }
        }
        ImGui::Separator();
    }

    // Editor area
    render_editor_area();

    // Status bar
    render_status_bar();

    // Search/Replace dialog
    if (m_show_search || m_show_replace) {
        render_search_replace();
    }

    if (m_show_settings) {
        ImGui::SetNextWindowSize(ImVec2(520, 260), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Code Editor Settings", &m_show_settings)) {
            if (input_text_string("Arduino CLI", m_arduino_cli_path)) {
                save_config();
            }
            if (input_text_string("CLI Config", m_arduino_cli_config_path)) {
                save_config();
            }
            if (input_text_string("FQBN", m_fqbn)) {
                save_config();
            }
            if (ImGui::InputInt("Baud", &m_baud_rate)) {
                if (m_baud_rate < 300) m_baud_rate = 300;
                save_config();
            }
            if (ImGui::Checkbox("Auto Save", &m_auto_save_enabled)) {
                save_config();
            }
            if (ImGui::InputDouble("Auto Save Interval", &m_auto_save_interval, 1.0, 10.0, "%.1f s")) {
                if (m_auto_save_interval < 1.0) m_auto_save_interval = 1.0;
                save_config();
            }
            ImGui::Separator();
            ImGui::TextWrapped("Bundled CLI and board packages live under tools/arduino-cli.");
            ImGui::End();
        }
    }

    render_file_dialogs();
}

void CodeEditor::render_menu_bar(SimulationOrchestrator& orchestrator) {
    // Toolbar buttons instead of menu bar (we're inside a tab)
    if (ImGui::Button("New")) {
        m_editor.content = "// New Sketch\n\nvoid setup() {}\n\nvoid loop() {}\n";
        m_current_file.clear();
        m_file_modified = false;
        m_compiled_hex_path.clear();
        update_search_matches();
    }
    ImGui::SameLine();
    if (ImGui::Button("Open")) {
        m_file_dialog_path = m_current_file.empty() ? default_sketch_path() : m_current_file;
        m_show_open_dialog = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        if (m_current_file.empty()) {
            m_file_dialog_path = default_sketch_path();
            m_show_save_as_dialog = true;
        } else {
            save_file(m_current_file);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save As")) {
        m_file_dialog_path = m_current_file.empty() ? default_sketch_path() : m_current_file;
        m_show_save_as_dialog = true;
    }
    ImGui::SameLine();
    if (ImGui::BeginCombo("Recent##CodeEditor", m_recent_files.empty() ? "Recent" : "Recent")) {
        if (m_recent_files.empty()) {
            ImGui::TextDisabled("No recent files");
        }
        for (const auto& path : m_recent_files) {
            if (ImGui::Selectable(path.c_str())) {
                load_file(path);
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();

    // Compile button
    if (m_compiling) {
        ImGui::BeginDisabled();
        ImGui::Button("Compiling...");
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Compile")) {
            compile_sketch();
        }
    }
    ImGui::SameLine();
    render_mcu_selector(orchestrator);
    ImGui::SameLine();

    // Upload button
    if (m_uploading) {
        ImGui::BeginDisabled();
        ImGui::Button("Uploading...");
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Upload")) {
            upload_to_mcu(orchestrator);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Find")) {
        m_show_search = !m_show_search;
        m_show_replace = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Replace")) {
        m_show_replace = !m_show_replace;
        m_show_search = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Settings")) {
        m_show_settings = !m_show_settings;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Line##", &m_editor.show_line_numbers);
    ImGui::SetNextItemWidth(100);

    // Display Arduino CLI configuration
    ImGui::SameLine();
    ImGui::TextDisabled("(CLI: %s | %s)",
        m_arduino_cli_path.c_str(),
        m_fqbn.c_str());
}

void CodeEditor::render_mcu_selector(SimulationOrchestrator& orchestrator) {
    std::vector<std::string> mcu_ids;
    orchestrator.registry().for_each([&](Component& comp) {
        if (comp.category() == "mcu") {
            mcu_ids.push_back(comp.id());
        }
    });
    std::sort(mcu_ids.begin(), mcu_ids.end());

    if (!m_selected_mcu_id.empty() &&
        std::find(mcu_ids.begin(), mcu_ids.end(), m_selected_mcu_id) == mcu_ids.end()) {
        m_selected_mcu_id.clear();
    }
    if (m_selected_mcu_id.empty() && !mcu_ids.empty()) {
        m_selected_mcu_id = mcu_ids.front();
    }

    const char* preview = m_selected_mcu_id.empty() ? "No MCU" : m_selected_mcu_id.c_str();
    ImGui::SetNextItemWidth(150);
    if (ImGui::BeginCombo("MCU##UploadTarget", preview)) {
        if (mcu_ids.empty()) {
            ImGui::TextDisabled("No MCU in circuit");
        }
        for (const auto& id : mcu_ids) {
            bool selected = (id == m_selected_mcu_id);
            if (ImGui::Selectable(id.c_str(), selected)) {
                m_selected_mcu_id = id;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

void CodeEditor::render_editor_area() {
    ImVec2 size = ImGui::GetContentRegionAvail();
    size.y -= ImGui::GetFrameHeightWithSpacing(); // Reserve space for status bar

    if (m_editor.show_line_numbers) {
        ImGui::BeginChild("LineNumbers", ImVec2(46, size.y), false, ImGuiWindowFlags_NoScrollbar);
        int lines = get_line_count();
        for (int i = 1; i <= lines; ++i) {
            ImGui::TextDisabled("%4d", i);
        }
        ImGui::EndChild();
        ImGui::SameLine();
        size.x -= 52;
    }

    if (input_text_multiline_string("##CodeEditor", m_editor.content, size)) {
        m_file_modified = true;
        m_compiled_hex_path.clear();
        update_syntax_highlighting();
        update_search_matches();
    }
    if (ImGui::IsItemActive()) {
        ImGuiInputTextState* state = ImGui::GetInputTextState(ImGui::GetItemID());
        if (state) {
            update_cursor_from_index(static_cast<size_t>(state->GetCursorPos()));
        }
    }
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
            update_search_matches();
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
        if (ImGui::Button("Find Next") && m_total_matches > 0) {
            m_current_match = (m_current_match + 1) % m_total_matches;
        }
        ImGui::SameLine();
        if (ImGui::Button("Find Previous") && m_total_matches > 0) {
            m_current_match = (m_current_match + m_total_matches - 1) % m_total_matches;
        }

        if (m_show_replace) {
            ImGui::SameLine();
            if (ImGui::Button("Replace") && !std::string_view(m_search_buffer).empty()) {
                size_t pos = m_editor.content.find(m_search_buffer);
                if (pos != std::string::npos) {
                    m_editor.content.replace(pos, std::strlen(m_search_buffer), m_replace_buffer);
                    m_file_modified = true;
                    m_compiled_hex_path.clear();
                    update_search_matches();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Replace All") && !std::string_view(m_search_buffer).empty()) {
                size_t pos = 0;
                size_t find_len = std::strlen(m_search_buffer);
                size_t replace_len = std::strlen(m_replace_buffer);
                while ((pos = m_editor.content.find(m_search_buffer, pos)) != std::string::npos) {
                    m_editor.content.replace(pos, find_len, m_replace_buffer);
                    pos += replace_len;
                }
                m_file_modified = true;
                m_compiled_hex_path.clear();
                update_search_matches();
            }
        }

        ImGui::End();
    } else {
        m_show_search = false;
        m_show_replace = false;
    }
}

void CodeEditor::render_file_dialogs() {
    if (m_show_open_dialog) {
        ImGui::OpenPopup("Open Sketch");
        m_show_open_dialog = false;
    }
    if (m_show_save_as_dialog) {
        ImGui::OpenPopup("Save Sketch As");
        m_show_save_as_dialog = false;
    }

    if (ImGui::BeginPopupModal("Open Sketch", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        input_text_string("Path", m_file_dialog_path);
        ImGui::Spacing();
        if (ImGui::Button("Open", ImVec2(100, 0))) {
            load_file(m_file_dialog_path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("Save Sketch As", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        input_text_string("Path", m_file_dialog_path);
        ImGui::Spacing();
        if (ImGui::Button("Save", ImVec2(100, 0))) {
            save_file(m_file_dialog_path);
            add_recent_file(m_file_dialog_path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
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
                        hr.color = Theme::U32(Theme::CurrentPalette().primaryHover);
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
                        hr.color = Theme::U32(Theme::CurrentPalette().success);
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
                        hr.color = Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 1.0f));
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
        if (token == *kw) return Theme::U32(Theme::CurrentPalette().primaryHover);
    }
    // Types/functions: green
    for (const char** kw = s_types; *kw; ++kw) {
        if (token == *kw) return Theme::U32(Theme::CurrentPalette().success);
    }
    // Constants: red/orange
    for (const char** kw = s_constants; *kw; ++kw) {
        if (token == *kw) return Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 1.0f));
    }
    return Theme::U32(Theme::CurrentPalette().text);
}

void CodeEditor::update_cursor_from_index(size_t cursor_index) {
    cursor_index = std::min(cursor_index, m_editor.content.size());

    int line = 1;
    int column = 1;
    for (size_t i = 0; i < cursor_index; ++i) {
        if (m_editor.content[i] == '\n') {
            ++line;
            column = 1;
        } else {
            ++column;
        }
    }

    m_editor.cursor_line = line;
    m_editor.cursor_column = column;
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
        m_compiled_hex_path.clear();
        update_search_matches();
        spdlog::info("Loaded file: {}", path);

        // Add to recent files history
        add_recent_file(path);
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

        // Track save time for auto-save
        auto now = std::chrono::steady_clock::now();
        m_last_save_time = std::chrono::duration<double>(now.time_since_epoch()).count();

        spdlog::info("Saved file: {}", path);
    } else {
        spdlog::error("Failed to save file: {}", path);
    }
}

void CodeEditor::update_search_matches() {
    std::string needle = m_search_buffer;
    if (needle.empty()) {
        m_total_matches = 0;
        m_current_match = 0;
        return;
    }

    int count = 0;
    size_t pos = 0;
    while ((pos = m_editor.content.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    m_total_matches = count;
    if (m_total_matches == 0) {
        m_current_match = 0;
    } else if (m_current_match >= m_total_matches) {
        m_current_match = m_total_matches - 1;
    }
}

bool CodeEditor::check_arduino_cli() {
    spdlog::info("Checking Arduino CLI availability...");

    std::string output = run_arduino_command({"version"});

    if (output.empty()) {
        m_compile_errors = "Arduino CLI not found. Please install arduino-cli or set the correct path.";
        spdlog::error(m_compile_errors);
        return false;
    }

    // Parse version output
    if (output.find("arduino-cli") != std::string::npos ||
        output.find("Version") != std::string::npos) {
        spdlog::info("Arduino CLI detected: {}", output);

        // Auto-install required core if not present
        ensure_core_installed();

        return true;
    }

    m_compile_errors = "Arduino CLI executable found but returned unexpected output: " + output;
    spdlog::error(m_compile_errors);
    return false;
}

void CodeEditor::ensure_core_installed() {
    // Extract core from FQBN (e.g. "arduino:avr:uno" -> "arduino:avr")
    std::string core;
    int colon_count = 0;
    for (char c : m_fqbn) {
        if (c == ':') {
            colon_count++;
            if (colon_count == 2) break;
        }
        core += c;
    }

    if (colon_count < 2) return;

    // Check if core is already installed
    std::string list_output = run_arduino_command({"core", "list", "--format", "json"});
    bool installed = false;

    try {
        auto json = nlohmann::json::parse(list_output);
        if (json.is_array()) {
            for (const auto& item : json) {
                if (item.value("id", "") == core) {
                    installed = true;
                    break;
                }
            }
        }
    } catch (...) {
        // If we can't parse, try a simpler text check
        installed = list_output.find(core) != std::string::npos;
    }

    if (installed) return;

    // Update core index first
    spdlog::info("Updating Arduino core index for {}...", core);
    run_arduino_command({"core", "update-index"});

    // Install the core
    spdlog::info("Installing Arduino core: {}...", core);
    std::string install_output = run_arduino_command({"core", "install", core});
    spdlog::info("Core install output: {}", install_output);
    spdlog::info("Arduino core {} installed successfully", core);
}

std::string CodeEditor::run_arduino_command(const std::vector<std::string>& args) {
    std::string output;
    std::string errors;

    // Build command line
    std::string cmd = m_arduino_cli_path;
    cmd = shell_quote(cmd);
    if (!m_arduino_cli_config_path.empty()) {
        cmd += " --config-file " + shell_quote(m_arduino_cli_config_path);
    }
    for (const auto& arg : args) {
        cmd += " " + shell_quote(arg);
    }
    cmd += " 2>&1";

    auto project_root = project_root_from_cli_config(m_arduino_cli_config_path);
    if (!project_root.empty()) {
#ifdef _WIN32
        cmd = "cd /d " + shell_quote(project_root.string()) + " && " + cmd;
#else
        cmd = "cd " + shell_quote(project_root.string()) + " && " + cmd;
#endif
    }

#ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "r");
#else
    FILE* pipe = popen(cmd.c_str(), "r");
#endif

    if (!pipe) {
        spdlog::error("Failed to execute Arduino CLI command: {}", cmd);
        return "";
    }

    // Read output
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }

#ifdef _WIN32
    int result = _pclose(pipe);
#else
    int result = pclose(pipe);
#endif

    if (result != 0) {
        spdlog::warn("Arduino CLI command exited with code: {}", result);
    }

    return output;
}

std::string CodeEditor::prepare_sketch_for_compile() {
    namespace fs = std::filesystem;

    if (!m_current_file.empty()) {
        save_file(m_current_file);
        return fs::path(m_current_file).parent_path().string();
    }

    fs::path sketch_dir = fs::temp_directory_path() / "mechatron_sketch";
    std::error_code ec;
    fs::create_directories(sketch_dir, ec);
    if (ec) {
        m_compile_errors = "Failed to create temporary sketch directory: " + ec.message();
        return {};
    }

    fs::path sketch_file = sketch_dir / "mechatron_sketch.ino";
    std::ofstream file(sketch_file);
    if (!file.is_open()) {
        m_compile_errors = "Failed to write temporary sketch: " + sketch_file.string();
        return {};
    }
    file << m_editor.content;
    return sketch_dir.string();
}

std::string CodeEditor::find_hex_file(const std::string& build_path, const std::string& sketch_name) const {
    namespace fs = std::filesystem;

    if (build_path.empty()) return {};
    std::error_code ec;
    if (!fs::exists(build_path, ec)) return {};

    fs::path expected = fs::path(build_path) / (sketch_name + ".ino.hex");
    if (fs::exists(expected, ec)) {
        return expected.string();
    }

    for (const auto& entry : fs::recursive_directory_iterator(build_path, fs::directory_options::skip_permission_denied, ec)) {
        if (ec) break;
        if (!entry.is_regular_file(ec)) continue;
        if (entry.path().extension() == ".hex") {
            return entry.path().string();
        }
    }
    return {};
}

bool CodeEditor::compile_sketch() {
    spdlog::info("Compiling sketch...");
    m_compiling = true;
    m_compile_output.clear();
    m_compile_errors.clear();
    m_compiled_hex_path.clear();

    // Check if Arduino CLI is available
    if (!check_arduino_cli()) {
        m_compiling = false;
        return false;
    }

    std::string sketch_dir = prepare_sketch_for_compile();
    if (sketch_dir.empty()) {
        m_compiling = false;
        return false;
    }
    std::string sketch_name = filename_without_extension(std::filesystem::path(sketch_dir).filename().string());

    std::string build_path;
    auto project_root = project_root_from_cli_config(m_arduino_cli_config_path);
    if (!project_root.empty()) {
        std::filesystem::path build_dir = project_root / "tools" / "arduino-cli" / "build-cache" / sketch_name;
        std::error_code ec;
        std::filesystem::remove_all(build_dir, ec);
        ec.clear();
        std::filesystem::create_directories(build_dir, ec);
        if (!ec) {
            build_path = build_dir.string();
        }
    }

    // Build compile command
    std::vector<std::string> compile_args = {
        "compile",
        "--fqbn", m_fqbn,
        "--format", "json",
        sketch_dir
    };
    if (!build_path.empty()) {
        compile_args.insert(compile_args.end() - 1, {"--build-path", build_path});
    }

    spdlog::info("Running: {} compile --fqbn {}", m_arduino_cli_path, m_fqbn);

    // Execute compilation
    std::string result = run_arduino_command(compile_args);
    m_compile_output = result;

    try {
        auto json = nlohmann::json::parse(result);
        bool success = json.value("success", false);
        if (success) {
            std::string build_path;
            if (json.contains("builder_result") && json["builder_result"].contains("build_path")) {
                build_path = json["builder_result"]["build_path"].get<std::string>();
            } else if (json.contains("build_path")) {
                build_path = json["build_path"].get<std::string>();
            }

            m_compiled_hex_path = find_hex_file(build_path, sketch_name);
            if (m_compiled_hex_path.empty()) {
                m_compile_errors = "Compilation succeeded, but no .hex file was found in build path: " + build_path;
                m_compiling = false;
                return false;
            }

            spdlog::info("Compiled hex file: {}", m_compiled_hex_path);
            m_compiling = false;
            return true;
        }

        if (json.contains("error")) {
            m_compile_errors = json["error"].dump();
        } else {
            m_compile_errors = "Compilation failed. Check output for details.";
        }
    } catch (const std::exception& e) {
        if (result.find("Error") != std::string::npos || result.find("error") != std::string::npos) {
            m_compile_errors = result;
        } else {
            m_compile_errors = std::string("Could not parse Arduino CLI JSON output: ") + e.what() + "\n" + result;
        }
    }

    spdlog::error("Compilation failed: {}", m_compile_errors);
    m_compiling = false;
    return false;
}

bool CodeEditor::upload_to_mcu(SimulationOrchestrator& orchestrator) {
    spdlog::info("Uploading to MCU...");
    m_uploading = true;

    // Always compile immediately before upload so external edits or stale build
    // cache cannot result in an old firmware being loaded into the MCU.
    if (!compile_sketch()) {
        if (m_compile_errors.empty()) {
            m_compile_errors = "Upload failed: Compilation failed";
        } else {
            m_compile_errors = "Upload failed: Compilation failed\n" + m_compile_errors;
        }
        m_uploading = false;
        return false;
    }

    // Verify hex file exists
    std::error_code ec;
    if (m_compiled_hex_path.empty() || !std::filesystem::exists(m_compiled_hex_path, ec)) {
        m_compile_errors = "Upload failed: No compiled hex file available";
        spdlog::error(m_compile_errors);
        m_uploading = false;
        return false;
    }

    // Find selected MCU component in the registry
    auto& registry = orchestrator.registry();

    Component* mcu_component = nullptr;
    std::string mcu_id;
    if (!m_selected_mcu_id.empty()) {
        Component* selected = registry.get(m_selected_mcu_id);
        if (selected && selected->category() == "mcu") {
            mcu_component = selected;
            mcu_id = selected->id();
        }
    }

    if (!mcu_component) {
        m_compile_errors = "No MCU selected. Please add an ATmega328P to the circuit and select it in the MCU dropdown.";
        spdlog::warn(m_compile_errors);
        m_uploading = false;
        return false;
    }

    spdlog::info("Found MCU component: {}", mcu_id);

    if (!mcu_component->load_firmware_file(m_compiled_hex_path)) {
        m_compile_errors = "Upload failed: MCU rejected firmware file: " + m_compiled_hex_path;
        spdlog::error(m_compile_errors);
        m_uploading = false;
        return false;
    }
    orchestrator.mark_circuit_topology_dirty();

    m_compile_output = "Firmware compiled: " + m_compiled_hex_path + "\n";
    m_compile_output += "Uploaded to MCU: " + mcu_id + "\n";

    spdlog::info("Firmware compiled at: {}", m_compiled_hex_path);
    spdlog::info("Firmware uploaded to MCU component: {}", mcu_id);

    m_uploading = false;
    return true;
}

void CodeEditor::add_recent_file(const std::string& path) {
    // Remove if already exists
    auto it = std::find(m_recent_files.begin(), m_recent_files.end(), path);
    if (it != m_recent_files.end()) {
        m_recent_files.erase(it);
    }

    // Add to front
    m_recent_files.insert(m_recent_files.begin(), path);

    // Limit to MAX_RECENT_FILES
    if (m_recent_files.size() > MAX_RECENT_FILES) {
        m_recent_files.resize(MAX_RECENT_FILES);
    }

    // Save configuration
    save_config();
}

void CodeEditor::save_config() {
    nlohmann::json config;
    config["recent_files"] = m_recent_files;
    config["arduino_cli_path"] = m_arduino_cli_path;
    config["arduino_cli_config_path"] = m_arduino_cli_config_path;
    config["fqbn"] = m_fqbn;
    config["baud_rate"] = m_baud_rate;
    config["auto_save_enabled"] = m_auto_save_enabled;
    config["auto_save_interval"] = m_auto_save_interval;

    try {
        std::ofstream out(config_file_path());
        out << config.dump(2);
        spdlog::debug("[CodeEditor] Configuration saved to {}", config_file_path());
    } catch (const std::exception& e) {
        spdlog::warn("[CodeEditor] Failed to save configuration: {}", e.what());
    }
}

void CodeEditor::load_config() {
    try {
        std::ifstream in(config_file_path());
        if (!in.is_open()) {
            spdlog::debug("[CodeEditor] No configuration file found, using defaults");
            return;
        }

        nlohmann::json config;
        in >> config;

        if (config.contains("recent_files")) {
            m_recent_files = config["recent_files"].get<std::vector<std::string>>();
        }
        if (config.contains("arduino_cli_path")) {
            m_arduino_cli_path = config["arduino_cli_path"];
        }
        if (config.contains("arduino_cli_config_path")) {
            m_arduino_cli_config_path = config["arduino_cli_config_path"];
        }
        if (config.contains("fqbn")) {
            m_fqbn = config["fqbn"];
        }
        if (config.contains("baud_rate")) {
            m_baud_rate = config["baud_rate"];
        }
        if (config.contains("auto_save_enabled")) {
            m_auto_save_enabled = config["auto_save_enabled"];
        }
        if (config.contains("auto_save_interval")) {
            m_auto_save_interval = config["auto_save_interval"];
        }

        spdlog::info("[CodeEditor] Configuration loaded: {} recent files", m_recent_files.size());
    } catch (const std::exception& e) {
        spdlog::warn("[CodeEditor] Failed to load configuration: {}", e.what());
    }
}

void CodeEditor::set_auto_save_enabled(bool enabled) {
    m_auto_save_enabled = enabled;
    save_config();  // Save configuration when changed
}

bool CodeEditor::is_auto_save_enabled() const {
    return m_auto_save_enabled;
}

void CodeEditor::set_auto_save_interval(double seconds) {
    m_auto_save_interval = seconds;
    save_config();  // Save configuration when changed
}

double CodeEditor::auto_save_interval() const {
    return m_auto_save_interval;
}

void CodeEditor::check_auto_save() {
    if (!m_auto_save_enabled || m_current_file.empty()) {
        return;
    }

    // Get current time
    auto now = std::chrono::steady_clock::now();
    auto now_time = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Check if auto-save interval has elapsed
    if (now_time - m_last_auto_save_time >= m_auto_save_interval) {
        // Only auto-save if content has been modified since last save
        if (m_file_modified) {
            save_file(m_current_file);
            m_last_auto_save_time = now_time;
            spdlog::debug("[CodeEditor] Auto-saved: {}", m_current_file);
        }
    }
}

} // namespace mechatron
