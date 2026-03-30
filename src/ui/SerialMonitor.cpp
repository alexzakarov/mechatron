#include "SerialMonitor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <ctime>

namespace mechatron {

SerialMonitor::SerialMonitor() {
    // Add welcome message
    add_message(SerialMessage::Type::Info, "Serial Monitor initialized");
    add_message(SerialMessage::Type::Info, "Connect to a port to begin communication");
}

void SerialMonitor::render(SimulationOrchestrator& orchestrator) {
    ImGui::Begin("Serial Monitor");

    render_menu_bar();

    // Connection status bar
    if (m_connected) {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "[CONNECTED] %s @ %d baud", m_port.c_str(), m_baud_rate);
    } else {
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "[DISCONNECTED]");
    }

    ImGui::SameLine();
    if (m_connected) {
        if (ImGui::SmallButton("Disconnect")) {
            disconnect();
        }
    } else {
        if (ImGui::SmallButton("Connect")) {
            connect();
        }
    }

    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &m_auto_scroll);
    ImGui::SameLine();
    ImGui::Checkbox("Timestamp", &m_show_timestamp);
    ImGui::SameLine();
    ImGui::Checkbox("Hex Mode", &m_hex_mode);

    // Output area
    render_output_area();

    // Input area
    render_input_area();

    // Statistics (optional)
    if (m_show_stats) {
        render_stats();
    }

    ImGui::End();
}

void SerialMonitor::render_menu_bar() {
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Clear Log", "Ctrl+L")) {
                clear();
            }
            if (ImGui::MenuItem("Save Log...", "Ctrl+S")) {
                spdlog::info("Save log (not implemented)");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Settings", nullptr, &m_show_settings);
            ImGui::MenuItem("Statistics", nullptr, &m_show_stats);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Format")) {
            if (ImGui::MenuItem("ASCII", nullptr, !m_hex_mode)) {
                m_hex_mode = false;
            }
            if (ImGui::MenuItem("HEX", nullptr, m_hex_mode)) {
                m_hex_mode = true;
            }
            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
}

void SerialMonitor::render_output_area() {
    ImGuiWindowFlags flags = ImGuiWindowFlags_HorizontalScrollbar;
    ImGui::BeginChild("SerialOutput", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() * 2), true, flags);

    // Display messages
    for (const auto& msg : m_messages) {
        // Set color based on message type
        ImU32 color = IM_COL32(255, 255, 255, 255);
        const char* prefix = "";

        switch (msg.type) {
            case SerialMessage::Type::Rx:
                color = IM_COL32(100, 200, 255, 255);
                prefix = "[RX] ";
                break;
            case SerialMessage::Type::Tx:
                color = IM_COL32(255, 200, 100, 255);
                prefix = "[TX] ";
                break;
            case SerialMessage::Type::Info:
                color = IM_COL32(150, 150, 150, 255);
                prefix = "[INFO] ";
                break;
            case SerialMessage::Type::Error:
                color = IM_COL32(255, 100, 100, 255);
                prefix = "[ERROR] ";
                break;
        }

        // Format message
        std::string formatted = format_message(msg);
        formatted = prefix + formatted;

        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextUnformatted(formatted.c_str());
        ImGui::PopStyleColor();
    }

    // Auto-scroll to bottom if enabled
    if (m_auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
}

void SerialMonitor::render_input_area() {
    ImGui::Spacing();

    // Input text field
    ImGuiInputTextFlags flags = ImGuiInputTextFlags_EnterReturnsTrue;
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 100);

    if (ImGui::InputText("##SerialInput", m_input_buffer, 256, flags)) {
        send(m_input_buffer);
        m_input_buffer[0] = '\0';
        ImGui::SetKeyboardFocusHere(-1);
    }

    ImGui::SameLine();

    // Send button
    if (ImGui::Button("Send", ImVec2(80, 0))) {
        send(m_input_buffer);
        m_input_buffer[0] = '\0';
    }

    // Line ending options
    ImGui::Spacing();
    ImGui::Text("Line Ending:");
    ImGui::SameLine();

    static int line_ending = 0;
    ImGui::RadioButton("No line ending", &line_ending, 0);
    ImGui::SameLine();
    ImGui::RadioButton("NL", &line_ending, 1);
    ImGui::SameLine();
    ImGui::RadioButton("CR", &line_ending, 2);
    ImGui::SameLine();
    ImGui::RadioButton("NL & CR", &line_ending, 3);

    m_append_newline = (line_ending == 1 || line_ending == 3);
    m_append_cr = (line_ending == 2 || line_ending == 3);

    // Settings panel
    if (m_show_settings) {
        ImGui::Separator();
        ImGui::Text("Port:");
        ImGui::SameLine();

        static char port_buf[64];
        std::strncpy(port_buf, m_port.c_str(), 63);
        port_buf[63] = '\0';
        if (ImGui::InputText("##Port", port_buf, 64)) {
            m_port = port_buf;
        }

        ImGui::SameLine();
        ImGui::Text("Baud:");
        ImGui::SameLine();

        static int baud_preset = 2;  // Default to 9600
        const char* baud_rates[] = {"300", "1200", "9600", "19200", "38400", "57600", "115200"};
        if (ImGui::Combo("##Baud", &baud_preset, baud_rates, 7)) {
            m_baud_rate = std::atoi(baud_rates[baud_preset]);
        }
    }
}

void SerialMonitor::render_settings_panel() {
    if (ImGui::Begin("Serial Monitor Settings")) {
        ImGui::Text("Connection Settings");

        static char port_buf[64];
        std::strncpy(port_buf, m_port.c_str(), 63);
        port_buf[63] = '\0';
        if (ImGui::InputText("Port", port_buf, 64)) {
            m_port = port_buf;
        }

        ImGui::InputInt("Baud Rate", &m_baud_rate);

        ImGui::Separator();
        ImGui::Text("Display Settings");

        ImGui::Checkbox("Auto-scroll", &m_auto_scroll);
        ImGui::Checkbox("Show Timestamp", &m_show_timestamp);
        ImGui::Checkbox("Hex Mode", &m_hex_mode);

        ImGui::End();
    }
}

void SerialMonitor::render_stats() {
    ImGui::Separator();
    ImGui::Text("Statistics:");
    ImGui::Text("Sent: %zu bytes (%zu messages)", m_stats.bytes_sent, m_stats.messages_sent);
    ImGui::Text("Received: %zu bytes (%zu messages)", m_stats.bytes_received, m_stats.messages_received);

    if (m_stats.first_message_time > 0 && m_stats.last_message_time > 0) {
        double elapsed = m_stats.last_message_time - m_stats.first_message_time;
        ImGui::Text("Session time: %.2f seconds", elapsed);
    }
}

void SerialMonitor::clear() {
    m_messages.clear();
    add_message(SerialMessage::Type::Info, "Log cleared");
}

void SerialMonitor::send(const std::string& data) {
    if (!m_connected) {
        add_message(SerialMessage::Type::Error, "Not connected - cannot send");
        return;
    }

    if (data.empty()) return;

    std::string to_send = data;

    // Add line ending
    if (m_append_newline) to_send += '\n';
    if (m_append_cr) to_send += '\r';

    // Convert to hex if needed
    if (m_hex_mode) {
        to_send = hex_to_string(to_send);
    }

    // Send data (placeholder - would use actual serial port)
    add_message(SerialMessage::Type::Tx, data);

    // Update stats
    m_stats.bytes_sent += to_send.size();
    m_stats.messages_sent++;

    spdlog::debug("Serial TX: {}", to_send);
}

void SerialMonitor::connect() {
    // Placeholder - would use actual serial port library
    m_connected = true;
    add_message(SerialMessage::Type::Info, "Connected to " + m_port);
    spdlog::info("Serial connected: {} @ {} baud", m_port, m_baud_rate);
}

void SerialMonitor::disconnect() {
    m_connected = false;
    add_message(SerialMessage::Type::Info, "Disconnected from " + m_port);
    spdlog::info("Serial disconnected");
}

void SerialMonitor::add_message(SerialMessage::Type type, const std::string& content) {
    SerialMessage msg;
    msg.type = type;
    msg.content = content;

    // Get current time
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    msg.timestamp = std::chrono::duration<double>(duration).count();

    // Update stats timing
    if (m_stats.first_message_time == 0) {
        m_stats.first_message_time = msg.timestamp;
    }
    m_stats.last_message_time = msg.timestamp;

    if (type == SerialMessage::Type::Rx) {
        m_stats.bytes_received += content.size();
        m_stats.messages_received++;
    }

    m_messages.push_back(msg);

    // Limit message count
    while (m_messages.size() > MAX_MESSAGES) {
        m_messages.pop_front();
    }
}

std::string SerialMonitor::format_message(const SerialMessage& msg) const {
    std::ostringstream result;

    if (m_show_timestamp) {
        // Format timestamp as HH:MM:SS.mmm
        auto time_seconds = static_cast<time_t>(msg.timestamp);
        auto ms = static_cast<int>((msg.timestamp - time_seconds) * 1000);
        auto tm = *std::localtime(&time_seconds);

        result << std::setfill('0');
        result << std::setw(2) << tm.tm_hour << ":";
        result << std::setw(2) << tm.tm_min << ":";
        result << std::setw(2) << tm.tm_sec << ".";
        result << std::setw(3) << ms << " ";
    }

    // Convert to hex if in hex mode
    if (m_hex_mode && (msg.type == SerialMessage::Type::Rx || msg.type == SerialMessage::Type::Tx)) {
        result << string_to_hex(msg.content);
    } else {
        result << msg.content;
    }

    return result.str();
}

std::string SerialMonitor::string_to_hex(const std::string& input) const {
    std::ostringstream result;
    for (char c : input) {
        if (std::isprint(static_cast<unsigned char>(c))) {
            result << c;
        } else {
            result << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                   << static_cast<int>(static_cast<unsigned char>(c));
        }
    }
    return result.str();
}

std::string SerialMonitor::hex_to_string(const std::string& hex) const {
    std::string result;
    // Simple hex to string conversion (not fully implemented)
    // For production, would need proper hex parsing
    return hex;
}

} // namespace mechatron
