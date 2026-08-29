#include "SerialMonitor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include <imgui.h>
#include "Theme.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <ctime>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>

namespace mechatron {

SerialMonitor::SerialMonitor() {
    // Load saved configuration
    load_config();

    // Add welcome message
    add_message(SerialMessage::Type::Info, "Serial Monitor initialized");
    add_message(SerialMessage::Type::Info, "Connect to a port to begin communication");
}

SerialMonitor::~SerialMonitor() {
    // Save configuration before exit
    save_config();

    // Ensure disconnect on destruction
    if (m_connected) {
        disconnect();
    }
}

void SerialMonitor::save_config() {
    nlohmann::json config;
    config["port"] = m_port;
    config["baud_rate"] = m_baud_rate;

    try {
        std::ofstream out(config_file_path());
        out << config.dump(2);
        spdlog::debug("[SerialMonitor] Configuration saved to {}", config_file_path());
    } catch (const std::exception& e) {
        spdlog::warn("[SerialMonitor] Failed to save configuration: {}", e.what());
    }
}

void SerialMonitor::load_config() {
    try {
        std::ifstream in(config_file_path());
        if (!in.is_open()) {
            spdlog::debug("[SerialMonitor] No configuration file found, using defaults");
            return;
        }

        nlohmann::json config;
        in >> config;

        if (config.contains("port")) m_port = config["port"];
        if (config.contains("baud_rate")) m_baud_rate = config["baud_rate"];

        spdlog::info("[SerialMonitor] Configuration loaded: port={}, baud={}", m_port, m_baud_rate);
    } catch (const std::exception& e) {
        spdlog::warn("[SerialMonitor] Failed to load configuration: {}", e.what());
    }
}

void SerialMonitor::set_baud_rate(int baud) {
    m_baud_rate = baud;
    save_config();  // Save configuration when changed
}

void SerialMonitor::set_port(const std::string& port) {
    m_port = port;
    save_config();  // Save configuration when changed
}

void SerialMonitor::render(SimulationOrchestrator& orchestrator) {
    // No Begin/End - we're inside a tab

    render_menu_bar();

    // Connection status bar
    if (m_connected) {
        ImGui::TextColored(Theme::CurrentPalette().success, "[CONNECTED] %s @ %d baud", m_port.c_str(), m_baud_rate);
    } else {
        ImGui::TextColored(Theme::CurrentPalette().error, "[DISCONNECTED]");
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
}

void SerialMonitor::render_menu_bar() {
    // Toolbar buttons instead of menu bar (we're inside a tab)
    if (ImGui::Button("Clear##Serial")) {
        clear();
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Log##Serial")) {
        spdlog::info("Save log (not implemented)");
    }
    ImGui::SameLine();
    ImGui::Checkbox("Settings##Serial", &m_show_settings);
    ImGui::SameLine();
    ImGui::Checkbox("Stats##Serial", &m_show_stats);
    ImGui::SameLine();
    if (ImGui::RadioButton("ASCII##Serial", !m_hex_mode)) {
        m_hex_mode = false;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("HEX##Serial", m_hex_mode)) {
        m_hex_mode = true;
    }
}

void SerialMonitor::render_output_area() {
    // No BeginChild - we're inside a tab, scroll is handled by parent

    // Display messages
    for (const auto& msg : m_messages) {
        // Set color based on message type
        ImU32 color = Theme::U32(Theme::CurrentPalette().text);
        const char* prefix = "";

        switch (msg.type) {
            case SerialMessage::Type::Rx:
                color = Theme::U32(Theme::CurrentPalette().primary);
                prefix = "[RX] ";
                break;
            case SerialMessage::Type::Tx:
                color = Theme::U32(Theme::CurrentPalette().warning);
                prefix = "[TX] ";
                break;
            case SerialMessage::Type::Info:
                color = Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().textDim, 0.78f));
                prefix = "[INFO] ";
                break;
            case SerialMessage::Type::Error:
                color = Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 1.0f));
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
        const char* baud_rates[] = {"300", "1200", "2400", "4800", "9600", "14400", "19200",
                                     "28800", "38400", "57600", "115200", "230400", "460800", "921600"};
        if (ImGui::Combo("##Baud", &baud_preset, baud_rates, 14)) {
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

    // Send data via serial port
    ssize_t bytes_written = m_serial.write(to_send.c_str(), to_send.size());

    if (bytes_written < 0) {
        add_message(SerialMessage::Type::Error, "Failed to send data");
        return;
    }

    // Add to message log
    add_message(SerialMessage::Type::Tx, data);

    // Update stats
    m_stats.bytes_sent += bytes_written;
    m_stats.messages_sent++;

    spdlog::debug("Serial TX: {} ({} bytes)", data, bytes_written);
}

void SerialMonitor::connect() {
    if (m_connected) {
        add_message(SerialMessage::Type::Info, "Already connected to " + m_port);
        return;
    }

    if (m_port.empty()) {
        add_message(SerialMessage::Type::Error, "No port specified");
        return;
    }

    // Open and configure serial port
    if (!m_serial.open(m_port, m_baud_rate)) {
        add_message(SerialMessage::Type::Error, "Failed to open port: " + m_port);
        return;
    }

    // Start read thread
    m_read_thread_running = true;
    m_read_thread = std::thread(&SerialMonitor::read_thread_func, this);

    m_connected = true;
    add_message(SerialMessage::Type::Info, "Connected to " + m_port);
    spdlog::info("Serial connected: {} @ {} baud", m_port, m_baud_rate);
}

void SerialMonitor::disconnect() {
    if (!m_connected) {
        return;
    }

    // Stop read thread
    m_read_thread_running = false;

    // Wait for thread to finish (with timeout)
    if (m_read_thread.joinable()) {
        m_read_thread.join();
    }

    // Close serial port
    m_serial.close();

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

    // Skip 0x prefix if present
    size_t start = 0;
    if (hex.size() >= 2 && hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) {
        start = 2;
    }

    // Parse hex characters in pairs
    for (size_t i = start; i < hex.size(); ) {
        // Skip whitespace and separators
        if (std::isspace(static_cast<unsigned char>(hex[i])) || hex[i] == ',' || hex[i] == ':' || hex[i] == '-') {
            ++i;
            continue;
        }

        // Parse two hex digits
        int byte = 0;
        int digits = 0;

        while (digits < 2 && i < hex.size()) {
            char c = hex[i];
            int value = 0;

            if (c >= '0' && c <= '9') {
                value = c - '0';
            } else if (c >= 'a' && c <= 'f') {
                value = 10 + (c - 'a');
            } else if (c >= 'A' && c <= 'F') {
                value = 10 + (c - 'A');
            } else if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == ':' || c == '-') {
                ++i;
                continue;  // Skip separators between digits
            } else {
                break;  // Invalid character
            }

            byte = (byte << 4) | value;
            ++digits;
            ++i;
        }

        if (digits > 0) {
            result.push_back(static_cast<char>(byte & 0xFF));
        }

        // Skip trailing whitespace
        while (i < hex.size() && std::isspace(static_cast<unsigned char>(hex[i]))) {
            ++i;
        }
    }

    return result;
}

void SerialMonitor::read_thread_func() {
    spdlog::info("Serial read thread started");

    char buffer[1024];
    std::string line_buffer;

    while (m_read_thread_running) {
        if (!m_serial.is_open()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Read data from serial port
        ssize_t bytes_read = m_serial.read(buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';

            // Process received data
            for (ssize_t i = 0; i < bytes_read; i++) {
                char c = buffer[i];

                // Build line buffer
                if (c == '\n' || c == '\r') {
                    if (!line_buffer.empty()) {
                        // Add complete line to messages
                        std::lock_guard<std::mutex> lock(m_mutex);
                        add_message(SerialMessage::Type::Rx, line_buffer);
                        line_buffer.clear();

                        // Update stats
                        m_stats.bytes_received += bytes_read;
                        m_stats.messages_received++;
                    }
                } else {
                    line_buffer += c;
                }
            }

            // Log raw bytes if not newline
            if (bytes_read > 0 && line_buffer.empty() &&
                std::string(buffer, bytes_read).find_first_of("\n\r") == std::string::npos) {
                std::string data(buffer, bytes_read);
                std::lock_guard<std::mutex> lock(m_mutex);
                add_message(SerialMessage::Type::Rx, data);
            }
        }

        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    spdlog::info("Serial read thread stopped");
}

} // namespace mechatron
