#pragma once

#include <string>
#include <vector>
#include <deque>

namespace mechatron {

class SimulationOrchestrator;

class SerialMonitor {
public:
    SerialMonitor();
    void render(SimulationOrchestrator& orchestrator);

    void clear();
    void send(const std::string& data);

    // Serial port configuration
    void set_baud_rate(int baud) { m_baud_rate = baud; }
    int baud_rate() const { return m_baud_rate; }

    void set_port(const std::string& port) { m_port = port; }
    const std::string& port() const { return m_port; }

    bool is_connected() const { return m_connected; }

private:
    struct SerialMessage {
        enum class Type { Rx, Tx, Info, Error };
        Type type;
        std::string content;
        double timestamp;  // seconds since epoch
    };

    std::deque<SerialMessage> m_messages;
    static constexpr size_t MAX_MESSAGES = 1000;

    // Configuration
    std::string m_port = "COM3";
    int m_baud_rate = 9600;
    bool m_connected = false;
    bool m_auto_scroll = true;
    bool m_show_timestamp = true;
    bool m_hex_mode = false;

    // Input buffer
    char m_input_buffer[256] = "";
    bool m_append_newline = true;
    bool m_append_cr = true;

    // UI state
    bool m_show_settings = true;
    bool m_show_stats = false;

    void render_menu_bar();
    void render_output_area();
    void render_input_area();
    void render_settings_panel();
    void render_stats();

    void connect();
    void disconnect();
    void add_message(SerialMessage::Type type, const std::string& content);

    std::string format_message(const SerialMessage& msg) const;
    std::string string_to_hex(const std::string& input) const;
    std::string hex_to_string(const std::string& hex) const;

    // Statistics
    struct Stats {
        size_t bytes_sent = 0;
        size_t bytes_received = 0;
        size_t messages_sent = 0;
        size_t messages_received = 0;
        double first_message_time = 0;
        double last_message_time = 0;
    } m_stats;
};

} // namespace mechatron
