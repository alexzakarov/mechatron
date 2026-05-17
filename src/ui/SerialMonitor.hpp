#pragma once

#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <atomic>
#include <mutex>

#include "io/SerialPort.hpp"

namespace mechatron {

class SimulationOrchestrator;

class SerialMonitor {
public:
    SerialMonitor();
    ~SerialMonitor();  // Added for proper cleanup

    void render(SimulationOrchestrator& orchestrator);

    void clear();
    void send(const std::string& data);

    // Serial port configuration
    void set_baud_rate(int baud);
    int baud_rate() const { return m_baud_rate; }

    void set_port(const std::string& port);
    const std::string& port() const { return m_port; }

    bool is_connected() const { return m_connected; }

    // Configuration persistence
    void save_config();
    void load_config();
    static const char* config_file_path() { return "mechatron_serial_config.json"; }

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
    std::string m_port = "/dev/ttyUSB0";  // Changed default for Unix-like systems
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

    // Serial port handle (platform-specific)
    SerialPort m_serial;

    // Read thread for async serial data reception
    std::thread m_read_thread;
    std::atomic<bool> m_read_thread_running{false};
    std::mutex m_mutex;  // Protects m_messages and m_serial_handle

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

    // Platform-specific serial port methods
    void read_thread_func();

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
