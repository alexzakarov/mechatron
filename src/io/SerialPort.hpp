#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>

// Platform-specific serial port headers
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #include <BaseTsd.h>
    typedef HANDLE SerialHandle;
    #define INVALID_SERIAL_HANDLE INVALID_HANDLE_VALUE
    typedef SSIZE_T ssize_t;
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #ifdef __APPLE__
        #include <IOKit/serial/ioss.h>
    #endif
    typedef int SerialHandle;
    #define INVALID_SERIAL_HANDLE -1
#endif

namespace mechatron {

struct SerialPortInfo {
    std::string port;
    std::string display_name;
    bool likely_arduino = false;
};

// Thin, reusable serial port wrapper (UI-independent).
// Thread-safe for concurrent read/write via an internal mutex.
class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port, int baud_rate);
    void close();
    bool is_open() const;

    std::string port() const { return m_port; }
    int baud_rate() const { return m_baud_rate; }

    // Non-blocking friendly read/write. Returns bytes read/written, or -1 on error.
    ssize_t read(void* buffer, size_t size);
    ssize_t write(const void* data, size_t size);

    static std::vector<SerialPortInfo> list_ports();

private:
    bool configure_locked();
    void close_locked();

    mutable std::mutex m_mutex;
    SerialHandle m_handle = INVALID_SERIAL_HANDLE;
    std::string m_port;
    int m_baud_rate = 9600;
};

} // namespace mechatron
