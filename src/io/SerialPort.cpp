#include "SerialPort.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstring>
#include <filesystem>

namespace mechatron {

static std::string normalize_windows_com_port(const std::string& port) {
#if defined(_WIN32) || defined(_WIN64)
    // Accept "COM3", "\\\\.\\COM3", or "\\\\.\\COM12". Windows needs the "\\\\.\\"
    // prefix for COM10+.
    if (port.rfind("\\\\.\\", 0) == 0) return port;
    if (port.rfind("COM", 0) == 0 || port.rfind("com", 0) == 0) {
        return std::string("\\\\.\\") + port;
    }
    return port;
#else
    return port;
#endif
}

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
    close();
}

std::vector<SerialPortInfo> SerialPort::list_ports() {
    std::vector<SerialPortInfo> ports;

#if defined(_WIN32) || defined(_WIN64)
    for (int i = 1; i <= 256; ++i) {
        std::string name = "COM" + std::to_string(i);
        char target[256] = {};
        if (QueryDosDeviceA(name.c_str(), target, static_cast<DWORD>(sizeof(target))) == 0) {
            continue;
        }

        SerialPortInfo info;
        info.port = name;
        info.display_name = name;
        info.likely_arduino = true;
        ports.push_back(std::move(info));
    }
#else
    namespace fs = std::filesystem;
    const fs::path dev_dir("/dev");
    std::error_code ec;
    if (!fs::exists(dev_dir, ec)) {
        return ports;
    }

    for (const auto& entry : fs::directory_iterator(dev_dir, ec)) {
        if (ec) break;
        const std::string name = entry.path().filename().string();
        bool serial_candidate = false;
        bool likely_arduino = false;

#if defined(__APPLE__)
        serial_candidate =
            name.rfind("cu.", 0) == 0 &&
            (name.find("usbmodem") != std::string::npos ||
             name.find("usbserial") != std::string::npos ||
             name.find("wchusbserial") != std::string::npos ||
             name.find("Bluetooth-Incoming-Port") == std::string::npos);
        likely_arduino =
            name.find("usbmodem") != std::string::npos ||
            name.find("usbserial") != std::string::npos ||
            name.find("wchusbserial") != std::string::npos;
#else
        serial_candidate =
            name.rfind("ttyACM", 0) == 0 ||
            name.rfind("ttyUSB", 0) == 0 ||
            name.rfind("serial", 0) == 0;
        likely_arduino = name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0;
#endif

        if (!serial_candidate) {
            continue;
        }

        SerialPortInfo info;
        info.port = entry.path().string();
        info.display_name = info.port;
        info.likely_arduino = likely_arduino;
        ports.push_back(std::move(info));
    }
#endif

    std::sort(ports.begin(), ports.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        if (a.likely_arduino != b.likely_arduino) return a.likely_arduino > b.likely_arduino;
        return a.port < b.port;
    });
    return ports;
}

bool SerialPort::is_open() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_handle != INVALID_SERIAL_HANDLE;
}

bool SerialPort::open(const std::string& port, int baud_rate) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_handle != INVALID_SERIAL_HANDLE) {
        close_locked();
    }

    m_port = port;
    m_baud_rate = baud_rate;

#if defined(_WIN32) || defined(_WIN64)
    const std::string win_port = normalize_windows_com_port(m_port);

    m_handle = CreateFileA(
        win_port.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,                // exclusive access
        nullptr,
        OPEN_EXISTING,
        0,                // non-overlapped (timeouts provide non-blocking reads)
        nullptr);

    if (m_handle == INVALID_SERIAL_HANDLE) {
        spdlog::error("SerialPort: failed to open {} (GetLastError={})", win_port, GetLastError());
        return false;
    }

    // Reasonable driver buffers
    SetupComm(m_handle, 4096, 4096);
    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    if (!configure_locked()) {
        CloseHandle(m_handle);
        m_handle = INVALID_SERIAL_HANDLE;
        return false;
    }

    spdlog::info("SerialPort: opened {} @ {} baud", win_port, m_baud_rate);
    return true;
#else
    m_handle = ::open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_handle == INVALID_SERIAL_HANDLE) {
        spdlog::error("SerialPort: failed to open {}: {}", m_port, strerror(errno));
        return false;
    }

    if (!configure_locked()) {
        ::close(m_handle);
        m_handle = INVALID_SERIAL_HANDLE;
        return false;
    }

    spdlog::info("SerialPort: opened {} @ {} baud (fd={})", m_port, m_baud_rate, m_handle);
    return true;
#endif
}

void SerialPort::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    close_locked();
}

void SerialPort::close_locked() {
#if defined(_WIN32) || defined(_WIN64)
    if (m_handle != INVALID_SERIAL_HANDLE) {
        CloseHandle(m_handle);
        m_handle = INVALID_SERIAL_HANDLE;
        spdlog::info("SerialPort: closed {}", m_port);
    }
#else
    if (m_handle != INVALID_SERIAL_HANDLE) {
        ::close(m_handle);
        m_handle = INVALID_SERIAL_HANDLE;
        spdlog::info("SerialPort: closed {}", m_port);
    }
#endif
}

ssize_t SerialPort::read(void* buffer, size_t size) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_handle == INVALID_SERIAL_HANDLE) return -1;
#if defined(_WIN32) || defined(_WIN64)
    if (size == 0) return 0;

    DWORD bytes_read = 0;
    BOOL ok = ReadFile(m_handle, buffer, static_cast<DWORD>(size), &bytes_read, nullptr);
    if (!ok) {
        DWORD err = GetLastError();
        // With our timeout configuration, ReadFile should succeed and return 0 bytes when no data.
        spdlog::error("SerialPort: read failed (GetLastError={})", err);
        return -1;
    }
    return static_cast<ssize_t>(bytes_read);
#else
    ssize_t n = ::read(m_handle, buffer, size);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        spdlog::error("SerialPort: read failed: {}", strerror(errno));
    }
    return n;
#endif
}

ssize_t SerialPort::write(const void* data, size_t size) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_handle == INVALID_SERIAL_HANDLE) return -1;
#if defined(_WIN32) || defined(_WIN64)
    if (size == 0) return 0;

    DWORD bytes_written = 0;
    BOOL ok = WriteFile(m_handle, data, static_cast<DWORD>(size), &bytes_written, nullptr);
    if (!ok) {
        spdlog::error("SerialPort: write failed (GetLastError={})", GetLastError());
        return -1;
    }
    return static_cast<ssize_t>(bytes_written);
#else
    ssize_t n = ::write(m_handle, data, size);
    if (n < 0) {
        spdlog::error("SerialPort: write failed: {}", strerror(errno));
    }
    return n;
#endif
}

bool SerialPort::configure_locked() {
#if defined(_WIN32) || defined(_WIN64)
    if (m_handle == INVALID_SERIAL_HANDLE) return false;

    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(m_handle, &dcb)) {
        spdlog::error("SerialPort: GetCommState failed (GetLastError={})", GetLastError());
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(m_baud_rate);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    // Disable HW/SW flow control by default
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;

    if (!SetCommState(m_handle, &dcb)) {
        spdlog::error("SerialPort: SetCommState failed (GetLastError={})", GetLastError());
        return false;
    }

    // Non-blocking-ish reads: return immediately with 0 bytes if no data.
    COMMTIMEOUTS timeouts;
    SecureZeroMemory(&timeouts, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200;

    if (!SetCommTimeouts(m_handle, &timeouts)) {
        spdlog::error("SerialPort: SetCommTimeouts failed (GetLastError={})", GetLastError());
        return false;
    }

    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
#else
    if (m_handle == INVALID_SERIAL_HANDLE) return false;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(m_handle, &tty) != 0) {
        spdlog::error("SerialPort: tcgetattr failed: {}", strerror(errno));
        return false;
    }

    speed_t speed;
    switch (m_baud_rate) {
        case 300:    speed = B300;    break;
        case 1200:   speed = B1200;   break;
        case 2400:   speed = B2400;   break;
        case 4800:   speed = B4800;   break;
        case 9600:   speed = B9600;   break;
#ifdef B14400
        case 14400:  speed = B14400;  break;
#endif
        case 19200:  speed = B19200;  break;
#ifdef B28800
        case 28800:  speed = B28800;  break;
#endif
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
#ifdef B230400
        case 230400: speed = B230400; break;
#endif
#ifdef B460800
        case 460800: speed = B460800; break;
#endif
#ifdef B921600
        case 921600: speed = B921600; break;
#endif
        default:
            spdlog::warn("SerialPort: baud {} may not be supported; trying raw", m_baud_rate);
            speed = static_cast<speed_t>(m_baud_rate);
            break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_handle, TCSANOW, &tty) != 0) {
        spdlog::error("SerialPort: tcsetattr failed: {}", strerror(errno));
        return false;
    }

    tcflush(m_handle, TCIOFLUSH);
    return true;
#endif
}

} // namespace mechatron
