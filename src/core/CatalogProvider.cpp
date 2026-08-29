#include "CatalogProvider.hpp"
#include "AppConfig.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <string_view>
#include <spdlog/spdlog.h>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace mechatron {

namespace {

constexpr auto kBackendCatalogRetryCooldown = std::chrono::seconds(2);
constexpr auto kBackendCatalogSuccessTtl = std::chrono::seconds(5);

struct ParsedUrl {
    std::string host;
    std::string port;
    std::string path;
};

std::optional<ParsedUrl> parse_http_url(std::string url) {
    if (url.empty()) {
        return std::nullopt;
    }

    constexpr std::string_view prefix = "http://";
    if (url.rfind(prefix.data(), 0) != 0) {
        spdlog::warn("[CatalogProvider] unsupported backend URL scheme: {}", url);
        return std::nullopt;
    }

    url.erase(0, prefix.size());
    ParsedUrl parsed;
    parsed.path = "/";

    const auto slash = url.find('/');
    std::string host_port = slash == std::string::npos ? url : url.substr(0, slash);
    if (slash != std::string::npos) {
        parsed.path = url.substr(slash);
    }

    const auto colon = host_port.rfind(':');
    if (colon == std::string::npos) {
        parsed.host = host_port;
        parsed.port = "80";
    } else {
        parsed.host = host_port.substr(0, colon);
        parsed.port = host_port.substr(colon + 1);
    }

    if (parsed.host.empty() || parsed.port.empty()) {
        return std::nullopt;
    }
    return parsed;
}

class SocketHandle {
public:
#if defined(_WIN32)
    using native_type = SOCKET;
    static constexpr native_type invalid = INVALID_SOCKET;
#else
    using native_type = int;
    static constexpr native_type invalid = -1;
#endif

    SocketHandle() = default;
    explicit SocketHandle(native_type value) : m_value(value) {}
    ~SocketHandle() { reset(); }

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;

    SocketHandle(SocketHandle&& other) noexcept : m_value(other.m_value) {
        other.m_value = invalid;
    }
    SocketHandle& operator=(SocketHandle&& other) noexcept {
        if (this != &other) {
            reset();
            m_value = other.m_value;
            other.m_value = invalid;
        }
        return *this;
    }

    native_type get() const { return m_value; }
    explicit operator bool() const { return m_value != invalid; }

    void reset() {
        if (m_value == invalid) return;
#if defined(_WIN32)
        closesocket(m_value);
#else
        close(m_value);
#endif
        m_value = invalid;
    }

private:
    native_type m_value = invalid;
};

std::optional<std::string> http_get_text(const std::string& url, std::string* error_out = nullptr) {
    auto set_error = [&](std::string message) {
        if (error_out) {
            *error_out = std::move(message);
        }
    };
    const auto parsed = parse_http_url(url);
    if (!parsed) {
        set_error("invalid backend URL");
        return std::nullopt;
    }

#if defined(_WIN32)
    WSADATA wsa_data{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        spdlog::warn("[CatalogProvider] WSAStartup failed");
        return std::nullopt;
    }
    struct WsaGuard {
        ~WsaGuard() { WSACleanup(); }
    } wsa_guard;
#endif

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo* results = nullptr;
    if (getaddrinfo(parsed->host.c_str(), parsed->port.c_str(), &hints, &results) != 0) {
        spdlog::warn("[CatalogProvider] getaddrinfo failed for {}:{}", parsed->host, parsed->port);
        set_error("DNS/addr lookup failed");
        return std::nullopt;
    }

    struct AddrInfoGuard {
        addrinfo* ptr = nullptr;
        ~AddrInfoGuard() { if (ptr) freeaddrinfo(ptr); }
    } guard{results};

    SocketHandle socket;
    for (addrinfo* it = results; it != nullptr; it = it->ai_next) {
        SocketHandle candidate(::socket(it->ai_family, it->ai_socktype, it->ai_protocol));
        if (!candidate) {
            continue;
        }
        if (::connect(candidate.get(), it->ai_addr, static_cast<int>(it->ai_addrlen)) == 0) {
            socket = std::move(candidate);
            break;
        }
    }

    if (!socket) {
        spdlog::warn("[CatalogProvider] could not connect to backend {}", url);
        set_error("connection failed");
        return std::nullopt;
    }

    const std::string request =
        "GET " + parsed->path + " HTTP/1.1\r\n"
        "Host: " + parsed->host + "\r\n"
        "Connection: close\r\n"
        "Accept: application/json\r\n\r\n";

    size_t sent_total = 0;
    while (sent_total < request.size()) {
#if defined(_WIN32)
        const int sent = ::send(socket.get(),
                                request.c_str() + sent_total,
                                static_cast<int>(request.size() - sent_total),
                                0);
        if (sent == SOCKET_ERROR || sent <= 0) {
#else
        const ssize_t sent = ::send(socket.get(),
                                    request.c_str() + sent_total,
                                    request.size() - sent_total,
                                    0);
        if (sent <= 0) {
#endif
            spdlog::warn("[CatalogProvider] failed to send full request to backend {} (sent {} of {} bytes)",
                         url, sent_total, request.size());
            set_error("request send failed");
            return std::nullopt;
        }
        sent_total += static_cast<size_t>(sent);
    }

    std::string response;
    char buffer[4096];
    while (true) {
#if defined(_WIN32)
        const int received = ::recv(socket.get(), buffer, sizeof(buffer), 0);
        if (received == SOCKET_ERROR) {
#else
        const ssize_t received = ::recv(socket.get(), buffer, sizeof(buffer), 0);
        if (received < 0) {
#endif
            spdlog::warn("[CatalogProvider] failed reading response from backend {}", url);
            set_error("socket read failed");
            return std::nullopt;
        }
        if (received == 0) break;
        response.append(buffer, buffer + received);
    }

    if (response.empty()) {
        spdlog::warn("[CatalogProvider] empty HTTP response from backend {}", url);
        set_error("empty HTTP response");
        return std::nullopt;
    }

    size_t header_end = response.find("\r\n\r\n");
    size_t delimiter_size = 4;
    if (header_end == std::string::npos) {
        header_end = response.find("\n\n");
        delimiter_size = 2;
    }
    if (header_end == std::string::npos) {
        const std::string preview = response.substr(0, std::min<size_t>(response.size(), 200));
        spdlog::warn("[CatalogProvider] malformed HTTP response from backend {}: {}", url, preview);
        set_error("malformed HTTP response");
        return std::nullopt;
    }

    const std::string headers = response.substr(0, header_end);
    const auto status_line_end = headers.find('\n');
    const std::string status_line = headers.substr(0, status_line_end);
    if (status_line.find("HTTP/") != 0) {
        spdlog::warn("[CatalogProvider] invalid HTTP status line from backend {}: {}", url, status_line);
        set_error("invalid HTTP status line");
        return std::nullopt;
    }
    if (status_line.find(" 200 ") == std::string::npos && status_line.find(" 200\r") == std::string::npos) {
        spdlog::warn("[CatalogProvider] backend returned non-200 response for {}: {}", url, status_line);
        set_error("non-200 response");
        return std::nullopt;
    }

    std::string body = response.substr(header_end + delimiter_size);

    // Gin streams large JSON responses with "Transfer-Encoding: chunked".
    // The body then arrives as "<hex-size>\r\n<data>\r\n<hex-size>\r\n<data>\r\n...0\r\n\r\n",
    // which is NOT valid JSON. Detect the header and decode the chunked body
    // before handing it to nlohmann::json::parse. Otherwise the parser fails
    // with "invalid literal" on the first hex size token (e.g. "f7d").
    std::string lower_headers = headers;
    std::transform(lower_headers.begin(), lower_headers.end(), lower_headers.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    const auto fold_pos = lower_headers.find("transfer-encoding:");
    if (fold_pos != std::string::npos) {
        const auto value_start = lower_headers.find_first_not_of(" \t", fold_pos + 18);
        const auto value_end = lower_headers.find_first_of("\r\n", value_start);
        const std::string value = (value_start != std::string::npos)
            ? lower_headers.substr(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start)
            : std::string{};
        if (value.find("chunked") != std::string::npos) {
            std::string decoded;
            decoded.reserve(body.size());
            size_t pos = 0;
            while (pos < body.size()) {
                const size_t eol = body.find("\r\n", pos);
                if (eol == std::string::npos) break;
                const std::string size_token = body.substr(pos, eol - pos);
                const char* size_ptr = size_token.c_str();
                char* end_ptr = nullptr;
                const unsigned long long chunk_size = std::strtoull(size_ptr, &end_ptr, 16);
                if (end_ptr == size_ptr || chunk_size == 0) break;
                const size_t data_start = eol + 2;
                if (data_start + chunk_size > body.size()) break;
                decoded.append(body, data_start, static_cast<size_t>(chunk_size));
                pos = data_start + static_cast<size_t>(chunk_size);
                if (body.compare(pos, 2, "\r\n") == 0) pos += 2;
            }
            body = std::move(decoded);
        }
    }

    return body;
}

std::string default_backend_catalog_url() {
    if (const char* env = std::getenv(app_config::kBackendUrlEnv)) {
        std::string base = env;
        while (!base.empty() && base.back() == '/') base.pop_back();
        return base + app_config::kBackendCatalogPath;
    }
    std::string url = "http://";
    url += app_config::kBackendHost;
    url += ":";
    url += std::to_string(app_config::kBackendPort);
    url += app_config::kBackendCatalogPath;
    return url;
}

} // namespace

LocalManifestCatalogProvider::LocalManifestCatalogProvider(std::optional<std::filesystem::path> manifest_path)
    : m_manifest_path(std::move(manifest_path)) {}

std::optional<std::filesystem::path> LocalManifestCatalogProvider::find_default_catalog_path() {
    namespace fs = std::filesystem;
    std::vector<fs::path> roots;
    roots.push_back(fs::current_path());
    roots.push_back(fs::path(__FILE__).parent_path());

    for (auto root : roots) {
        for (int depth = 0; depth < 8 && !root.empty(); ++depth) {
            auto candidate = root / "backend" / "catalog" / "component_catalog.json";
            if (fs::exists(candidate)) {
                return candidate;
            }
            root = root.parent_path();
        }
    }
    return std::nullopt;
}

std::optional<nlohmann::json> LocalManifestCatalogProvider::load_catalog() const {
    const auto path = m_manifest_path ? m_manifest_path : find_default_catalog_path();
    if (!path) {
        spdlog::warn("[CatalogProvider] component catalog path not found");
        return std::nullopt;
    }

    std::optional<std::filesystem::file_time_type> mtime;
    std::error_code ec;
    if (std::filesystem::exists(*path, ec)) {
        mtime = std::filesystem::last_write_time(*path, ec);
        if (ec) mtime.reset();
    }

    {
        std::lock_guard lock(m_cache_mutex);
        if (m_cached_catalog && m_cached_path == *path && m_cached_mtime == mtime) {
            return m_cached_catalog;
        }
    }

    try {
        std::ifstream input(*path);
        auto parsed = nlohmann::json::parse(input);
        {
            std::lock_guard lock(m_cache_mutex);
            m_cached_path = *path;
            m_cached_mtime = mtime;
            m_cached_catalog = parsed;
        }
        return parsed;
    } catch (const std::exception& e) {
        spdlog::warn("[CatalogProvider] failed to load catalog from {}: {}", path->string(), e.what());
    }
    return std::nullopt;
}

BackendApiCatalogProvider::BackendApiCatalogProvider(std::string base_url)
    : m_base_url(base_url.empty() ? default_backend_catalog_url() : std::move(base_url)) {}

std::optional<nlohmann::json> BackendApiCatalogProvider::load_catalog() const {
    {
        std::lock_guard lock(m_cache_mutex);
        if (m_cached_catalog && m_last_success_at) {
            const auto now = std::chrono::steady_clock::now();
            if ((now - *m_last_success_at) < kBackendCatalogSuccessTtl) {
                return m_cached_catalog;
            }
        }
        if (!m_cached_catalog && m_last_failure_at) {
            const auto now = std::chrono::steady_clock::now();
            if ((now - *m_last_failure_at) < kBackendCatalogRetryCooldown) {
                return std::nullopt;
            }
        }
    }

    std::string error_reason;
    if (const auto body = http_get_text(m_base_url, &error_reason)) {
        try {
            auto parsed = nlohmann::json::parse(*body);
            std::lock_guard lock(m_cache_mutex);
            m_cached_catalog = parsed;
            m_last_success_at = std::chrono::steady_clock::now();
            m_last_failure_at.reset();
            m_last_failure_reason.clear();
            return parsed;
        } catch (const std::exception& e) {
            spdlog::warn("[CatalogProvider] failed to parse backend catalog from {}: {}", m_base_url, e.what());
            std::lock_guard lock(m_cache_mutex);
            m_last_success_at.reset();
            m_last_failure_at = std::chrono::steady_clock::now();
            m_last_failure_reason = "catalog JSON parse failed";
            return std::nullopt;
        }
    }

    std::lock_guard lock(m_cache_mutex);
    m_cached_catalog.reset();
    const auto now = std::chrono::steady_clock::now();
    if (!m_last_failure_at || m_last_failure_reason != error_reason) {
        spdlog::warn("[CatalogProvider] backend catalog unavailable at {} ({})", m_base_url, error_reason);
    }
    m_last_failure_at = now;
    m_last_failure_reason = error_reason;
    return std::nullopt;
}

const CatalogProvider& default_catalog_provider() {
    static const BackendApiCatalogProvider provider;
    return provider;
}

} // namespace mechatron
