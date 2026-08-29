#include "WebView.hpp"
#include <spdlog/spdlog.h>

// Platform-specific implementations
#ifdef __APPLE__
#include "WebViewImpl.hpp"
#endif

#ifdef _WIN32
#include "WebViewImpl.hpp"
#endif

namespace mechatron {

class WebView::Impl {
public:
    Impl() = default;
    ~Impl() = default;

    // Platform-specific implementation
#ifdef __APPLE__
    WebViewMacOS* native = nullptr;
#endif

#ifdef _WIN32
    WebViewWindows* native = nullptr;
#endif
};

WebView::WebView() : m_impl(std::make_unique<Impl>()) {}

WebView::~WebView() {
#ifdef __APPLE__
    if (m_impl->native) {
        delete m_impl->native;
        m_impl->native = nullptr;
    }
#endif

#ifdef _WIN32
    if (m_impl->native) {
        delete m_impl->native;
        m_impl->native = nullptr;
    }
#endif
}

bool WebView::initialize() {
#ifdef __APPLE__
    m_impl->native = new WebViewMacOS();
    if (!m_impl->native->initialize()) {
        spdlog::error("Failed to initialize WKWebView");
        return false;
    }
    m_impl->native->set_message_callback(m_message_callback);
    m_native_handle = m_impl->native;
    m_ready = true;
    return true;
#endif

#ifdef _WIN32
    m_impl->native = new WebViewWindows();
    if (!m_impl->native->initialize()) {
        spdlog::error("Failed to initialize WebView2");
        return false;
    }
    m_impl->native->set_message_callback(m_message_callback);
    m_native_handle = m_impl->native;
    m_ready = true;
    return true;
#endif

    spdlog::error("WebView not supported on this platform");
    return false;
}

void WebView::load_url(const std::string& url) {
    if (!m_ready || !m_impl->native) {
        spdlog::warn("WebView not ready, cannot load URL");
        return;
    }

#ifdef __APPLE__
    m_impl->native->load_url(url);
#endif

#ifdef _WIN32
    m_impl->native->load_url(url);
#endif
}

void WebView::load_html(const std::string& html) {
    if (!m_ready || !m_impl->native) {
        spdlog::warn("WebView not ready, cannot load HTML");
        return;
    }

#ifdef __APPLE__
    m_impl->native->load_html(html);
#endif

#ifdef _WIN32
    m_impl->native->load_html(html);
#endif
}

void WebView::execute_javascript(const std::string& js) {
    if (!m_ready || !m_impl->native) {
        spdlog::warn("WebView not ready, cannot execute JavaScript");
        return;
    }

#ifdef __APPLE__
    m_impl->native->execute_javascript(js);
#endif

#ifdef _WIN32
    m_impl->native->execute_javascript(js);
#endif
}

void WebView::set_message_callback(MessageCallback callback) {
    m_message_callback = std::move(callback);

    if (m_ready && m_impl->native) {
#ifdef __APPLE__
        m_impl->native->set_message_callback(m_message_callback);
#endif

#ifdef _WIN32
        m_impl->native->set_message_callback(m_message_callback);
#endif
    }
}

void WebView::send_message(const std::string& message) {
    if (!m_ready || !m_impl->native) {
        spdlog::warn("WebView not ready, cannot send message");
        return;
    }

    // Properly escape the JSON string for JavaScript
    std::string escaped;
    for (char c : message) {
        switch (c) {
            case '\\': escaped += "\\\\"; break;
            case '"':  escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += c; break;
        }
    }

    std::string js = "if (window.mechatron && window.mechatron.onMessage) { "
                    "window.mechatron.onMessage(\"" + escaped + "\"); }";

#ifdef __APPLE__
    m_impl->native->execute_javascript(js);
#endif

#ifdef _WIN32
    m_impl->native->execute_javascript(js);
#endif
}

void WebView::resize(int x, int y, int width, int height) {
    m_bounds = {x, y, width, height};

    if (!m_ready || !m_impl->native) {
        return;
    }

#ifdef __APPLE__
    m_impl->native->resize(x, y, width, height);
#endif

#ifdef _WIN32
    m_impl->native->resize(x, y, width, height);
#endif
}

void WebView::set_visible(bool visible) {
    if (!m_ready || !m_impl->native) {
        return;
    }

#ifdef __APPLE__
    m_impl->native->set_visible(visible);
#endif

#ifdef _WIN32
    m_impl->native->set_visible(visible);
#endif
}

void WebView::render() {
    if (!m_ready || !m_impl->native) {
        return;
    }

#ifdef __APPLE__
    m_impl->native->render();
#endif

#ifdef _WIN32
    m_impl->native->render();
#endif
}

void WebView::attach_to_window(void* window) {
#ifdef __APPLE__
    if (m_impl->native) {
        m_impl->native->attach_to_window(window);
    }
#endif

#ifdef _WIN32
    if (m_impl->native) {
        m_impl->native->attach_to_window(window);
    }
#endif
}

} // namespace mechatron
