#pragma once

#include <string>
#include <functional>
#include <memory>
#include <glm/glm.hpp>

namespace mechatron {

/**
 * Cross-platform WebView wrapper
 * macOS: WKWebView
 * Windows: WebView2 (Edge Chromium)
 */
class WebView {
public:
    // Message callback from JavaScript
    using MessageCallback = std::function<void(const std::string&)>;

    WebView();
    ~WebView();

    // Initialize the webview (call after OpenGL context is ready)
    bool initialize();

    // Navigate to a URL or load HTML content
    void load_url(const std::string& url);
    void load_html(const std::string& html);

    // Execute JavaScript in the webview
    void execute_javascript(const std::string& js);

    // Set message callback (receive messages from JS via window.mechatron.sendMessage())
    void set_message_callback(MessageCallback callback);

    // Send message to JavaScript (received in JS via window.mechatron.onMessage)
    void send_message(const std::string& message);

    // Resize the webview
    void resize(int x, int y, int width, int height);

    // Show/hide the native webview. Hidden webviews keep their loaded page state.
    void set_visible(bool visible);

    // Render/update the webview (call every frame)
    void render();

    // Check if webview is ready
    bool is_ready() const { return m_ready; }

    // Get native handle (platform-specific)
    void* native_handle() { return m_native_handle; }

    // Attach to native window (must be called after initialize)
    void attach_to_window(void* window);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;

    bool m_ready = false;
    void* m_native_handle = nullptr;

    glm::ivec4 m_bounds{0, 0, 800, 600}; // x, y, width, height

    MessageCallback m_message_callback;
};

} // namespace mechatron
