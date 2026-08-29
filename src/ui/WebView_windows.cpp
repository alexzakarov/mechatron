#if _WIN32
#include "WebViewImpl.hpp"
#include <spdlog/spdlog.h>
#include <windows.h>
#include <WebView2.h>
#include <wrl.h>
#include <string>

using namespace Microsoft::WRL;

WebViewWindows::WebViewWindows() : m_webview(nullptr), m_controller(nullptr) {}

WebViewWindows::~WebViewWindows() {
    if (m_controller) {
        auto controller = reinterpret_cast<IWebView2Controller*>(m_controller);
        controller->Close();
        m_controller = nullptr;
        m_webview = nullptr;
    }
}

bool WebViewWindows::initialize() {
    // Create WebView2 environment
    HWND parent = reinterpret_cast<HWND>(m_parent_window);
    if (!parent) {
        parent = GetActiveWindow();
    }

    HRESULT hr = CreateWebView2EnvironmentWithOptions(
        nullptr, nullptr, nullptr,
        Callback<IWebView2CreateWebView2EnvironmentCompletedHandler>(
            [this](HRESULT result, IWebView2Environment* env) -> HRESULT {
                if (SUCCEEDED(result)) {
                    // Create WebView2 controller
                    env->CreateWebViewController(
                        reinterpret_cast<HWND>(m_parent_window) ? reinterpret_cast<HWND>(m_parent_window) : GetActiveWindow(),
                        Callback<IWebView2CreateWebView2ControllerCompletedHandler>(
                            [this](HRESULT result, IWebView2Controller* controller) -> HRESULT {
                                if (SUCCEEDED(result)) {
                                    m_controller = controller;
                                    controller->get_WebView(
                                        reinterpret_cast<IWebView2WebView**>(&m_webview));

                                    auto webview = reinterpret_cast<IWebView2WebView*>(m_webview);

                                    // Add web message handler
                                    EventRegistrationToken token;
                                    webview->add_WebMessageReceived(
                                        Callback<ICoreWebView2WebMessageReceivedEventHandler>(
                                            [this](ICoreWebView2* sender,
                                                   ICoreWebView2WebMessageReceivedEventArgs* args) -> HRESULT {
                                                LPWSTR message = nullptr;
                                                args->TryGetWebMessageAsString(&message);
                                                if (message && m_message_callback) {
                                                    std::wstring wmsg(message);
                                                    std::string msg(wmsg.begin(), wmsg.end());
                                                    m_message_callback(msg);
                                                    CoTaskMemFree(message);
                                                }
                                                return S_OK;
                                            }).Get(),
                                        &token);

                                    // Inject initialization script
                                    std::wstring initScript = L"window.mechatron = {}; "
                                        L"window.mechatron.sendMessage = function(msg) { "
                                        L"  window.chrome.webview.postMessage(msg); "
                                        L"};";
                                    webview->AddScriptToExecuteOnDocumentCreated(
                                        initScript.c_str(), nullptr);

                                    if (!m_pending_html.empty()) {
                                        std::wstring whtml(m_pending_html.begin(), m_pending_html.end());
                                        webview->NavigateToString(whtml.c_str());
                                        m_pending_html.clear();
                                    }

                                    spdlog::debug("WebView2 initialized successfully");
                                }
                                return S_OK;
                            }).Get());
                }
                return S_OK;
            }).Get());

    return SUCCEEDED(hr);
}

void WebViewWindows::load_url(const std::string& url) {
    if (!m_webview) return;

    std::wstring wurl(url.begin(), url.end());
    auto webview = reinterpret_cast<IWebView2WebView*>(m_webview);
    webview->Navigate(wurl.c_str());
}

void WebViewWindows::load_html(const std::string& html) {
    if (!m_webview) {
        m_pending_html = html;
        return;
    }

    std::wstring whtml(html.begin(), html.end());
    auto webview = reinterpret_cast<IWebView2WebView*>(m_webview);
    webview->NavigateToString(whtml.c_str());
}

void WebViewWindows::execute_javascript(const std::string& js) {
    if (!m_webview) return;

    std::wstring wjs(js.begin(), js.end());
    auto webview = reinterpret_cast<IWebView2WebView*>(m_webview);
    webview->ExecuteScript(wjs.c_str(),
        Callback<ICoreWebView2ExecuteScriptCompletedHandler>(
            [](HRESULT result, LPCWSTR resultAsJson) -> HRESULT {
                if (FAILED(result)) {
                    spdlog::error("JavaScript execution failed");
                }
                return S_OK;
            }).Get());
}

void WebViewWindows::set_message_callback(mechatron::WebView::MessageCallback callback) {
    m_message_callback = callback;
}

void WebViewWindows::resize(int x, int y, int width, int height) {
    if (!m_controller) return;

    auto controller = reinterpret_cast<IWebView2Controller*>(m_controller);

    RECT bounds;
    bounds.left = x;
    bounds.top = y;
    bounds.right = x + width;
    bounds.bottom = y + height;

    controller->put_Bounds(bounds);
}

void WebViewWindows::set_visible(bool visible) {
    if (!m_controller) return;

    auto controller = reinterpret_cast<IWebView2Controller*>(m_controller);
    controller->put_IsVisible(visible ? TRUE : FALSE);
}

void WebViewWindows::render() {
    // WebView2 renders automatically
    // This is a no-op for Windows
}

void WebViewWindows::attach_to_window(void* window) {
    m_parent_window = window;
}

#endif // _WIN32
