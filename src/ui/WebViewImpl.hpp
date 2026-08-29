#ifndef WEBVIEW_IMPL_HPP
#define WEBVIEW_IMPL_HPP

#include "WebView.hpp"
#include <functional>

#if __APPLE__

#ifdef __OBJC__
@class WKWebView;
@class NSWindow;
@interface WebViewDelegate : NSObject <WKNavigationDelegate, WKScriptMessageHandler>
@property(nonatomic, assign) mechatron::WebView::MessageCallback messageCallback;
@end
#else
// Forward declarations for Objective-C types in C++ context
typedef struct objc_object WKWebView;
typedef struct objc_object NSWindow;
typedef struct objc_object WebViewDelegate;
#endif

class WebViewMacOS {
public:
    WebViewMacOS();
    ~WebViewMacOS();

    bool initialize();
    void load_url(const std::string& url);
    void load_html(const std::string& html);
    void execute_javascript(const std::string& js);
    void set_message_callback(mechatron::WebView::MessageCallback callback);
    void resize(int x, int y, int width, int height);
    void set_visible(bool visible);
    void render();

    // Add webview to NSWindow (call this after getting the window)
    void attach_to_window(void* glfwWindow);

    WKWebView* get_webview();

private:
    WebViewDelegate* m_delegate = nullptr;
    WKWebView* m_webview = nullptr;
    NSWindow* m_parent_window = nullptr;
};

#endif // __APPLE__

#if _WIN32
class WebViewWindows {
public:
    WebViewWindows();
    ~WebViewWindows();

    bool initialize();
    void load_url(const std::string& url);
    void load_html(const std::string& html);
    void execute_javascript(const std::string& js);
    void set_message_callback(mechatron::WebView::MessageCallback callback);
    void resize(int x, int y, int width, int height);
    void set_visible(bool visible);
    void render();
    void attach_to_window(void* window);

private:
    void* m_webview = nullptr;  // IWebView2WebView*
    void* m_controller = nullptr;  // IWebView2Controller*
    void* m_parent_window = nullptr;  // HWND
    std::string m_pending_html;
    mechatron::WebView::MessageCallback m_message_callback;
};

#endif // _WIN32

#endif // WEBVIEW_IMPL_HPP
