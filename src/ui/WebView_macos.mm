#if __APPLE__
#import <Foundation/Foundation.h>
#import <WebKit/WebKit.h>
#import <Cocoa/Cocoa.h>
#import <OpenGL/gl.h>
#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include "WebViewImpl.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <iostream>

@implementation WebViewDelegate

- (void)userContentController:(WKUserContentController*)userContentController
      didReceiveScriptMessage:(WKScriptMessage*)message {
    NSString* bodyStr = [message.body isKindOfClass:[NSString class]] ? (NSString*)message.body : @"";
    std::string msg([bodyStr UTF8String]);

    // Log console messages
    if (msg.find("CONSOLE:") == 0) {
        spdlog::info("[WebView Console] {}", msg.substr(8));
    }

    if (self.messageCallback) {
        self.messageCallback(msg);
    }
}

- (void)webView:(WKWebView*)webView
    didFinishNavigation:(WKNavigation*)navigation {
    spdlog::debug("WebView navigation completed");
}

- (void)webView:(WKWebView*)webView
    didFailNavigation:(WKNavigation*)navigation
            withError:(NSError*)error {
    spdlog::error("WebView navigation failed: {}", [error.localizedDescription UTF8String]);
}

- (void)webView:(WKWebView*)webView
    didFailProvisionalNavigation:(WKNavigation*)navigation
                    withError:(NSError*)error {
    spdlog::error("WebView provisional navigation failed: {}", [error.localizedDescription UTF8String]);
}
@end

WebViewMacOS::WebViewMacOS() : m_delegate(nil), m_webview(nil) {}

WebViewMacOS::~WebViewMacOS() {
    // With ARC, objects are automatically managed
    // Just set to nil
    m_webview = nil;
    m_delegate = nil;
}

bool WebViewMacOS::initialize() {
    // Create WKWebView configuration
    WKWebViewConfiguration* config = [[WKWebViewConfiguration alloc] init];
    if (!config) {
        return false;
    }

    // Enable developer extras for console.log access
    [config.preferences setValue:@YES forKey:@"developerExtrasEnabled"];

    // Enable JavaScript and console logging
    [[config preferences] setValue:@YES forKey:@"javaScriptEnabled"];
    [[config preferences] setValue:@YES forKey:@"javaScriptCanOpenWindowsAutomatically"];

    // Add script message handler for mechatron
    m_delegate = [[WebViewDelegate alloc] init];
    [config.userContentController addScriptMessageHandler:m_delegate name:@"mechatron"];

    // Create WKWebView with initial frame
    NSRect frame = NSMakeRect(0, 0, 800, 600);
    m_webview = [[WKWebView alloc] initWithFrame:frame configuration:config];
    if (!m_webview) {
        return false;
    }

    [m_webview setNavigationDelegate:m_delegate];
    [m_webview setAutoresizingMask:NSViewWidthSizable | NSViewHeightSizable];

    // Make the webview visible
    [m_webview setHidden:NO];
    [m_webview setAlphaValue:1.0];

    spdlog::debug("WKWebView initialized successfully");
    return true;
}

void WebViewMacOS::load_url(const std::string& url) {
    if (!m_webview) return;

    NSString* urlStr = [NSString stringWithUTF8String:url.c_str()];
    NSURL* nsUrl = [NSURL URLWithString:urlStr];
    NSURLRequest* request = [NSURLRequest requestWithURL:nsUrl];
    [m_webview loadRequest:request];
}

void WebViewMacOS::load_html(const std::string& html) {
    if (!m_webview) return;

    NSString* htmlStr = [NSString stringWithUTF8String:html.c_str()];

    // Load with base URL for better resource loading
    NSURL* baseURL = [NSURL fileURLWithPath:NSFileManager.defaultManager.currentDirectoryPath];

    spdlog::debug("Loading HTML content, size: {} bytes", html.length());

    [m_webview loadHTMLString:htmlStr baseURL:baseURL];

    // Force the webview to display
    [m_webview setNeedsDisplay:YES];
    [m_webview displayIfNeeded];
}

void WebViewMacOS::execute_javascript(const std::string& js) {
    if (!m_webview) return;

    // Log what we're about to execute for debugging
    if (!js.empty() && js.length() < 300) {
        spdlog::debug("Executing JS: {}", js);
    }

    NSString* jsStr = [NSString stringWithUTF8String:js.c_str()];
    [m_webview evaluateJavaScript:jsStr
             completionHandler:^(id result, NSError* error) {
        if (error) {
            NSString* errorDesc = [error localizedDescription];
            std::string errorStr = [errorDesc UTF8String];

            // Log the error with context
            spdlog::error("JavaScript execution failed: {}", errorStr);

            // Try to get more error info
            NSDictionary* userInfo = [error userInfo];
            if (userInfo) {
                NSString* jsStack = userInfo[@"JSStackTrace"];
                if (jsStack) {
                    spdlog::error("JS Stack: {}", [jsStack UTF8String]);
                }
            }
        } else if (result) {
            // Log successful results for debugging
            if ([result isKindOfClass:[NSString class]]) {
                std::string resultStr = [(NSString*)result UTF8String];
                if (!resultStr.empty() && resultStr.length() < 200) {
                    spdlog::debug("JavaScript result: {}", resultStr);
                }
            }
        }
    }];
}

void WebViewMacOS::set_message_callback(mechatron::WebView::MessageCallback callback) {
    if (m_delegate) {
        m_delegate.messageCallback = callback;
    }
}

void WebViewMacOS::resize(int x, int y, int width, int height) {
    if (!m_webview) return;

    NSView* superview = [m_webview superview];
    CGFloat flipped_y = static_cast<CGFloat>(y);
    if (superview) {
        // ImGui reports screen coordinates from the top-left of the main
        // viewport, while Cocoa content views are bottom-left based.
        NSRect bounds = [superview bounds];
        flipped_y = NSHeight(bounds) - static_cast<CGFloat>(y) - static_cast<CGFloat>(height);
    }

    NSRect frame = NSMakeRect(
        static_cast<CGFloat>(x),
        std::max<CGFloat>(0.0, flipped_y),
        std::max<CGFloat>(0.0, static_cast<CGFloat>(width)),
        std::max<CGFloat>(0.0, static_cast<CGFloat>(height)));
    [m_webview setFrame:frame];
}

void WebViewMacOS::attach_to_window(void* glfwWindow) {
    if (!m_webview || !glfwWindow) return;

    NSWindow* hostWindow = glfwGetCocoaWindow(static_cast<GLFWwindow*>(glfwWindow));
    if (!hostWindow) return;

    NSView* contentView = [hostWindow contentView];
    if (!contentView) return;

    if ([m_webview superview] != contentView) {
        [m_webview removeFromSuperview];
        [m_webview setFrame:NSMakeRect(0, 0, 1, 1)];
        [m_webview setAutoresizingMask:NSViewNotSizable];
        [contentView addSubview:m_webview positioned:NSWindowAbove relativeTo:nil];
    }

    m_parent_window = hostWindow;
    spdlog::debug("WKWebView attached to GLFW content view");
}

void WebViewMacOS::set_visible(bool visible) {
    if (!m_webview) return;
    [m_webview setHidden:visible ? NO : YES];
}

void WebViewMacOS::render() {
    // WKWebView renders automatically via Cocoa
    // This is a no-op for macOS
}

WKWebView* WebViewMacOS::get_webview() {
    return m_webview;
}

#endif // __APPLE__
