#include "MonacoCodeEditor.hpp"
#include "core/SimulationOrchestrator.hpp"
#include "core/Registry.hpp"
#include "core/SystemConfig.hpp"
#include <imgui.h>
#include "Theme.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <cstdio>
#include <cstring>
#include <thread>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace mechatron {

namespace {
std::string json_escape_for_script(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 16);
    for (char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += ch; break;
        }
    }
    return escaped;
}

std::string file_uri_from_path(const std::filesystem::path& path) {
    std::string value = std::filesystem::absolute(path).lexically_normal().string();
    std::string uri = "file://";
    for (unsigned char ch : value) {
        if (ch == ' ') {
            uri += "%20";
        } else {
            uri += static_cast<char>(ch);
        }
    }
    return uri;
}

// Helper functions from old CodeEditor
std::string shell_quote(const std::string& value) {
    std::string quoted = "'";
    for (char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

std::string filename_without_extension(const std::string& path) {
    return std::filesystem::path(path).stem().string();
}

std::string default_sketch_path() {
    return (std::filesystem::current_path() / "sketch.ino").string();
}

std::string default_sketch_content(int baud_rate) {
    (void)baud_rate;
    return {};
}

bool input_path_string(const char* label, std::string& value) {
    char buffer[4096] = {};
    std::snprintf(buffer, sizeof(buffer), "%s", value.c_str());
    if (ImGui::InputText(label, buffer, sizeof(buffer))) {
        value = buffer;
        return true;
    }
    return false;
}

std::string find_project_file(const std::filesystem::path& relative_path) {
    namespace fs = std::filesystem;

    std::error_code ec;
    fs::path current = fs::current_path(ec);
    if (ec) return {};

    for (int depth = 0; depth < 6; ++depth) {
        fs::path candidate = current / relative_path;
        if (fs::exists(candidate, ec)) {
            return candidate.lexically_normal().string();
        }
        if (!current.has_parent_path() || current == current.parent_path()) {
            break;
        }
        current = current.parent_path();
    }
    return {};
}

std::string bundled_arduino_cli_path() {
    return find_project_file("tools/arduino-cli/bin/arduino-cli");
}

std::string bundled_arduino_lsp_cli_wrapper_path() {
    return find_project_file("tools/arduino-cli/bin/arduino-cli-lsp-wrapper");
}

std::string bundled_arduino_cli_config_path() {
    return find_project_file("tools/arduino-cli/arduino-cli.yaml");
}

std::filesystem::path project_root_from_cli_config(const std::string& config_path) {
    if (config_path.empty()) return {};

    std::filesystem::path config(config_path);
    if (config.filename() != "arduino-cli.yaml") return {};

    auto cli_dir = config.parent_path();
    auto tools_dir = cli_dir.parent_path();
    return tools_dir.parent_path();
}

std::filesystem::path lsp_sketch_path(const std::filesystem::path& project_root) {
    if (!project_root.empty()) {
        return project_root / ".mechatron_lsp" / "sketch" / "sketch.ino";
    }
    return std::filesystem::current_path() / ".mechatron_lsp" / "sketch" / "sketch.ino";
}

nlohmann::json lsp_end_position(const std::string& content) {
    int line = 0;
    int character = 0;
    for (char ch : content) {
        if (ch == '\n') {
            ++line;
            character = 0;
        } else {
            ++character;
        }
    }
    return {{"line", line}, {"character", character}};
}

} // anonymous namespace

class ArduinoLspClient {
public:
    ArduinoLspClient() = default;
    ~ArduinoLspClient() { stop(); }

    bool start(const std::string& server_path,
               const std::string& cli_path,
               const std::string& cli_config_path,
               const std::string& fqbn,
               const std::string& initial_content) {
        if (m_ready) {
            return true;
        }
        if (server_path.empty() || cli_path.empty() || cli_config_path.empty()) {
            m_last_error = "Arduino LSP path configuration is incomplete.";
            return false;
        }

        std::filesystem::path project_root = project_root_from_cli_config(cli_config_path);
        if (project_root.empty()) {
            project_root = std::filesystem::current_path();
        }
        const std::filesystem::path sketch = lsp_sketch_path(project_root);
        const std::string bootstrap_content = "#include <Arduino.h>\nvoid setup() {}\nvoid loop() {}\n";
        m_workspace_root = sketch.parent_path();
        m_document_path = sketch;
        std::error_code ec;
        std::filesystem::create_directories(m_workspace_root, ec);
        if (ec) {
            m_last_error = "Failed to create Arduino LSP sketch workspace.";
            return false;
        }
        {
            std::ofstream out(sketch);
            if (!out.is_open()) {
                m_last_error = "Failed to write Arduino LSP bootstrap sketch.";
                return false;
            }
            out << bootstrap_content;
        }
        (void)initial_content;

#ifdef _WIN32
        m_last_error = "Arduino LSP process bridge is not implemented for Windows yet.";
        return false;
#else
        signal(SIGPIPE, SIG_IGN);

        int in_pipe[2] = {-1, -1};
        int out_pipe[2] = {-1, -1};
        if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0) {
            m_last_error = "Failed to create Arduino LSP pipes.";
            return false;
        }

        pid_t pid = fork();
        if (pid < 0) {
            close(in_pipe[0]); close(in_pipe[1]);
            close(out_pipe[0]); close(out_pipe[1]);
            m_last_error = "Failed to fork Arduino LSP process.";
            return false;
        }

        if (pid == 0) {
            dup2(in_pipe[0], STDIN_FILENO);
            dup2(out_pipe[1], STDOUT_FILENO);
            int dev_null = open("/dev/null", O_WRONLY);
            if (dev_null >= 0) {
                dup2(dev_null, STDERR_FILENO);
                close(dev_null);
            }
            close(in_pipe[0]);
            close(in_pipe[1]);
            close(out_pipe[0]);
            close(out_pipe[1]);

            execl(server_path.c_str(),
                  server_path.c_str(),
                  "-clangd", "/usr/bin/clangd",
                  "-cli", cli_path.c_str(),
                  "-cli-config", cli_config_path.c_str(),
                  "-fqbn", fqbn.c_str(),
                  "-skip-libraries-discovery-on-rebuild",
                  nullptr);
            _exit(127);
        }

        close(in_pipe[0]);
        close(out_pipe[1]);
        m_pid = pid;
        m_stdin_fd = in_pipe[1];
        m_stdout_fd = out_pipe[0];
        m_running = true;
        m_reader_thread = std::thread([this]() { reader_loop(); });

        nlohmann::json init_params = {
            {"processId", static_cast<int>(getpid())},
            {"rootUri", file_uri_from_path(m_workspace_root)},
            {"capabilities", {
                {"textDocument", {
                    {"synchronization", {
                        {"didSave", true},
                        {"dynamicRegistration", false}
                    }},
                    {"completion", {
                        {"completionItem", {
                            {"snippetSupport", true},
                            {"documentationFormat", {"markdown", "plaintext"}}
                        }}
                    }},
                    {"publishDiagnostics", {
                        {"relatedInformation", true}
                    }}
                }},
                {"workspace", {
                    {"workspaceFolders", true},
                    {"configuration", true}
                }}
            }},
            {"workspaceFolders", {{
                {"uri", file_uri_from_path(m_workspace_root)},
                {"name", m_workspace_root.filename().string()}
            }}}
        };

        auto init_result = request("initialize", init_params, std::chrono::milliseconds(10000));
        if (!init_result) {
            stop();
            if (m_last_error.empty()) {
                m_last_error = "Arduino LSP initialize timed out.";
            }
            return false;
        }

        notify("initialized", nlohmann::json::object());
        m_document_uri = file_uri_from_path(sketch);
        m_document_version = 1;
        m_document_open = true;
        m_document_content = bootstrap_content;
        notify("textDocument/didOpen", {
            {"textDocument", {
                {"uri", m_document_uri},
                {"languageId", "arduino"},
                {"version", m_document_version},
                {"text", bootstrap_content}
            }}
        });

        m_ready = true;
        m_last_error.clear();
        spdlog::info("[MonacoCodeEditor] Arduino language server started, workspace={}", m_workspace_root.string());
        return true;
#endif
    }

    void stop() {
        m_ready = false;
        m_running = false;

        if (m_stdin_fd >= 0) {
            notify("shutdown", nullptr);
            notify("exit", nullptr);
        }

#ifndef _WIN32
        if (m_stdin_fd >= 0) {
            close(m_stdin_fd);
            m_stdin_fd = -1;
        }
        if (m_stdout_fd >= 0) {
            close(m_stdout_fd);
            m_stdout_fd = -1;
        }
        if (m_pid > 0) {
            kill(m_pid, SIGTERM);
            waitpid(m_pid, nullptr, WNOHANG);
            m_pid = -1;
        }
#endif
        if (m_reader_thread.joinable()) {
            m_reader_thread.join();
        }
    }

    bool ready() const { return m_ready; }
    const std::string& last_error() const { return m_last_error; }

    std::optional<nlohmann::json> completion(const std::string& content, int line, int character) {
        std::lock_guard<std::mutex> completion_lock(m_completion_mutex);
        if (!m_ready) {
            m_last_error = "Arduino LSP is not ready.";
            return std::nullopt;
        }
        ensure_document(content);
        return request("textDocument/completion", {
            {"textDocument", {{"uri", m_document_uri}}},
            {"position", {{"line", std::max(0, line)}, {"character", std::max(0, character)}}},
            {"context", {{"triggerKind", 1}}}
        }, std::chrono::milliseconds(30000));
    }

    bool sync_document(const std::string& content) {
        std::lock_guard<std::mutex> completion_lock(m_completion_mutex);
        if (!m_ready) {
            return false;
        }
        ensure_document(content);
        return true;
    }

private:
    struct PendingRequest {
        std::mutex mutex;
        std::condition_variable cv;
        bool done = false;
        std::optional<nlohmann::json> result;
        std::string error;
    };

    std::optional<nlohmann::json> request(const std::string& method,
                                          const nlohmann::json& params,
                                          std::chrono::milliseconds timeout) {
        const int id = m_next_id++;
        auto pending = std::make_shared<PendingRequest>();
        {
            std::lock_guard<std::mutex> lock(m_pending_mutex);
            m_pending[id] = pending;
        }

        if (!send_json({{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}})) {
            std::lock_guard<std::mutex> pending_lock(m_pending_mutex);
            m_pending.erase(id);
            return std::nullopt;
        }

        std::unique_lock<std::mutex> lock(pending->mutex);
        if (!pending->cv.wait_for(lock, timeout, [&]() { return pending->done; })) {
            std::lock_guard<std::mutex> pending_lock(m_pending_mutex);
            m_pending.erase(id);
            m_last_error = method + " timed out.";
            return std::nullopt;
        }
        if (!pending->error.empty()) {
            m_last_error = pending->error;
            return std::nullopt;
        }
        return pending->result.value_or(nlohmann::json{});
    }

    void notify(const std::string& method, const nlohmann::json& params) {
        (void)send_json({{"jsonrpc", "2.0"}, {"method", method}, {"params", params}});
    }

    void ensure_document(const std::string& content) {
        const std::filesystem::path sketch = !m_document_path.empty() ? m_document_path : lsp_sketch_path(m_workspace_root.parent_path().parent_path());
        std::error_code ec;
        std::filesystem::create_directories(sketch.parent_path(), ec);
        std::ofstream out(sketch);
        if (out.is_open()) {
            out << content;
        }

        if (m_document_uri.empty()) {
            m_document_uri = file_uri_from_path(sketch);
        }

        if (m_document_open && content == m_document_content) {
            return;
        }

        ++m_document_version;

        if (!m_document_open) {
            notify("textDocument/didOpen", {
                {"textDocument", {
                    {"uri", m_document_uri},
                    {"languageId", "arduino"},
                    {"version", m_document_version},
                    {"text", content}
                }}
            });
            m_document_open = true;
            m_document_content = content;
            return;
        }

        notify("textDocument/didChange", {
            {"textDocument", {{"uri", m_document_uri}, {"version", m_document_version}}},
            {"contentChanges", {{
                {"range", {
                    {"start", {{"line", 0}, {"character", 0}}},
                    {"end", lsp_end_position(m_document_content)}
                }},
                {"rangeLength", m_document_content.size()},
                {"text", content}
            }}}
        });
        m_document_content = content;
    }

    bool send_json(const nlohmann::json& message) {
        if (m_stdin_fd < 0) return false;
        const std::string body = message.dump();
        const std::string framed = "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
        std::lock_guard<std::mutex> lock(m_write_mutex);
        const char* data = framed.data();
        size_t remaining = framed.size();
        while (remaining > 0) {
#ifndef _WIN32
            ssize_t written = write(m_stdin_fd, data, remaining);
            if (written <= 0) {
                const int error_code = errno;
                m_last_error = "Arduino LSP pipe write failed: ";
                m_last_error += std::strerror(error_code);
                m_ready = false;
                m_running = false;
                if (m_stdin_fd >= 0) {
                    close(m_stdin_fd);
                    m_stdin_fd = -1;
                }
                return false;
            }
            data += written;
            remaining -= static_cast<size_t>(written);
#else
            return false;
#endif
        }
        return true;
    }

    void reader_loop() {
        std::string buffer;
        char chunk[4096];
        while (m_running) {
#ifndef _WIN32
            ssize_t read_count = read(m_stdout_fd, chunk, sizeof(chunk));
            if (read_count <= 0) {
                break;
            }
            buffer.append(chunk, static_cast<size_t>(read_count));
            parse_messages(buffer);
#else
            break;
#endif
        }
        m_running = false;
        m_ready = false;
    }

    void parse_messages(std::string& buffer) {
        while (true) {
            const size_t header_end = buffer.find("\r\n\r\n");
            if (header_end == std::string::npos) return;
            const std::string header = buffer.substr(0, header_end);
            const std::string needle = "Content-Length:";
            const size_t length_pos = header.find(needle);
            if (length_pos == std::string::npos) {
                buffer.erase(0, header_end + 4);
                continue;
            }
            const size_t value_start = length_pos + needle.size();
            const size_t value_end = header.find("\r\n", value_start);
            const size_t body_length = static_cast<size_t>(std::stoul(header.substr(value_start, value_end - value_start)));
            const size_t body_start = header_end + 4;
            if (buffer.size() < body_start + body_length) return;
            const std::string body = buffer.substr(body_start, body_length);
            buffer.erase(0, body_start + body_length);
            try {
                handle_message(nlohmann::json::parse(body));
            } catch (const std::exception& e) {
                spdlog::warn("[MonacoCodeEditor] Failed to parse LSP message: {}", e.what());
            }
        }
    }

    void handle_message(const nlohmann::json& message) {
        if (message.contains("id") && !message.contains("method")) {
            const int id = message["id"].get<int>();
            std::shared_ptr<PendingRequest> pending;
            {
                std::lock_guard<std::mutex> lock(m_pending_mutex);
                auto it = m_pending.find(id);
                if (it == m_pending.end()) return;
                pending = it->second;
                m_pending.erase(it);
            }
            {
                std::lock_guard<std::mutex> lock(pending->mutex);
                if (message.contains("error")) {
                    pending->error = message["error"].dump();
                } else if (message.contains("result")) {
                    pending->result = message["result"];
                } else {
                    pending->result = nullptr;
                }
                pending->done = true;
            }
            pending->cv.notify_one();
            return;
        }

        if (message.contains("id") && message.contains("method")) {
            const auto id = message["id"];
            const std::string method = message.value("method", "");
            if (method == "workspace/configuration") {
                nlohmann::json items = nlohmann::json::array();
                for (const auto& ignored : message.value("params", nlohmann::json::object()).value("items", nlohmann::json::array())) {
                    (void)ignored;
                    items.push_back(nlohmann::json::object());
                }
                send_json({{"jsonrpc", "2.0"}, {"id", id}, {"result", items}});
            } else if (method == "workspace/workspaceFolders") {
                send_json({{"jsonrpc", "2.0"}, {"id", id}, {"result", {{{"uri", file_uri_from_path(m_workspace_root)}, {"name", m_workspace_root.filename().string()}}}}});
            } else {
                send_json({{"jsonrpc", "2.0"}, {"id", id}, {"result", nullptr}});
            }
        }
    }

    std::atomic_bool m_running{false};
    std::atomic_bool m_ready{false};
    std::atomic_int m_next_id{1};
    std::thread m_reader_thread;
    std::mutex m_write_mutex;
    std::mutex m_pending_mutex;
    std::mutex m_completion_mutex;
    std::map<int, std::shared_ptr<PendingRequest>> m_pending;
    std::string m_last_error;
    std::filesystem::path m_workspace_root;
    std::filesystem::path m_document_path;
    std::string m_document_uri;
    std::string m_document_content;
    int m_document_version = 0;
    bool m_document_open = false;

#ifndef _WIN32
    pid_t m_pid = -1;
    int m_stdin_fd = -1;
    int m_stdout_fd = -1;
#else
    int m_stdin_fd = -1;
    int m_stdout_fd = -1;
#endif
};

namespace {

// HTML template for Monaco Editor
const char* monaco_editor_html = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mechatron Code Editor</title>
    <style>
        html, body {
            margin: 0;
            padding: 0;
            height: 100%;
            overflow: hidden;
            background: #1e1e1e;
            color: #d4d4d4;
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
        }
        #editor-container {
            width: 100%;
            height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        #loading {
            color: #fff;
            font-size: 18px;
            text-align: center;
        }
        #error {
            display: none;
            max-width: 760px;
            color: #ffb4b4;
            background: #2d1515;
            border: 1px solid #8a3333;
            border-radius: 6px;
            padding: 16px;
            line-height: 1.45;
        }
        .test-content {
            background: #2d2d2d;
            padding: 20px;
            border-radius: 8px;
            min-height: 100px;
        }
    </style>
</head>
<body>
    <div id="loading">Loading Monaco Editor...</div>
    <div id="error"></div>
    <div id="editor-container">
        <div class="test-content">
            <h2>Monaco Editor Test</h2>
            <p>If you see this, WKWebView is working!</p>
            <p>Monaco Editor will load from CDN...</p>
        </div>
    </div>

    <script>
        console.log('Page loaded, setting up Monaco bridge...');

        // Setup communication bridge FIRST
        window.mechatron = {
            sendMessage: function(msg) {
                console.log('Sending message:', msg);
                try {
                    if (window.webkit && window.webkit.messageHandlers && window.webkit.messageHandlers.mechatron) {
                        window.webkit.messageHandlers.mechatron.postMessage(msg);
                    } else if (window.chrome && window.chrome.webview) {
                        window.chrome.webview.postMessage(msg);
                    } else {
                        console.log('Native message bridge not available yet');
                    }
                } catch(e) {
                    console.log('Message handler not available yet:', e);
                }
            },
            onMessage: null
        };

        console.log('Mechatron bridge initialized');

        window.showMonacoLoadError = function(detail) {
            const message = 'Monaco Editor could not be loaded. ' + detail;
            document.getElementById('loading').style.display = 'none';
            document.getElementById('editor-container').style.display = 'none';
            const error = document.getElementById('error');
            error.style.display = 'block';
            error.textContent = message;
            window.mechatron.sendMessage(JSON.stringify({
                type: 'error',
                message: message
            }));
        };
    </script>

    <!-- Load Monaco Editor from CDN until a local bundle is packaged. -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.45.0/min/vs/loader.min.js"
            onerror="window.showMonacoLoadError('The CDN loader script is unavailable.')"></script>

    <script>
        if (typeof require === 'undefined') {
            window.showMonacoLoadError('The AMD loader did not initialize.');
        } else {
        require.onError = function(err) {
            window.showMonacoLoadError(err && err.message ? err.message : 'Monaco module loading failed.');
        };

        // Configure Monaco paths
        require.config({
            paths: {
                'vs': 'https://cdnjs.cloudflare.com/ajax/libs/monaco-editor/0.45.0/min/vs'
            }
        });

        // Global editor instance
        let editor = null;
        let language = 'arduino';
        let theme = 'vs-dark';
        let fontSize = 14;
        let nextLspCompletionRequestId = 1;
        let deferredSuggestTimer = null;
        let isApplyingExternalContent = false;
        let currentDocumentGeneration = 0;
        const pendingLspCompletions = new Map();

        function completionRange(model, position) {
            const word = model.getWordUntilPosition(position);
            return {
                startLineNumber: position.lineNumber,
                endLineNumber: position.lineNumber,
                startColumn: word.startColumn,
                endColumn: word.endColumn
            };
        }

        function lspCompletionKindToMonaco(kind) {
            const K = monaco.languages.CompletionItemKind;
            const mapping = {
                1: K.Text,
                2: K.Method,
                3: K.Function,
                4: K.Constructor,
                5: K.Field,
                6: K.Variable,
                7: K.Class,
                8: K.Interface,
                9: K.Module,
                10: K.Property,
                11: K.Unit,
                12: K.Value,
                13: K.Enum,
                14: K.Keyword,
                15: K.Snippet,
                16: K.Color,
                17: K.File,
                18: K.Reference,
                19: K.Folder,
                20: K.EnumMember,
                21: K.Constant,
                22: K.Struct,
                23: K.Event,
                24: K.Operator,
                25: K.TypeParameter
            };
            return mapping[kind] || K.Text;
        }

        function normalizeLspDocumentation(documentation) {
            if (!documentation) {
                return undefined;
            }
            if (typeof documentation === 'string') {
                return documentation;
            }
            if (documentation.value) {
                return {
                    value: documentation.value
                };
            }
            return undefined;
        }

        function normalizeLspLabel(label) {
            if (typeof label === 'string') {
                return label;
            }
            if (label && typeof label.label === 'string') {
                return label.label;
            }
            return '';
        }

        function normalizeLspCompletions(result, range) {
            const items = Array.isArray(result) ? result : (result && Array.isArray(result.items) ? result.items : []);
            return items.map(function(item) {
                const label = normalizeLspLabel(item.label);
                const insertText = item.insertText || item.textEdit?.newText || label;
                const monacoItem = {
                    label: label,
                    kind: lspCompletionKindToMonaco(item.kind),
                    insertText: insertText,
                    detail: item.detail,
                    documentation: normalizeLspDocumentation(item.documentation),
                    sortText: item.sortText,
                    filterText: item.filterText,
                    range: range
                };
                if (item.insertTextFormat === 2) {
                    monacoItem.insertTextRules = monaco.languages.CompletionItemInsertTextRule.InsertAsSnippet;
                }
                return monacoItem;
            }).filter(function(item) {
                return item.label.length > 0;
            });
        }

        function requestLspCompletion(model, position) {
            if (!window.mechatron || typeof window.mechatron.sendMessage !== 'function') {
                return Promise.resolve(null);
            }

            const requestId = nextLspCompletionRequestId++;
            return new Promise(function(resolve) {
                const timeout = setTimeout(function() {
                    pendingLspCompletions.delete(requestId);
                    resolve(null);
                }, 30000);

                pendingLspCompletions.set(requestId, {
                    resolve: resolve,
                    timeout: timeout
                });

                try {
                    window.mechatron.sendMessage(JSON.stringify({
                        type: 'lspCompletionRequest',
                        id: requestId,
                        content: model.getValue(),
                        line: position.lineNumber - 1,
                        character: position.column - 1
                    }));
                } catch (e) {
                    clearTimeout(timeout);
                    pendingLspCompletions.delete(requestId);
                    resolve(null);
                }
            });
        }

        function registerLspCompletionProvider(languageId) {
            monaco.languages.registerCompletionItemProvider(languageId, {
                triggerCharacters: ['.', '>', ':', '"', "'", '#'],
                provideCompletionItems: function(model, position) {
                    const range = completionRange(model, position);
                    return requestLspCompletion(model, position).then(function(result) {
                        return {
                            suggestions: normalizeLspCompletions(result, range)
                        };
                    });
                }
            });
        }

        function installArduinoLanguageSupport() {
            const languageIds = monaco.languages.getLanguages().map(function(item) { return item.id; });
            if (languageIds.indexOf('arduino') === -1) {
                monaco.languages.register({
                    id: 'arduino',
                    extensions: ['.ino', '.pde'],
                    aliases: ['Arduino', 'arduino'],
                    mimetypes: ['text/x-arduino']
                });
            }

            monaco.languages.setLanguageConfiguration('arduino', {
                comments: {
                    lineComment: '//',
                    blockComment: ['/*', '*/']
                },
                brackets: [
                    ['{', '}'],
                    ['[', ']'],
                    ['(', ')']
                ],
                autoClosingPairs: [
                    { open: '{', close: '}' },
                    { open: '[', close: ']' },
                    { open: '(', close: ')' },
                    { open: '"', close: '"' },
                    { open: "'", close: "'" }
                ],
                surroundingPairs: [
                    { open: '{', close: '}' },
                    { open: '[', close: ']' },
                    { open: '(', close: ')' },
                    { open: '"', close: '"' },
                    { open: "'", close: "'" }
                ]
            });

            monaco.languages.setMonarchTokensProvider('arduino', {
                defaultToken: '',
                tokenPostfix: '.ino',
                keywords: [
                    'break', 'case', 'continue', 'default', 'do', 'else', 'for', 'if',
                    'return', 'switch', 'while', 'class', 'struct', 'public', 'private',
                    'protected', 'const', 'static', 'volatile', 'extern', 'true', 'false',
                    'HIGH', 'LOW', 'INPUT', 'OUTPUT', 'INPUT_PULLUP', 'LED_BUILTIN'
                ],
                typeKeywords: [
                    'void', 'bool', 'boolean', 'byte', 'char', 'word', 'int', 'short',
                    'long', 'float', 'double', 'String', 'unsigned', 'signed', 'size_t'
                ],
                operators: [
                    '=', '>', '<', '!', '~', '?', ':', '==', '<=', '>=', '!=', '&&', '||',
                    '++', '--', '+', '-', '*', '/', '&', '|', '^', '%', '<<', '>>'
                ],
                symbols: /[=><!~?:&|+\-*\/\^%]+/,
                escapes: /\\(?:[abfnrtv\\"'0-9xXuU])/,
                tokenizer: {
                    root: [
                        [/[a-zA-Z_]\w*/, {
                            cases: {
                                '@typeKeywords': 'type',
                                '@keywords': 'keyword',
                                '@default': 'identifier'
                            }
                        }],
                        { include: '@whitespace' },
                        [/[{}()\[\]]/, '@brackets'],
                        [/@symbols/, {
                            cases: {
                                '@operators': 'operator',
                                '@default': ''
                            }
                        }],
                        [/\d*\.\d+([eE][\-+]?\d+)?[fFdD]?/, 'number.float'],
                        [/0[xX][0-9a-fA-F]+[lL]?/, 'number.hex'],
                        [/\d+[lL]?/, 'number'],
                        [/[;,.]/, 'delimiter'],
                        [/"([^"\\]|\\.)*$/, 'string.invalid'],
                        [/"/, { token: 'string.quote', bracket: '@open', next: '@string' }],
                        [/'[^\\']'/, 'string'],
                        [/(')(@escapes)(')/, ['string', 'string.escape', 'string']],
                        [/'/, 'string.invalid']
                    ],
                    whitespace: [
                        [/[ \t\r\n]+/, ''],
                        [/\/\*/, 'comment', '@comment'],
                        [/\/\/.*$/, 'comment']
                    ],
                    comment: [
                        [/[^\/*]+/, 'comment'],
                        [/\*\//, 'comment', '@pop'],
                        [/[\/*]/, 'comment']
                    ],
                    string: [
                        [/[^\\"]+/, 'string'],
                        [/@escapes/, 'string.escape'],
                        [/\\./, 'string.escape.invalid'],
                        [/"/, { token: 'string.quote', bracket: '@close', next: '@pop' }]
                    ]
                }
            });
        }

        // Initialize Monaco Editor
        require(['vs/editor/editor.main'], function() {
            console.log('Monaco Editor loaded');

            installArduinoLanguageSupport();
            registerLspCompletionProvider('arduino');
            registerLspCompletionProvider('cpp');

            // Clear test content
            document.getElementById('editor-container').innerHTML = '';
            document.getElementById('loading').style.display = 'none';

            // Create Monaco editor container
            const container = document.createElement('div');
            container.style.width = '100%';
            container.style.height = '100vh';
            document.getElementById('editor-container').appendChild(container);

            editor = monaco.editor.create(container, {
                value: '',
                language: language,
                theme: theme,
                fontSize: fontSize,
                automaticLayout: true,
                minimap: { enabled: true },
                scrollBeyondLastLine: false,
                wordWrap: 'off',
                lineNumbers: 'on',
                renderWhitespace: 'selection',
                cursorBlinking: 'smooth',
                cursorSmoothCaretAnimation: 'on',
                smoothScrolling: true,
                tabSize: 4,
                insertSpaces: true,
                formatOnPaste: true,
                formatOnType: true,
                quickSuggestions: {
                    other: true,
                    comments: false,
                    strings: false
                },
                quickSuggestionsDelay: 1800,
                suggestOnTriggerCharacters: true,
                acceptSuggestionOnEnter: 'on',
                tabCompletion: 'on',
                snippetSuggestions: 'top',
                wordBasedSuggestions: 'off',
                parameterHints: {
                    enabled: true
                }
            });

            console.log('Monaco Editor created');

            // Notify C++ that editor is ready FIRST
            try {
                window.mechatron.sendMessage(JSON.stringify({
                    type: 'ready'
                }));
                console.log('Ready message sent');
            } catch(e) {
                console.error('Failed to send ready message:', e);
            }

            // Wait a bit for message handler to be ready, then add listeners
            setTimeout(function() {
                // Listen for content changes
                editor.onDidChangeModelContent(function() {
                    if (isApplyingExternalContent) {
                        return;
                    }
                    try {
                        window.mechatron.sendMessage(JSON.stringify({
                            type: 'contentChanged',
                            content: editor.getValue(),
                            generation: currentDocumentGeneration
                        }));
                        if (deferredSuggestTimer !== null) {
                            clearTimeout(deferredSuggestTimer);
                        }
                        deferredSuggestTimer = setTimeout(function() {
                            deferredSuggestTimer = null;
                            if (editor && editor.hasTextFocus()) {
                                editor.trigger('mechatron.lsp', 'editor.action.triggerSuggest', {});
                            }
                        }, 1800);
                    } catch(e) {
                        console.error('Failed to send content changed:', e);
                    }
                });

                // Listen for cursor position changes
                editor.onDidChangeCursorPosition(function(e) {
                    try {
                        window.mechatron.sendMessage(JSON.stringify({
                            type: 'cursorChanged',
                            line: e.position.lineNumber,
                            column: e.position.column
                        }));
                    } catch(e) {
                        // Ignore cursor errors
                    }
                });
                console.log('Event listeners added');
            }, 100);
        });

        // Messages from C++
        window.mechatron.onMessage = function(messageStr) {
            console.log('Received message from C++:', messageStr);
            try {
                const message = JSON.parse(messageStr);

                switch (message.type) {
                    case 'setContent':
                        if (editor) {
                            if (Number.isInteger(message.generation)) {
                                currentDocumentGeneration = message.generation;
                            }
                            isApplyingExternalContent = true;
                            try {
                                editor.setValue(message.content);
                            } finally {
                                isApplyingExternalContent = false;
                            }
                            console.log('Content set');
                        }
                        break;

                    case 'getContent':
                        if (editor) {
                            window.mechatron.sendMessage(JSON.stringify({
                                type: 'content',
                                content: editor.getValue(),
                                generation: currentDocumentGeneration
                            }));
                        }
                        break;

                    case 'setLanguage':
                        if (editor) {
                            monaco.editor.setModelLanguage(editor.getModel(), message.language);
                            language = message.language;
                            console.log('Language set to:', message.language);
                        }
                        break;

                    case 'setTheme':
                        if (editor) {
                            monaco.editor.setTheme(message.theme);
                            theme = message.theme;
                            console.log('Theme set to:', message.theme);
                        }
                        break;

                    case 'setFontSize':
                        if (editor) {
                            editor.updateOptions({ fontSize: message.fontSize });
                            fontSize = message.fontSize;
                            console.log('Font size set to:', message.fontSize);
                        }
                        break;

                    case 'getConfig':
                        if (editor) {
                            window.mechatron.sendMessage(JSON.stringify({
                                type: 'config',
                                language: language,
                                theme: theme,
                                fontSize: fontSize
                            }));
                        }
                        break;

                    case 'lspCompletionResult': {
                        const pending = pendingLspCompletions.get(message.id);
                        if (pending) {
                            clearTimeout(pending.timeout);
                            pendingLspCompletions.delete(message.id);
                            pending.resolve(message.result || null);
                        }
                        break;
                    }

                    case 'lspStatus':
                        console.log('Arduino LSP status:', message.ready, message.message || '');
                        break;

                    default:
                        console.log('Unknown message type:', message.type);
                }
            } catch(e) {
                console.error('Error processing message:', e);
            }
        };

        console.log('Monaco Editor page fully loaded');
        }
    </script>
</body>
</html>
)HTML";

} // anonymous namespace

MonacoCodeEditor::MonacoCodeEditor() {
    // Load configuration
    load_config();

    // Check for bundled Arduino CLI
    std::string bundled_cli = bundled_arduino_cli_path();
    if (!bundled_cli.empty()) {
        m_arduino_cli_path = bundled_cli;
    }

    std::string bundled_config = bundled_arduino_cli_config_path();
    if (!bundled_config.empty()) {
        // Use bundled config
    }

    if (m_content_cache.empty()) {
        m_content_cache = default_sketch_content(m_baud_rate);
    }
    m_last_saved_content = m_content_cache;
}

MonacoCodeEditor::~MonacoCodeEditor() {
    if (m_lsp_client) {
        m_lsp_client->stop();
    }
}

bool MonacoCodeEditor::initialize() {
    if (m_initialized) {
        return true;
    }

    m_webview = std::make_unique<WebView>();
    if (!m_webview->initialize()) {
        spdlog::error("Failed to initialize WebView for Monaco Editor");
        return false;
    }

    // Set message callback
    m_webview->set_message_callback([this](const std::string& msg) {
        handle_webview_message(msg);
    });

    if (m_host_window) {
        m_webview->attach_to_window(m_host_window);
        m_webview_attached = true;
    }

    // Load Monaco Editor HTML
    m_webview->load_html(monaco_editor_html);
    start_arduino_lsp();

    m_initialized = true;
    spdlog::info("Monaco Code Editor initialized");
    return true;
}

void MonacoCodeEditor::attach_to_window(void* window) {
    m_host_window = window;
    if (m_webview && m_webview->is_ready()) {
        m_webview->attach_to_window(window);
        m_webview_attached = true;
        spdlog::info("Monaco Code Editor attached to window");
    }
}

void MonacoCodeEditor::set_visible(bool visible) {
    m_visible = visible;
    if (m_webview && m_webview->is_ready()) {
        m_webview->set_visible(visible);
    }
}

void MonacoCodeEditor::render(SimulationOrchestrator& orchestrator) {
    if (!m_initialized) {
        if (!initialize()) {
            ImGui::Text("Failed to initialize Monaco Editor");
            return;
        }
    }
    if (m_host_window && !m_webview_attached && m_webview && m_webview->is_ready()) {
        m_webview->attach_to_window(m_host_window);
        m_webview_attached = true;
    }
    flush_webview_messages();
    log_lsp_not_ready_if_due();
    attempt_lsp_reconnection_if_needed();

    // Check auto-save
    check_auto_save();

    // Render menu bar
    render_menu_bar(orchestrator);

    if (!m_load_error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 0.85f)));
        ImGui::TextWrapped("%s", m_load_error.c_str());
        ImGui::PopStyleColor();
    }

    const float available_y = ImGui::GetContentRegionAvail().y;
    const float status_height = ImGui::GetFrameHeightWithSpacing();
    float bottom_panel_height = m_output_panel_height;
    if (available_y < 340.0f) {
        bottom_panel_height = std::max(82.0f, available_y * 0.32f);
    } else {
        bottom_panel_height = std::min(bottom_panel_height, available_y - status_height - 140.0f);
        bottom_panel_height = std::max(96.0f, bottom_panel_height);
    }

    render_editor_host(bottom_panel_height + status_height + ImGui::GetStyle().ItemSpacing.y);
    render_bottom_tabs(bottom_panel_height);

    // Render status bar
    render_status_bar();

    // Render settings dialog
    if (m_show_settings) {
        render_settings();
    }

    render_file_dialogs();
}

void MonacoCodeEditor::handle_webview_message(const std::string& message) {
    try {
        auto json = nlohmann::json::parse(message);
        std::string type = json.value("type", "");

        spdlog::debug("Received message from Monaco: type={}", type);

        if (type == "ready") {
            m_editor_ready = true;
            m_load_error.clear();
            spdlog::info("Monaco Editor is ready");

            // Send initial configuration
            update_editor_config();

            // Send current content if any
            if (!m_content_cache.empty()) {
                send_content_to_editor();
            }
        }
        else if (type == "contentChanged") {
            std::string incoming = json.value("content", "");
            const int generation = json.value("generation", m_editor_document_generation);
            if (generation != m_editor_document_generation) {
                spdlog::debug("[MonacoCodeEditor] Ignoring stale contentChanged generation {} (current {})",
                              generation, m_editor_document_generation);
                return;
            }

            // Ignore contentChanged events that simply echo back content we
            // just pushed via setContent/load_file. Without this, loading a
            // file would immediately mark it as modified and trigger an
            // auto-save that could race with switching files.
            if (incoming == m_last_saved_content) {
                m_content_cache = incoming;
                mark_content_as_clean(incoming);
                spdlog::debug("[MonacoCodeEditor] contentChanged matches last saved content, ignoring (len={})", incoming.size());
                return;
            }

            m_content_cache = incoming;
            m_file_modified = true;
            m_compiled_hex_path.clear();
            m_pending_instant_save = true;
            m_modified_file = resolve_current_save_target();
            m_last_content_change_time = ImGui::GetTime();
            spdlog::info("[MonacoCodeEditor] contentChanged event received (len={})", m_content_cache.size());
            (void)auto_save_current_content("contentChanged");
            if (m_lsp_client && m_lsp_client->ready()) {
                const std::string content = m_content_cache;
                std::thread([this, content]() {
                    if (m_lsp_client) {
                        (void)m_lsp_client->sync_document(content);
                    }
                }).detach();
            }
        }
        else if (type == "content") {
            std::string incoming = json.value("content", "");
            const int generation = json.value("generation", m_editor_document_generation);
            if (generation != m_editor_document_generation) {
                spdlog::debug("[MonacoCodeEditor] Ignoring stale content poll generation {} (current {})",
                              generation, m_editor_document_generation);
                return;
            }
            // Ignore polled content that matches what we last saved/loaded.
            if (incoming == m_last_saved_content) {
                m_content_cache = incoming;
                mark_content_as_clean(incoming);
                return;
            }
            if (incoming != m_content_cache) {
                m_content_cache = incoming;
                m_file_modified = true;
                m_compiled_hex_path.clear();
                m_pending_instant_save = true;
                m_modified_file = resolve_current_save_target();
                m_last_content_change_time = ImGui::GetTime();
                spdlog::info("[MonacoCodeEditor] Polled content changed (len={})", m_content_cache.size());
                (void)auto_save_current_content("contentPoll");
            }
        }
        else if (type == "cursorChanged") {
            // Could track cursor position for status bar
        }
        else if (type == "config") {
            // Config response
        }
        else if (type == "error") {
            m_load_error = json.value("message", "Monaco Editor failed to load.");
            spdlog::error("Monaco Editor load error: {}", m_load_error);
        }
        else if (type == "lspCompletionRequest") {
            handle_lsp_completion_request(json);
        }
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse message from Monaco: {}", e.what());
    }
}

void MonacoCodeEditor::send_content_to_editor() {
    if (!is_ready()) return;

    ++m_editor_document_generation;
    auto message = nlohmann::json{
        {"type", "setContent"},
        {"content", m_content_cache},
        {"generation", m_editor_document_generation}
    };

    m_webview->send_message(message.dump());
}

void MonacoCodeEditor::request_content_from_editor() {
    if (!is_ready()) return;

    auto message = nlohmann::json{
        {"type", "getContent"}
    };

    m_webview->send_message(message.dump());
}

void MonacoCodeEditor::update_editor_config() {
    if (!is_ready()) return;

    auto message = nlohmann::json{
        {"type", "setLanguage"},
        {"language", m_language}
    };
    m_webview->send_message(message.dump());

    message = nlohmann::json{
        {"type", "setTheme"},
        {"theme", m_theme}
    };
    m_webview->send_message(message.dump());

    message = nlohmann::json{
        {"type", "setFontSize"},
        {"fontSize", m_font_size}
    };
    m_webview->send_message(message.dump());
}

void MonacoCodeEditor::queue_webview_message(const nlohmann::json& message) {
    std::lock_guard<std::mutex> lock(m_webview_message_mutex);
    m_webview_message_queue.push_back(message.dump());
}

void MonacoCodeEditor::flush_webview_messages() {
    if (!m_webview || !m_webview->is_ready()) {
        return;
    }

    std::vector<std::string> pending;
    {
        std::lock_guard<std::mutex> lock(m_webview_message_mutex);
        pending.swap(m_webview_message_queue);
    }

    for (const auto& message : pending) {
        m_webview->send_message(message);
    }
}

void MonacoCodeEditor::log_lsp_not_ready_if_due() {
    const bool lsp_ready = m_lsp_client && m_lsp_client->ready();
    if (lsp_ready) {
        m_last_lsp_ready_false_log_time = -1.0;
        return;
    }

    const double now = ImGui::GetTime();
    if (m_last_lsp_ready_false_log_time >= 0.0 && now - m_last_lsp_ready_false_log_time < 1.0) {
        return;
    }

    m_last_lsp_ready_false_log_time = now;
    const std::string reason = m_lsp_client ? m_lsp_client->last_error() : "Arduino LSP client has not been created.";
    spdlog::warn("[MonacoCodeEditor] Arduino LSP ready=false: {}", reason);
}

void MonacoCodeEditor::attempt_lsp_reconnection_if_needed() {
    // Only attempt reconnection if LSP is not ready
    const bool lsp_ready = m_lsp_client && m_lsp_client->ready();
    if (lsp_ready) {
        m_last_lsp_reconnect_attempt_time = -1.0;
        return;
    }

    const double now = ImGui::GetTime();
    // Check if enough time has passed since last attempt
    if (m_last_lsp_reconnect_attempt_time >= 0.0 &&
        now - m_last_lsp_reconnect_attempt_time < LSP_RECONNECT_INTERVAL) {
        return;
    }

    // Attempt reconnection
    m_last_lsp_reconnect_attempt_time = now;
    spdlog::info("[MonacoCodeEditor] Attempting LSP reconnection...");
    start_arduino_lsp();
}

void MonacoCodeEditor::start_arduino_lsp() {
    if (m_lsp_client && m_lsp_client->ready()) {
        return;
    }

    std::string server_path = find_project_file("tools/arduino-language-server/bin/arduino-language-server");
    std::string cli_path = m_arduino_cli_path;
    if (cli_path.empty() || cli_path == "arduino-cli") {
        cli_path = bundled_arduino_cli_path();
    }
    const std::string lsp_cli_wrapper_path = bundled_arduino_lsp_cli_wrapper_path();
    if (!lsp_cli_wrapper_path.empty()) {
        cli_path = lsp_cli_wrapper_path;
    }
    std::string cli_config_path = bundled_arduino_cli_config_path();

    m_lsp_client = std::make_unique<ArduinoLspClient>();
    const bool started = m_lsp_client->start(server_path, cli_path, cli_config_path, m_fqbn, m_content_cache);
    if (!started) {
        const std::string error = m_lsp_client->last_error();
        spdlog::error("[MonacoCodeEditor] Arduino LSP failed to start: {}", error);
        queue_webview_message({
            {"type", "lspStatus"},
            {"ready", false},
            {"message", error}
        });
        return;
    }

    queue_webview_message({
        {"type", "lspStatus"},
        {"ready", true},
        {"message", "Arduino language server is ready"}
    });
}

void MonacoCodeEditor::handle_lsp_completion_request(const nlohmann::json& request) {
    const int request_id = request.value("id", 0);
    const std::string content = request.value("content", "");
    const int line = request.value("line", 0);
    const int character = request.value("character", 0);

    std::thread([this, request_id, content, line, character]() {
        nlohmann::json response = {
            {"type", "lspCompletionResult"},
            {"id", request_id},
            {"result", nullptr}
        };

        if (!m_lsp_client || !m_lsp_client->ready()) {
            response["error"] = "Arduino LSP is not ready.";
            queue_webview_message(response);
            return;
        }

        auto result = m_lsp_client->completion(content, line, character);
        if (result) {
            response["result"] = *result;
        } else {
            response["error"] = m_lsp_client->last_error();
        }
        queue_webview_message(response);
    }).detach();
}

void MonacoCodeEditor::set_content(const std::string& content) {
    m_content_cache = content;
    mark_content_as_clean(content);
    m_load_error.clear();
    if (is_ready()) {
        send_content_to_editor();
    }
}

std::string MonacoCodeEditor::get_content() const {
    return m_content_cache;
}

std::string MonacoCodeEditor::resolve_current_save_target() const {
    return m_current_file.empty() ? m_project_sketch_path : m_current_file;
}

void MonacoCodeEditor::mark_content_as_clean(const std::string& content) {
    m_last_saved_content = content;
    m_file_modified = false;
    m_pending_instant_save = false;
    m_modified_file.clear();
}

bool MonacoCodeEditor::write_content_to_file(const std::string& path,
                                             const std::string& content,
                                             bool make_current) {
    namespace fs = std::filesystem;

    if (path.empty()) {
        spdlog::warn("[MonacoCodeEditor] Refusing to save content without a path");
        return false;
    }

    std::error_code ec;
    const fs::path file_path(path);
    const fs::path parent = file_path.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
        if (ec) {
            spdlog::error("Failed to create parent directory for '{}': {}", path, ec.message());
            return false;
        }
    }

    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        spdlog::error("Failed to save file: {}", path);
        return false;
    }

    file << content;
    if (!file.good()) {
        spdlog::error("Failed while writing file: {}", path);
        return false;
    }

    if (make_current) {
        m_current_file = path;
    }

    if (content == m_content_cache && path == resolve_current_save_target()) {
        mark_content_as_clean(content);
    }

    auto now = std::chrono::steady_clock::now();
    m_last_save_time = std::chrono::duration<double>(now.time_since_epoch()).count();

    spdlog::info("Saved file: {}", path);

    if (m_on_save_callback) {
        m_on_save_callback(path);
    }
    return true;
}

bool MonacoCodeEditor::auto_save_current_content(const char* reason) {
    if (!m_auto_save_enabled) {
        spdlog::debug("[AutoSave] Immediate file persistence is active while auto-save setting is disabled");
    }

    const std::string active_target = resolve_current_save_target();
    const std::string save_target = m_modified_file.empty() ? active_target : m_modified_file;
    if (save_target.empty()) {
        spdlog::warn("[AutoSave] Cannot save {}: no target (current_file='{}', project_sketch='{}')",
                     reason ? reason : "content", m_current_file, m_project_sketch_path);
        return false;
    }

    if (!m_modified_file.empty() && m_modified_file != active_target) {
        spdlog::warn("[AutoSave] Refusing {} save to stale target: modified='{}' active='{}'",
                     reason ? reason : "content", m_modified_file, active_target);
        m_pending_instant_save = false;
        return false;
    }

    spdlog::debug("[AutoSave] Saving {} immediately to: {}", reason ? reason : "content", save_target);
    if (!write_content_to_file(save_target, m_content_cache, save_target == active_target)) {
        return false;
    }

    auto now_chrono = std::chrono::steady_clock::now();
    m_last_auto_save_time = std::chrono::duration<double>(now_chrono.time_since_epoch()).count();
    return true;
}

void MonacoCodeEditor::flush_pending_autosave_before_switch(const std::string& next_path) {
    if (!m_auto_save_enabled || !m_file_modified) {
        return;
    }

    const std::string active_target = resolve_current_save_target();
    const std::string save_target = m_modified_file.empty() ? active_target : m_modified_file;
    if (save_target.empty()) {
        spdlog::warn("[MonacoCodeEditor] Cannot flush modified buffer before opening '{}': no save target", next_path);
        return;
    }

    if (save_target != active_target) {
        spdlog::warn("[MonacoCodeEditor] Refusing cross-file auto-save before opening '{}': modified='{}' active='{}'",
                     next_path, save_target, active_target);
        m_pending_instant_save = false;
        return;
    }

    spdlog::debug("[MonacoCodeEditor] Flushing modified buffer before opening '{}'", next_path);
    (void)write_content_to_file(save_target, m_content_cache, true);
}

void MonacoCodeEditor::set_language(const std::string& language) {
    m_language = language;
    if (is_ready()) {
        update_editor_config();
    }
}

void MonacoCodeEditor::set_theme(const std::string& theme) {
    m_theme = theme;
    if (is_ready()) {
        update_editor_config();
    }
}

void MonacoCodeEditor::set_font_size(int size) {
    m_font_size = size;
    if (is_ready()) {
        update_editor_config();
    }
}

void MonacoCodeEditor::load_file(const std::string& path) {
    flush_pending_autosave_before_switch(path);

    std::ifstream file(path, std::ios::binary);
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string new_content = buffer.str();

        m_content_cache = new_content;
        m_current_file = path;
        mark_content_as_clean(new_content);
        m_compiled_hex_path.clear();

        // Push the new content to Monaco editor
        if (is_ready()) {
            send_content_to_editor();
        }

        add_recent_file(path);
        spdlog::info("Loaded file: {}", path);

        // Attempt to reconnect LSP if connection was lost
        if (!m_lsp_client || !m_lsp_client->ready()) {
            spdlog::info("[MonacoCodeEditor] LSP not ready, attempting reconnection after loading file: {}", path);
            start_arduino_lsp();
        }
    } else {
        spdlog::error("Failed to open file: {}", path);
    }
}

void MonacoCodeEditor::save_file(const std::string& path) {
    (void)write_content_to_file(path, m_content_cache, true);
}

void MonacoCodeEditor::add_recent_file(const std::string& path) {
    auto it = std::find(m_recent_files.begin(), m_recent_files.end(), path);
    if (it != m_recent_files.end()) {
        m_recent_files.erase(it);
    }

    m_recent_files.insert(m_recent_files.begin(), path);

    if (m_recent_files.size() > MAX_RECENT_FILES) {
        m_recent_files.resize(MAX_RECENT_FILES);
    }

    save_config();
}

void MonacoCodeEditor::save_config() {
    nlohmann::json config;
    config["recent_files"] = m_recent_files;
    config["arduino_cli_path"] = m_arduino_cli_path;
    config["fqbn"] = m_fqbn;
    config["baud_rate"] = m_baud_rate;
    config["auto_save_enabled"] = m_auto_save_enabled;
    config["auto_save_interval"] = m_auto_save_interval;
    config["language"] = m_language;
    config["theme"] = m_theme;
    config["font_size"] = m_font_size;

    try {
        std::ofstream out(config_file_path());
        out << config.dump(2);
        spdlog::debug("[MonacoCodeEditor] Configuration saved");
    } catch (const std::exception& e) {
        spdlog::warn("[MonacoCodeEditor] Failed to save configuration: {}", e.what());
    }
}

void MonacoCodeEditor::load_config() {
    try {
        std::ifstream in(config_file_path());
        if (!in.is_open()) {
            spdlog::debug("[MonacoCodeEditor] No configuration file found");
            return;
        }

        nlohmann::json config;
        in >> config;

        if (config.contains("recent_files")) {
            m_recent_files = config["recent_files"].get<std::vector<std::string>>();
        }
        if (config.contains("arduino_cli_path")) {
            m_arduino_cli_path = config["arduino_cli_path"];
        }
        if (config.contains("fqbn")) {
            m_fqbn = config["fqbn"];
        }
        if (config.contains("baud_rate")) {
            m_baud_rate = config["baud_rate"];
        }
        if (config.contains("auto_save_enabled")) {
            m_auto_save_enabled = config["auto_save_enabled"];
        }
        if (config.contains("auto_save_interval")) {
            m_auto_save_interval = config["auto_save_interval"];
        }
        if (config.contains("language")) {
            m_language = config["language"];
        }
        if (config.contains("theme")) {
            m_theme = config["theme"];
        }
        if (config.contains("font_size")) {
            m_font_size = config["font_size"];
        }

        spdlog::info("[MonacoCodeEditor] Configuration loaded");
    } catch (const std::exception& e) {
        spdlog::warn("[MonacoCodeEditor] Failed to load configuration: {}", e.what());
    }
}

void MonacoCodeEditor::set_auto_save_enabled(bool enabled) {
    m_auto_save_enabled = enabled;
    save_config();
}

bool MonacoCodeEditor::is_auto_save_enabled() const {
    return m_auto_save_enabled;
}

void MonacoCodeEditor::set_auto_save_interval(double seconds) {
    m_auto_save_interval = seconds;
    save_config();
}

double MonacoCodeEditor::auto_save_interval() const {
    return m_auto_save_interval;
}

void MonacoCodeEditor::check_auto_save() {
    if (!m_auto_save_enabled) {
        return;
    }

    const std::string active_target = resolve_current_save_target();
    const std::string save_target = m_modified_file.empty() ? active_target : m_modified_file;
    if (save_target.empty()) {
        static double last_empty_log = -1.0;
        double now_log = ImGui::GetTime();
        if (now_log - last_empty_log > 5.0) {
            last_empty_log = now_log;
            spdlog::warn("[AutoSave] No save target! current_file='{}' project_sketch='{}'",
                         m_current_file, m_project_sketch_path);
        }
        return;
    }

    // --- Path 1: contentChanged events from Monaco (instant save after 0.3s idle) ---
    if (m_pending_instant_save && m_file_modified) {
        const double now = ImGui::GetTime();
        if (now - m_last_content_change_time >= INSTANT_SAVE_DELAY) {
            if (!m_modified_file.empty() && m_modified_file != active_target) {
                spdlog::warn("[AutoSave] Refusing to save active content to stale target: modified='{}' active='{}'",
                             m_modified_file, active_target);
                m_pending_instant_save = false;
                return;
            }
            m_pending_instant_save = false;
            spdlog::info("[AutoSave] Triggering save to: {}", save_target);
            if (write_content_to_file(save_target, m_content_cache, save_target == active_target)) {
                auto now_chrono = std::chrono::steady_clock::now();
                m_last_auto_save_time = std::chrono::duration<double>(now_chrono.time_since_epoch()).count();
            }
            return;
        }
    }

    // --- Path 2: Polling fallback (request content from Monaco periodically) ---
    if (is_ready()) {
        const double now = ImGui::GetTime();
        if (now - m_last_content_poll_time >= CONTENT_POLL_INTERVAL) {
            m_last_content_poll_time = now;
            request_content_from_editor();
        }
    }
}

nlohmann::json MonacoCodeEditor::serialize_project_state() const {
    return {
        {"content", m_content_cache},
        {"current_file", m_current_file},
        {"file_modified", m_file_modified},
        {"selected_mcu_id", m_selected_mcu_id},
        {"compiled_hex_path", m_compiled_hex_path},
        {"language", m_language},
        {"theme", m_theme},
        {"font_size", m_font_size}
    };
}

void MonacoCodeEditor::deserialize_project_state(const nlohmann::json& state) {
    if (!state.is_object()) return;

    m_current_file = state.value("current_file", std::string{});
    m_file_modified = state.value("file_modified", false);
    m_selected_mcu_id = state.value("selected_mcu_id", std::string{});
    m_compiled_hex_path = state.value("compiled_hex_path", std::string{});
    m_language = state.value("language", "arduino");
    m_theme = state.value("theme", "vs-dark");
    m_font_size = state.value("font_size", 14);

    if (state.contains("content")) {
        const std::string restored_content = state.value("content", "");
        m_content_cache = restored_content;
        m_last_saved_content = restored_content;
        m_pending_instant_save = false;
        m_modified_file = m_file_modified ? resolve_current_save_target() : std::string{};
        if (is_ready()) {
            send_content_to_editor();
        }
    }

    if (is_ready()) {
        update_editor_config();
    }
}

// Arduino integration methods (from old CodeEditor)
bool MonacoCodeEditor::check_arduino_cli() {
    spdlog::info("Checking Arduino CLI availability...");
    std::string output = run_arduino_command({"version"});

    if (output.empty()) {
        m_compile_errors = "Arduino CLI not found. Please install arduino-cli or set the correct path.";
        spdlog::error(m_compile_errors);
        return false;
    }

    if (output.find("arduino-cli") != std::string::npos ||
        output.find("Version") != std::string::npos) {
        spdlog::info("Arduino CLI detected: {}", output);
        return true;
    }

    m_compile_errors = "Arduino CLI executable found but returned unexpected output: " + output;
    spdlog::error(m_compile_errors);
    return false;
}

std::string MonacoCodeEditor::run_arduino_command(const std::vector<std::string>& args) {
    std::string output;
    std::string cmd = m_arduino_cli_path;
    cmd = shell_quote(cmd);
    for (const auto& arg : args) {
        cmd += " " + shell_quote(arg);
    }
    cmd += " 2>&1";

    auto project_root = project_root_from_cli_config(bundled_arduino_cli_config_path());
    if (!project_root.empty()) {
        cmd = "cd " + shell_quote(project_root.string()) + " && " + cmd;
    }

#ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "r");
#else
    FILE* pipe = popen(cmd.c_str(), "r");
#endif

    if (!pipe) {
        spdlog::error("Failed to execute Arduino CLI command: {}", cmd);
        return "";
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }

#ifdef _WIN32
    int result = _pclose(pipe);
#else
    int result = pclose(pipe);
#endif

    if (result != 0) {
        spdlog::warn("Arduino CLI command exited with code: {}", result);
    }

    return output;
}

std::string MonacoCodeEditor::prepare_sketch_for_compile() {
    namespace fs = std::filesystem;

    if (!m_current_file.empty()) {
        save_file(m_current_file);
        fs::path current_path(m_current_file);

        // Arduino CLI requires the .ino file name to match the directory name
        // If current file is code/code.ino, we need to ensure the directory is named 'code'
        fs::path parent_dir = current_path.parent_path();
        fs::path sketch_dir = parent_dir;

        // Check if the .ino file name matches the parent directory name
        // Arduino convention: sketch_dir/sketch_name.ino
        std::string filename = current_path.filename().string();
        std::string dir_name = parent_dir.filename().string();

        // If filename without extension doesn't match directory name,
        // create a temporary sketch with proper naming
        std::string sketch_name = filename_without_extension(filename);
        if (sketch_name != dir_name) {
            spdlog::warn("Arduino sketch naming convention violation: {} vs {}, creating temporary sketch",
                        sketch_name, dir_name);

            // Create temporary sketch with proper naming
            fs::path temp_sketch_dir = fs::temp_directory_path() / sketch_name;
            std::error_code ec2;
            fs::create_directories(temp_sketch_dir, ec2);
            if (ec2) {
                m_compile_errors = "Failed to create temporary sketch directory: " + ec2.message();
                return {};
            }

            fs::path temp_sketch_file = temp_sketch_dir / filename;
            std::ofstream file(temp_sketch_file);
            if (!file.is_open()) {
                m_compile_errors = "Failed to write temporary sketch: " + temp_sketch_file.string();
                return {};
            }
            file << m_content_cache;
            return temp_sketch_dir.string();
        }

        return sketch_dir.string();
    }

    fs::path sketch_dir = fs::temp_directory_path() / "mechatron_sketch";
    std::error_code ec;
    fs::create_directories(sketch_dir, ec);
    if (ec) {
        m_compile_errors = "Failed to create temporary sketch directory: " + ec.message();
        return {};
    }

    fs::path sketch_file = sketch_dir / "mechatron_sketch.ino";
    std::ofstream file(sketch_file);
    if (!file.is_open()) {
        m_compile_errors = "Failed to write temporary sketch: " + sketch_file.string();
        return {};
    }
    file << m_content_cache;
    return sketch_dir.string();
}

std::string MonacoCodeEditor::find_hex_file(const std::string& build_path, const std::string& sketch_name) const {
    namespace fs = std::filesystem;

    if (build_path.empty()) return {};
    std::error_code ec;
    if (!fs::exists(build_path, ec)) return {};

    fs::path expected = fs::path(build_path) / (sketch_name + ".ino.hex");
    if (fs::exists(expected, ec)) {
        return expected.string();
    }

    for (const auto& entry : fs::recursive_directory_iterator(build_path, fs::directory_options::skip_permission_denied, ec)) {
        if (ec) break;
        if (!entry.is_regular_file(ec)) continue;
        if (entry.path().extension() == ".hex") {
            return entry.path().string();
        }
    }
    return {};
}

bool MonacoCodeEditor::compile_sketch() {
    spdlog::info("Compiling sketch...");
    m_compiling = true;
    m_compile_output.clear();
    m_compile_errors.clear();
    m_compiled_hex_path.clear();

    if (!check_arduino_cli()) {
        m_compiling = false;
        return false;
    }

    std::string sketch_dir = prepare_sketch_for_compile();
    if (sketch_dir.empty()) {
        m_compiling = false;
        return false;
    }
    std::string sketch_name = filename_without_extension(std::filesystem::path(sketch_dir).filename().string());

    std::vector<std::string> compile_args = {
        "compile",
        "--fqbn", m_fqbn,
        "--format", "json",
        sketch_dir
    };

    spdlog::info("Running: {} compile --fqbn {}", m_arduino_cli_path, m_fqbn);

    std::string result = run_arduino_command(compile_args);
    m_compile_output = result;

    try {
        auto json = nlohmann::json::parse(result);
        bool success = json.value("success", false);
        if (success) {
            std::string build_path;
            if (json.contains("builder_result") && json["builder_result"].contains("build_path")) {
                build_path = json["builder_result"]["build_path"].get<std::string>();
            } else if (json.contains("build_path")) {
                build_path = json["build_path"].get<std::string>();
            }

            m_compiled_hex_path = find_hex_file(build_path, sketch_name);
            if (m_compiled_hex_path.empty()) {
                m_compile_errors = "Compilation succeeded, but no .hex file was found in build path: " + build_path;
                m_compiling = false;
                return false;
            }

            spdlog::info("Compiled hex file: {}", m_compiled_hex_path);
            m_compiling = false;
            return true;
        }

        if (json.contains("error")) {
            m_compile_errors = json["error"].dump();
        } else {
            m_compile_errors = "Compilation failed. Check output for details.";
        }
    } catch (const std::exception& e) {
        if (result.find("Error") != std::string::npos || result.find("error") != std::string::npos) {
            m_compile_errors = result;
        } else {
            m_compile_errors = std::string("Could not parse Arduino CLI JSON output: ") + e.what() + "\n" + result;
        }
    }

    spdlog::error("Compilation failed: {}", m_compile_errors);
    m_compiling = false;
    return false;
}

bool MonacoCodeEditor::upload_to_mcu(SimulationOrchestrator& orchestrator) {
    spdlog::info("Uploading to MCU...");
    m_uploading = true;

    if (!compile_sketch()) {
        if (m_compile_errors.empty()) {
            m_compile_errors = "Upload failed: Compilation failed";
        } else {
            m_compile_errors = "Upload failed: Compilation failed\n" + m_compile_errors;
        }
        m_uploading = false;
        return false;
    }

    std::error_code ec;
    if (m_compiled_hex_path.empty() || !std::filesystem::exists(m_compiled_hex_path, ec)) {
        m_compile_errors = "Upload failed: No compiled hex file available";
        spdlog::error(m_compile_errors);
        m_uploading = false;
        return false;
    }

    auto& registry = orchestrator.registry();

    Component* mcu_component = nullptr;
    if (!m_selected_mcu_id.empty()) {
        Component* selected = registry.get(m_selected_mcu_id);
        if (selected && (SystemConfig::instance().has_capability(selected->component_type(), "mcu.interpreter") ||
                        SystemConfig::instance().has_capability(selected->component_type(), "mcu.physical_bridge"))) {
            mcu_component = selected;
        }
    }

    if (!mcu_component) {
        m_compile_errors = "No MCU selected. Please add an ATmega328P to the circuit and select it in the MCU dropdown.";
        spdlog::warn(m_compile_errors);
        m_uploading = false;
        return false;
    }

    spdlog::info("Found MCU component: {}", m_selected_mcu_id);

    if (!mcu_component->load_firmware_file(m_compiled_hex_path)) {
        m_compile_errors = "Upload failed: MCU rejected firmware file: " + m_compiled_hex_path;
        spdlog::error(m_compile_errors);
        m_uploading = false;
        return false;
    }
    orchestrator.mark_circuit_topology_dirty();

    m_compile_output = "Firmware compiled: " + m_compiled_hex_path + "\n";
    m_compile_output += "Uploaded to MCU: " + m_selected_mcu_id + "\n";

    spdlog::info("Firmware compiled at: {}", m_compiled_hex_path);
    spdlog::info("Firmware uploaded to MCU component: {}", m_selected_mcu_id);

    m_uploading = false;
    return true;
}

// Render methods
void MonacoCodeEditor::render_menu_bar(SimulationOrchestrator& orchestrator) {
    if (ImGui::Button("New")) {
        set_content("");
        m_current_file.clear();
        m_file_modified = false;
        m_pending_instant_save = false;
        m_modified_file.clear();
        m_compiled_hex_path.clear();
    }
    ImGui::SameLine();

    if (ImGui::Button("Open")) {
        m_file_dialog_path = m_current_file.empty() ? default_sketch_path() : m_current_file;
        m_show_open_dialog = true;
    }
    ImGui::SameLine();

    if (ImGui::Button("Save")) {
        const std::string& target = m_current_file.empty() ? m_project_sketch_path : m_current_file;
        if (target.empty()) {
            m_file_dialog_path = default_sketch_path();
            m_show_save_as_dialog = true;
        } else {
            save_file(target);
        }
    }
    ImGui::SameLine();

    if (ImGui::Button("Save As")) {
        m_file_dialog_path = m_current_file.empty() ? default_sketch_path() : m_current_file;
        m_show_save_as_dialog = true;
    }
    ImGui::SameLine();

    if (ImGui::BeginCombo("Recent##MonacoCodeEditor", "Recent")) {
        if (m_recent_files.empty()) {
            ImGui::TextDisabled("No recent files");
        }
        for (const auto& path : m_recent_files) {
            if (ImGui::Selectable(path.c_str())) {
                load_file(path);
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();

    if (ImGui::Button("Settings")) {
        m_show_settings = !m_show_settings;
    }
    ImGui::SameLine();

    // Compile button
    if (m_compiling) {
        ImGui::BeginDisabled();
        ImGui::Button("Compiling...");
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Compile")) {
            compile_sketch();
        }
    }
    ImGui::SameLine();

    render_mcu_selector(orchestrator);
    ImGui::SameLine();

    // Upload button
    if (m_uploading) {
        ImGui::BeginDisabled();
        ImGui::Button("Uploading...");
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Upload")) {
            upload_to_mcu(orchestrator);
        }
    }

    ImGui::Separator();
}

void MonacoCodeEditor::render_mcu_selector(SimulationOrchestrator& orchestrator) {
    std::vector<std::string> mcu_ids;
    orchestrator.registry().for_each([&](Component& comp) {
        if (SystemConfig::instance().has_capability(comp.component_type(), "mcu.interpreter") ||
            SystemConfig::instance().has_capability(comp.component_type(), "mcu.physical_bridge")) {
            mcu_ids.push_back(comp.id());
        }
    });
    std::sort(mcu_ids.begin(), mcu_ids.end());

    if (!m_selected_mcu_id.empty() &&
        std::find(mcu_ids.begin(), mcu_ids.end(), m_selected_mcu_id) == mcu_ids.end()) {
        m_selected_mcu_id.clear();
    }
    if (m_selected_mcu_id.empty() && !mcu_ids.empty()) {
        m_selected_mcu_id = mcu_ids.front();
    }

    const char* preview = m_selected_mcu_id.empty() ? "No MCU" : m_selected_mcu_id.c_str();
    ImGui::SetNextItemWidth(150);
    if (ImGui::BeginCombo("MCU##UploadTarget", preview)) {
        if (mcu_ids.empty()) {
            ImGui::TextDisabled("No MCU in circuit");
        }
        for (const auto& id : mcu_ids) {
            bool selected = (id == m_selected_mcu_id);
            if (ImGui::Selectable(id.c_str(), selected)) {
                m_selected_mcu_id = id;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

void MonacoCodeEditor::render_editor_host(float reserved_bottom_height) {
    ImVec2 available = ImGui::GetContentRegionAvail();
    available.y = std::max(120.0f, available.y - reserved_bottom_height);
    const bool any_popup_open = ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel);
    const bool ui_overlay_open =
        any_popup_open ||
        m_show_settings ||
        m_show_open_dialog ||
        m_show_save_as_dialog ||
        ImGui::IsPopupOpen("Open Sketch##Monaco") ||
        ImGui::IsPopupOpen("Save Sketch As##Monaco");

    ImGui::BeginChild("MonacoEditorHost",
                      available,
                      true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 host_pos = ImGui::GetWindowPos();
    ImVec2 host_size = ImGui::GetWindowSize();

    if (!is_ready()) {
        ImGui::TextDisabled("Monaco Editor loading...");
        if (!m_load_error.empty()) {
            ImGui::TextWrapped("%s", m_load_error.c_str());
        }
    }

    ImGui::EndChild();

    if (m_webview && m_webview->is_ready()) {
        set_visible(!ui_overlay_open);
        if (!ui_overlay_open) {
            m_webview->resize(static_cast<int>(host_pos.x),
                              static_cast<int>(host_pos.y),
                              static_cast<int>(std::max(1.0f, host_size.x)),
                              static_cast<int>(std::max(1.0f, host_size.y)));
        }
        m_webview->render();
    }
}

void MonacoCodeEditor::render_status_bar() {
    ImGui::Text("Monaco Editor | %s | %s | Status: %s",
        m_file_modified ? "Modified" : "Saved",
        m_current_file.empty() ? "Untitled" : m_current_file.c_str(),
        is_ready() ? "Ready" : "Loading...");
}

void MonacoCodeEditor::render_compilation_output() {
    if (m_compile_output.empty() && m_compile_errors.empty()) {
        ImGui::TextDisabled("No compilation output yet.");
        return;
    }

    if (ImGui::Button("Clear Output")) {
        m_compile_output.clear();
        m_compile_errors.clear();
        return;
    }
    ImGui::Separator();

    ImGui::BeginChild("CompilationOutputScroll",
                      ImVec2(0, 0),
                      false,
                      ImGuiWindowFlags_HorizontalScrollbar);
    if (!m_compile_errors.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, Theme::U32(Theme::WithAlpha(Theme::CurrentPalette().error, 1.0f)));
        ImGui::TextWrapped("%s", m_compile_errors.c_str());
        ImGui::PopStyleColor();
    }
    if (!m_compile_output.empty()) {
        ImGui::TextWrapped("%s", m_compile_output.c_str());
    }
    ImGui::EndChild();
}

void MonacoCodeEditor::render_bottom_tabs(float height) {
    ImGui::BeginChild("CodeEditorBottomPanel",
                      ImVec2(0, height),
                      true,
                      ImGuiWindowFlags_NoScrollbar);
    if (ImGui::BeginTabBar("CodeEditorBottomTabs", ImGuiTabBarFlags_None)) {
        if (ImGui::BeginTabItem("Compilation Output")) {
            render_compilation_output();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void MonacoCodeEditor::render_settings() {
    ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Monaco Editor Settings", &m_show_settings)) {
        char arduino_cli_buf[256];
        std::strncpy(arduino_cli_buf, m_arduino_cli_path.c_str(), sizeof(arduino_cli_buf));
        if (ImGui::InputText("Arduino CLI", arduino_cli_buf, sizeof(arduino_cli_buf))) {
            m_arduino_cli_path = arduino_cli_buf;
            save_config();
        }

        char fqbn_buf[64];
        std::strncpy(fqbn_buf, m_fqbn.c_str(), sizeof(fqbn_buf));
        if (ImGui::InputText("FQBN", fqbn_buf, sizeof(fqbn_buf))) {
            m_fqbn = fqbn_buf;
            save_config();
        }

        if (ImGui::InputInt("Baud Rate", &m_baud_rate)) {
            if (m_baud_rate < 300) m_baud_rate = 300;
            save_config();
        }
        if (ImGui::Checkbox("Auto Save", &m_auto_save_enabled)) {
            save_config();
        }
        if (ImGui::InputDouble("Auto Save Interval", &m_auto_save_interval, 1.0, 10.0, "%.1f s")) {
            if (m_auto_save_interval < 1.0) m_auto_save_interval = 1.0;
            save_config();
        }

        ImGui::Separator();
        ImGui::Text("Font Size: %d", m_font_size);
        if (ImGui::Button("Decrease Font")) {
            set_font_size(std::max(8, m_font_size - 2));
        }
        ImGui::SameLine();
        if (ImGui::Button("Increase Font")) {
            set_font_size(std::min(72, m_font_size + 2));
        }

        ImGui::End();
    }
}

void MonacoCodeEditor::render_file_dialogs() {
    if (m_show_open_dialog) {
        ImGui::OpenPopup("Open Sketch##Monaco");
        m_show_open_dialog = false;
    }
    if (m_show_save_as_dialog) {
        ImGui::OpenPopup("Save Sketch As##Monaco");
        m_show_save_as_dialog = false;
    }

    if (ImGui::BeginPopupModal("Open Sketch##Monaco", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        input_path_string("Path", m_file_dialog_path);
        ImGui::Spacing();
        if (ImGui::Button("Open", ImVec2(100, 0))) {
            load_file(m_file_dialog_path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("Save Sketch As##Monaco", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        input_path_string("Path", m_file_dialog_path);
        ImGui::Spacing();
        if (ImGui::Button("Save", ImVec2(100, 0))) {
            save_file(m_file_dialog_path);
            add_recent_file(m_file_dialog_path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

} // namespace mechatron
