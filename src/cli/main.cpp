#include "MechatronCLI.hpp"
#include <spdlog/spdlog.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

namespace {

bool parse_port(const std::string& text, int& port) {
    try {
        size_t pos = 0;
        int parsed = std::stoi(text, &pos);
        if (pos != text.size() || parsed <= 0 || parsed > 65535) {
            return false;
        }
        port = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    bool verbose = false;
    std::vector<std::string> args;
    args.reserve(argc > 1 ? static_cast<size_t>(argc - 1) : 0);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--verbose") {
            verbose = true;
            continue;
        }
        args.push_back(std::move(arg));
    }

    spdlog::set_level(verbose ? spdlog::level::debug : spdlog::level::warn);

    mechatron::MechatronCLI cli;

    if (!cli.initialize()) {
        std::cerr << "Failed to initialize Mechatron CLI" << std::endl;
        return 1;
    }

    if (args.empty()) {
        // Interactive mode
        cli.run_interactive();
    } else {
        // Command execution mode
        if (args[0] == "--help" || args[0] == "-h") {
            std::cout << cli.get_help() << std::endl;
        } else if (args[0] == "--version" || args[0] == "-v") {
            auto result = cli.execute_command(std::vector<std::string>{"version"});
            std::cout << result.output << std::endl;
        } else if (args[0] == "--script" || args[0] == "-s") {
            if (args.size() < 2) {
                std::cerr << "Usage: mechatron-cli --script <script-file>" << std::endl;
                return 1;
            }

            auto result = cli.execute_script(args[1]);
            if (!result.success) {
                std::cerr << result.error << std::endl;
                return result.exit_code;
            }

            if (!result.output.empty()) {
                std::cout << result.output << std::endl;
            }
        } else if (args[0] == "--mcp" || args[0] == "-m") {
            // MCP server mode
            int port = mechatron::app_config::kMcpDefaultPort;
            if (args.size() > 1) {
                if (!parse_port(args[1], port)) {
                    std::cerr << "Invalid port number" << std::endl;
                    return 1;
                }
            }

            if (cli.start_mcp_server(port)) {
                std::cout << "MCP server started on port " << port << std::endl;
                std::cout << "Press Ctrl+C to stop..." << std::endl;

                // Keep running until interrupted
                while (true) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            } else {
                std::cerr << "MCP server backend is not available in this build" << std::endl;
                return 1;
            }
        } else {
            // Execute single command
            auto result = cli.execute_command(args);

            if (!result.success) {
                std::cerr << result.error << std::endl;
                return result.exit_code;
            }

            if (!result.output.empty()) {
                std::cout << result.output << std::endl;
            }
        }
    }

    cli.shutdown();
    return 0;
}
