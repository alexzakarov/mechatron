#include "UIApplication.hpp"
#include <spdlog/spdlog.h>
#include <cstdlib>
#include <string>

static spdlog::level::level_enum parse_log_level(const char* s) {
    if (!s) return spdlog::level::info;
    std::string v(s);
    for (auto& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (v == "trace") return spdlog::level::trace;
    if (v == "debug") return spdlog::level::debug;
    if (v == "info") return spdlog::level::info;
    if (v == "warn" || v == "warning") return spdlog::level::warn;
    if (v == "error") return spdlog::level::err;
    if (v == "critical") return spdlog::level::critical;
    if (v == "off") return spdlog::level::off;
    return spdlog::level::info;
}

int main(int argc, char* argv[]) {
    // Default to info to avoid long-run log overhead; override via MECHATRON_LOG=debug|trace|...
    spdlog::set_level(parse_log_level(std::getenv("MECHATRON_LOG")));
    spdlog::flush_on(spdlog::level::warn);
    spdlog::info("MECHATRON v0.1.0 - Universal Mechatronics Simulation Platform");

    mechatron::UIApplication app;

    if (!app.init()) {
        spdlog::error("Failed to initialize application");
        return 1;
    }

    app.run();
    app.shutdown();

    spdlog::info("MECHATRON shutdown complete");
    return 0;
}
