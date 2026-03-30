#include "UIApplication.hpp"
#include <spdlog/spdlog.h>

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::debug);
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
