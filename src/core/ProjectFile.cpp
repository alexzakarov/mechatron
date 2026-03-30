#include "ProjectFile.hpp"
#include <fstream>
#include <spdlog/spdlog.h>

namespace mechatron {

bool ProjectFile::load(const std::string& path) {
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            spdlog::error("Failed to open project file: {}", path);
            return false;
        }
        file >> m_data;
        spdlog::info("Project loaded: {} (v{})", name(), version());
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to parse project file '{}': {}", path, e.what());
        return false;
    }
}

bool ProjectFile::save(const std::string& path) const {
    try {
        std::ofstream file(path);
        if (!file.is_open()) {
            spdlog::error("Failed to create project file: {}", path);
            return false;
        }
        file << m_data.dump(2);
        spdlog::info("Project saved: {}", path);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to save project file '{}': {}", path, e.what());
        return false;
    }
}

std::string ProjectFile::name() const {
    if (m_data.is_null() || !m_data.contains("name")) {
        return "Untitled";
    }
    return m_data["name"].get<std::string>();
}

std::string ProjectFile::version() const {
    if (m_data.is_null() || !m_data.contains("version")) {
        return "2.0";
    }
    return m_data["version"].get<std::string>();
}

std::vector<std::string> ProjectFile::required_plugins() const {
    std::vector<std::string> result;
    if (m_data.contains("plugins")) {
        for (const auto& p : m_data["plugins"]) {
            result.push_back(p.get<std::string>());
        }
    }
    return result;
}

} // namespace mechatron
