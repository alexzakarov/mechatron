#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace mechatron {

class ProjectFile {
public:
    bool load(const std::string& path);
    bool save(const std::string& path) const;

    const nlohmann::json& data() const { return m_data; }
    nlohmann::json& data() { return m_data; }

    std::string name() const;
    std::string version() const;
    std::vector<std::string> required_plugins() const;

private:
    nlohmann::json m_data;
};

} // namespace mechatron
