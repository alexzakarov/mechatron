#pragma once

#include <string>
#include <unordered_map>
#include <glm/glm.hpp>

namespace mechatron {

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool load_from_file(const std::string& vertex_path, const std::string& fragment_path);
    bool load_from_source(const std::string& vertex_src, const std::string& fragment_src);

    void bind() const;
    void unbind() const;

    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, const glm::vec3& value);
    void set_uniform(const std::string& name, const glm::mat4& value);

    uint32_t program_id() const { return m_program; }

private:
    uint32_t compile(uint32_t type, const std::string& source);
    void link(uint32_t vertex, uint32_t fragment);
    int uniform_location(const std::string& name);

    uint32_t m_program = 0;
    std::unordered_map<std::string, int> m_uniform_cache;
};

} // namespace mechatron
