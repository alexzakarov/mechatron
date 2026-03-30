#include "Shader.hpp"
#include <GL/glew.h>
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

namespace mechatron {

Shader::~Shader() {
    if (m_program) glDeleteProgram(m_program);
}

bool Shader::load_from_file(const std::string& vertex_path, const std::string& fragment_path) {
    auto read_file = [](const std::string& path) -> std::string {
        std::ifstream f(path);
        if (!f.is_open()) return {};
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    };

    std::string vert = read_file(vertex_path);
    std::string frag = read_file(fragment_path);
    if (vert.empty() || frag.empty()) {
        spdlog::error("Failed to read shader files: {}, {}", vertex_path, fragment_path);
        return false;
    }
    return load_from_source(vert, frag);
}

bool Shader::load_from_source(const std::string& vertex_src, const std::string& fragment_src) {
    uint32_t vs = compile(GL_VERTEX_SHADER, vertex_src);
    if (!vs) return false;

    uint32_t fs = compile(GL_FRAGMENT_SHADER, fragment_src);
    if (!fs) {
        glDeleteShader(vs);
        return false;
    }

    link(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);

    return m_program != 0;
}

void Shader::bind() const {
    glUseProgram(m_program);
}

void Shader::unbind() const {
    glUseProgram(0);
}

void Shader::set_uniform(const std::string& name, float value) {
    glUniform1f(uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, int value) {
    glUniform1i(uniform_location(name), value);
}

void Shader::set_uniform(const std::string& name, const glm::vec3& value) {
    glUniform3f(uniform_location(name), value.x, value.y, value.z);
}

void Shader::set_uniform(const std::string& name, const glm::mat4& value) {
    glUniformMatrix4fv(uniform_location(name), 1, GL_FALSE, &value[0][0]);
}

uint32_t Shader::compile(uint32_t type, const std::string& source) {
    uint32_t shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        spdlog::error("Shader compile error: {}", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

void Shader::link(uint32_t vertex, uint32_t fragment) {
    m_program = glCreateProgram();
    glAttachShader(m_program, vertex);
    glAttachShader(m_program, fragment);
    glLinkProgram(m_program);

    int success;
    glGetProgramiv(m_program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(m_program, 512, nullptr, log);
        spdlog::error("Shader link error: {}", log);
        glDeleteProgram(m_program);
        m_program = 0;
    }
}

int Shader::uniform_location(const std::string& name) {
    auto it = m_uniform_cache.find(name);
    if (it != m_uniform_cache.end()) return it->second;
    int loc = glGetUniformLocation(m_program, name.c_str());
    m_uniform_cache[name] = loc;
    return loc;
}

} // namespace mechatron
