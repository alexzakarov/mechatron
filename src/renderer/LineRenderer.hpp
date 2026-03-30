#pragma once

#include "Types.hpp"
#include <vector>
#include <cstdint>

namespace mechatron {

class LineRenderer {
public:
    LineRenderer();
    ~LineRenderer();

    void begin();
    void draw_line(const Vec3& a, const Vec3& b, uint32_t color = 0xFFFFFFFF);
    void draw_box(const Vec3& min, const Vec3& max, uint32_t color = 0xFFFFFFFF);
    void end(uint32_t shader_program, const float* view_proj);

private:
    struct LineVertex {
        Vec3 position;
        uint32_t color;
    };

    std::vector<LineVertex> m_vertices;
    uint32_t m_vao = 0;
    uint32_t m_vbo = 0;
};

} // namespace mechatron
