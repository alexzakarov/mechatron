#pragma once

#include "Types.hpp"
#include <glm/glm.hpp>

namespace mechatron {

class Camera {
public:
    Camera();

    glm::mat4 view_matrix() const;
    glm::mat4 projection_matrix(float aspect) const;

    void set_position(Vec3 pos) { m_position = pos; }
    void set_target(Vec3 target) { m_target = target; }
    void set_up(Vec3 up) { m_up = up; }

    Vec3 position() const { return m_position; }
    Vec3 target() const { return m_target; }
    float fov() const { return m_fov; }

    void set_fov(float fov) { m_fov = fov; }
    void set_near_far(float near, float far) { m_near = near; m_far = far; }

    // Orbit camera controls
    void orbit(float delta_yaw, float delta_pitch);
    void zoom(float delta);
    void pan(float dx, float dy);
    void move(float forward, float right, float up, float dt);
    void reset();

private:
    Vec3 m_position{0, 5, 10};
    Vec3 m_target{0, 0, 0};
    Vec3 m_up{0, 1, 0};

    float m_fov = 60.0f;
    float m_near = 0.1f;
    float m_far = 1000.0f;

    float m_distance = 10.0f;
    float m_yaw = -90.0f;
    float m_pitch = 30.0f;
};

} // namespace mechatron
