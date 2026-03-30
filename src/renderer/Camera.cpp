#include "Camera.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

namespace mechatron {

Camera::Camera() = default;

glm::mat4 Camera::view_matrix() const {
    return glm::lookAt(
        glm::vec3(m_position.x, m_position.y, m_position.z),
        glm::vec3(m_target.x, m_target.y, m_target.z),
        glm::vec3(m_up.x, m_up.y, m_up.z)
    );
}

glm::mat4 Camera::projection_matrix(float aspect) const {
    return glm::perspective(glm::radians(m_fov), aspect, m_near, m_far);
}

void Camera::orbit(float delta_yaw, float delta_pitch) {
    m_yaw += delta_yaw;
    m_pitch += delta_pitch;
    m_pitch = (std::max)(-89.0f, (std::min)(89.0f, m_pitch));

    float rad_yaw = glm::radians(m_yaw);
    float rad_pitch = glm::radians(m_pitch);

    m_position.x = m_target.x + m_distance * std::cos(rad_pitch) * std::cos(rad_yaw);
    m_position.y = m_target.y + m_distance * std::sin(rad_pitch);
    m_position.z = m_target.z + m_distance * std::cos(rad_pitch) * std::sin(rad_yaw);
}

void Camera::zoom(float delta) {
    m_distance = std::max(1.0f, m_distance - delta);
    orbit(0, 0);
}

void Camera::pan(float dx, float dy) {
    float rad_yaw = glm::radians(m_yaw);
    Vec3 right = {std::cos(rad_yaw), 0, std::sin(rad_yaw)};
    Vec3 forward = (m_target - m_position).normalized();
    Vec3 up = right.cross(forward);

    m_target = m_target + right * dx + up * dy;
    m_position = m_position + right * dx + up * dy;
}

void Camera::move(float forward_amt, float right_amt, float up_amt, float dt) {
    float rad_yaw = glm::radians(m_yaw);

    // Forward direction on XZ plane only (horizontal strafe)
    Vec3 forward = {std::cos(rad_yaw), 0, std::sin(rad_yaw)};
    // Right direction on XZ plane
    Vec3 right = {-std::sin(rad_yaw), 0, std::cos(rad_yaw)};

    float speed = 5.0f * dt;
    Vec3 offset = forward * (forward_amt * speed) + right * (right_amt * speed) + Vec3{0, 1, 0} * (up_amt * speed);

    m_position = m_position + offset;
    m_target = m_target + offset;
}

void Camera::reset() {
    m_position = {0, 5, 10};
    m_target = {0, 0, 0};
    m_up = {0, 1, 0};
    m_distance = 10.0f;
    m_yaw = -90.0f;
    m_pitch = 30.0f;
}

} // namespace mechatron
