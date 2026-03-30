#pragma once

#include <array>
#include <cmath>

namespace mechatron {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float l = length();
        return l > 0 ? *this / l : Vec3{};
    }
    constexpr float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
};

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

struct Transform {
    Vec3 position;
    Quat rotation;
    Vec3 scale{1, 1, 1};
};

} // namespace mechatron
