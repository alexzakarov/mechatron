#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// Pin direction
enum class PinDirection {
    Input,
    Output,
    Bidirectional
};

// Pin type
enum class PinType {
    Digital,      // Digital logic (0/1)
    Analog,       // Analog voltage (continuous)
    Power,        // Power supply (VCC/GND)
    Ground        // Ground reference
};

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

    // Create from Euler angles (radians) - XYZ order
    static Quat from_euler(float pitch, float yaw, float roll) {
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);

        Quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    // Multiply (compose rotations)
    Quat operator*(const Quat& o) const {
        Quat result;
        result.w = w * o.w - x * o.x - y * o.y - z * o.z;
        result.x = w * o.x + x * o.w + y * o.z - z * o.y;
        result.y = w * o.y - x * o.z + y * o.w + z * o.x;
        result.z = w * o.z + x * o.y - y * o.x + z * o.w;
        return result;
    }

    // Convert to Euler angles (radians) - XYZ order
    Vec3 to_euler() const {
        Vec3 euler;

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f) {
            euler.y = std::copysign(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
        } else {
            euler.y = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }
};

struct Transform {
    Vec3 position;
    Quat rotation;
    Vec3 scale{1, 1, 1};
};

} // namespace mechatron
