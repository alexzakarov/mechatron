#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <cmath>

// Forward declaration
namespace mechatron {
    class OpenCascadeWrapper;
}

namespace mechatron {

// 3D Point/Vector
struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float len = length();
        return len > 0 ? Vec3(x / len, y / len, z / len) : Vec3();
    }
};

// Triangle face for mesh
struct Triangle {
    uint32_t v0, v1, v2;  // Vertex indices

    Triangle() : v0(0), v1(0), v2(0) {}
    Triangle(uint32_t a, uint32_t b, uint32_t c) : v0(a), v1(b), v2(c) {}
};

// Mesh data structure
struct MeshData {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<Vec3> tex_coords;
    std::vector<Triangle> triangles;

    void clear() {
        vertices.clear();
        normals.clear();
        tex_coords.clear();
        triangles.clear();
    }

    bool is_empty() const {
        return vertices.empty() || triangles.empty();
    }

    // Calculate bounding box
    void get_bounds(Vec3& min, Vec3& max) const;
    void calculate_normals();
    void unify_vertices(float tolerance = 0.0001f);
};

// Geometry primitive types
enum class GeometryType {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Torus,
    ExtrudedShape,
    RevolvedShape,
    CustomMesh
};

// CAD Geometry
struct Geometry {
    GeometryType type = GeometryType::CustomMesh;
    MeshData mesh;
    Vec3 position;
    Vec3 rotation;  // Euler angles in degrees
    Vec3 scale;

    Geometry() : scale(1, 1, 1) {}

    virtual ~Geometry() = default;

    // Transform operations
    void set_position(const Vec3& p) { position = p; }
    void set_rotation(const Vec3& r) { rotation = r; }
    void set_scale(const Vec3& s) { scale = s; }

    // Get transformed mesh
    MeshData get_transformed_mesh() const;
};

// Box geometry
struct BoxGeometry : public Geometry {
    float width, height, depth;

    BoxGeometry(float w = 1.0f, float h = 1.0f, float d = 1.0f)
        : width(w), height(h), depth(d) {
        type = GeometryType::Box;
        generate_mesh();
    }

    void generate_mesh();
};

// Sphere geometry
struct SphereGeometry : public Geometry {
    float radius;
    int segments;  // Latitude/longitude segments

    SphereGeometry(float r = 1.0f, int seg = 32)
        : radius(r), segments(seg) {
        type = GeometryType::Sphere;
        generate_mesh();
    }

    void generate_mesh();
};

// Cylinder geometry
struct CylinderGeometry : public Geometry {
    float radius, height;
    int segments;

    CylinderGeometry(float r = 1.0f, float h = 2.0f, int seg = 32)
        : radius(r), height(h), segments(seg) {
        type = GeometryType::Cylinder;
        generate_mesh();
    }

    void generate_mesh();
};

// CAD Kernel - Main interface
class CADKernel {
public:
    CADKernel();
    ~CADKernel();

    // Geometry creation
    std::shared_ptr<Geometry> create_box(float w, float h, float d);
    std::shared_ptr<Geometry> create_sphere(float radius, int segments = 32);
    std::shared_ptr<Geometry> create_cylinder(float radius, float height, int segments = 32);
    std::shared_ptr<Geometry> create_cone(float radius, float height, int segments = 32);

    // Custom mesh
    std::shared_ptr<Geometry> create_mesh(const MeshData& mesh);

    // File I/O
    bool import_stl(const std::string& path, MeshData& out_mesh);
    bool export_stl(const std::string& path, const MeshData& mesh);

    bool import_obj(const std::string& path, MeshData& out_mesh);
    bool export_obj(const std::string& path, const MeshData& mesh);

    // CAD file import (STEP/IGES/BREP) - requires OpenCASCADE
    bool import_step(const std::string& path, MeshData& out_mesh);
    bool import_iges(const std::string& path, MeshData& out_mesh);
    bool import_brep(const std::string& path, MeshData& out_mesh);
    bool export_step(const std::string& path, const MeshData& mesh);

    // Check if OpenCASCADE is available
    bool has_opencascade_support() const;

    // Boolean operations (requires OpenCASCADE for proper CSG)
    bool union_meshes(const MeshData& a, const MeshData& b, MeshData& result);
    bool subtract_meshes(const MeshData& a, const MeshData& b, MeshData& result);
    bool intersect_meshes(const MeshData& a, const MeshData& b, MeshData& result);

    // Mesh processing
    void simplify_mesh(MeshData& mesh, float target_ratio);
    void smooth_mesh(MeshData& mesh, int iterations);

    // Error handling
    const std::string& error() const { return m_error; }

private:
    std::string m_error;
    std::unique_ptr<OpenCascadeWrapper> m_occt;

    bool read_stl_ascii(const std::string& path, MeshData& mesh);
    bool read_stl_binary(const std::string& path, MeshData& mesh);
    bool write_stl_ascii(const std::string& path, const MeshData& mesh);
    bool write_stl_binary(const std::string& path, const MeshData& mesh);
};

} // namespace mechatron
