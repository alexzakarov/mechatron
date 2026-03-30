#include "CADKernel.hpp"
#include "OpenCascadeWrapper.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace mechatron {

// ============================================================================
// MeshData Implementation
// ============================================================================

void MeshData::get_bounds(Vec3& min, Vec3& max) const {
    if (vertices.empty()) {
        min = max = Vec3();
        return;
    }

    min = max = vertices[0];
    for (const auto& v : vertices) {
        min.x = std::min(min.x, v.x);
        min.y = std::min(min.y, v.y);
        min.z = std::min(min.z, v.z);
        max.x = std::max(max.x, v.x);
        max.y = std::max(max.y, v.y);
        max.z = std::max(max.z, v.z);
    }
}

void MeshData::calculate_normals() {
    normals.clear();
    normals.resize(vertices.size(), Vec3(0, 0, 0));

    // Calculate face normals and accumulate
    for (const auto& tri : triangles) {
        if (tri.v0 >= vertices.size() || tri.v1 >= vertices.size() || tri.v2 >= vertices.size()) {
            continue;
        }

        Vec3 v0 = vertices[tri.v0];
        Vec3 v1 = vertices[tri.v1];
        Vec3 v2 = vertices[tri.v2];

        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        Vec3 normal = edge1.cross(edge2).normalized();

        normals[tri.v0] = normals[tri.v0] + normal;
        normals[tri.v1] = normals[tri.v1] + normal;
        normals[tri.v2] = normals[tri.v2] + normal;
    }

    // Normalize accumulated normals
    for (auto& n : normals) {
        n = n.normalized();
    }
}

void MeshData::unify_vertices(float tolerance) {
    if (vertices.empty()) return;

    // Simple vertex welding based on position
    std::vector<Vec3> unified_vertices;
    std::vector<uint32_t> vertex_remap(vertices.size());

    for (size_t i = 0; i < vertices.size(); ++i) {
        bool found = false;
        for (size_t j = 0; j < unified_vertices.size(); ++j) {
            Vec3 diff = vertices[i] - unified_vertices[j];
            if (std::abs(diff.x) < tolerance &&
                std::abs(diff.y) < tolerance &&
                std::abs(diff.z) < tolerance) {
                vertex_remap[i] = j;
                found = true;
                break;
            }
        }

        if (!found) {
            vertex_remap[i] = unified_vertices.size();
            unified_vertices.push_back(vertices[i]);
        }
    }

    // Remap triangles
    for (auto& tri : triangles) {
        tri.v0 = vertex_remap[tri.v0];
        tri.v1 = vertex_remap[tri.v1];
        tri.v2 = vertex_remap[tri.v2];
    }

    vertices = unified_vertices;
}

// ============================================================================
// Geometry Implementation
// ============================================================================

MeshData Geometry::get_transformed_mesh() const {
    MeshData result = mesh;

    // Apply scale
    for (auto& v : result.vertices) {
        v.x *= scale.x;
        v.y *= scale.y;
        v.z *= scale.z;
    }

    // Apply rotation (Euler angles)
    float rx = rotation.x * 3.14159f / 180.0f;
    float ry = rotation.y * 3.14159f / 180.0f;
    float rz = rotation.z * 3.14159f / 180.0f;

    for (auto& v : result.vertices) {
        // Rotate around X
        float y1 = v.y * std::cos(rx) - v.z * std::sin(rx);
        float z1 = v.y * std::sin(rx) + v.z * std::cos(rx);
        v.y = y1;
        v.z = z1;

        // Rotate around Y
        float x2 = v.x * std::cos(ry) + v.z * std::sin(ry);
        float z2 = -v.x * std::sin(ry) + v.z * std::cos(ry);
        v.x = x2;
        v.z = z2;

        // Rotate around Z
        float x3 = v.x * std::cos(rz) - v.y * std::sin(rz);
        float y3 = v.x * std::sin(rz) + v.y * std::cos(rz);
        v.x = x3;
        v.y = y3;
    }

    // Apply translation
    for (auto& v : result.vertices) {
        v = v + position;
    }

    return result;
}

void BoxGeometry::generate_mesh() {
    mesh.clear();

    // 8 corners of a box
    float hw = width / 2;
    float hh = height / 2;
    float hd = depth / 2;

    mesh.vertices = {
        {-hw, -hh, -hd}, {hw, -hh, -hd}, {hw, hh, -hd}, {-hw, hh, -hd},  // Front face
        {-hw, -hh, hd},  {hw, -hh, hd},  {hw, hh, hd},  {-hw, hh, hd}   // Back face
    };

    // 12 triangles (2 per face * 6 faces)
    mesh.triangles = {
        // Front
        {0, 1, 2}, {0, 2, 3},
        // Back
        {5, 4, 7}, {5, 7, 6},
        // Left
        {4, 0, 3}, {4, 3, 7},
        // Right
        {1, 5, 6}, {1, 6, 2},
        // Bottom
        {4, 5, 1}, {4, 1, 0},
        // Top
        {3, 2, 6}, {3, 6, 7}
    };

    mesh.calculate_normals();
}

void SphereGeometry::generate_mesh() {
    mesh.clear();

    // Generate UV sphere
    for (int lat = 0; lat <= segments; ++lat) {
        float theta = lat * 3.14159f / segments;
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int lon = 0; lon <= segments; ++lon) {
            float phi = lon * 2 * 3.14159f / segments;
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            float x = cos_phi * sin_theta;
            float y = cos_theta;
            float z = sin_phi * sin_theta;

            mesh.vertices.push_back({x * radius, y * radius, z * radius});
        }
    }

    // Generate triangles
    for (int lat = 0; lat < segments; ++lat) {
        for (int lon = 0; lon < segments; ++lon) {
            uint32_t i0 = lat * (segments + 1) + lon;
            uint32_t i1 = i0 + 1;
            uint32_t i2 = (lat + 1) * (segments + 1) + lon;
            uint32_t i3 = i2 + 1;

            mesh.triangles.push_back({i0, i2, i1});
            mesh.triangles.push_back({i1, i2, i3});
        }
    }

    mesh.calculate_normals();
}

void CylinderGeometry::generate_mesh() {
    mesh.clear();

    float hh = height / 2;

    // Generate side vertices
    for (int i = 0; i <= segments; ++i) {
        float angle = i * 2 * 3.14159f / segments;
        float x = std::cos(angle) * radius;
        float z = std::sin(angle) * radius;

        // Bottom ring
        mesh.vertices.push_back({x, -hh, z});
        // Top ring
        mesh.vertices.push_back({x, hh, z});
    }

    // Generate side triangles
    for (int i = 0; i < segments; ++i) {
        uint32_t i0 = i * 2;
        uint32_t i1 = i0 + 1;
        uint32_t i2 = (i + 1) * 2;
        uint32_t i3 = i2 + 1;

        mesh.triangles.push_back({i0, i2, i1});
        mesh.triangles.push_back({i1, i2, i3});
    }

    // Add caps
    uint32_t center_bottom = mesh.vertices.size();
    mesh.vertices.push_back({0, -hh, 0});
    uint32_t center_top = mesh.vertices.size();
    mesh.vertices.push_back({0, hh, 0});

    for (int i = 0; i < segments; ++i) {
        uint32_t next = static_cast<uint32_t>((i + 1) % segments);
        // Bottom cap
        mesh.triangles.push_back({center_bottom, next * 2, static_cast<uint32_t>(i * 2)});
        // Top cap
        mesh.triangles.push_back({center_top, static_cast<uint32_t>(i * 2 + 1), next * 2 + 1});
    }

    mesh.calculate_normals();
}

// ============================================================================
// CADKernel Implementation
// ============================================================================

CADKernel::CADKernel() {
    spdlog::info("CAD Kernel initialized");

    // Initialize OpenCASCADE wrapper
    m_occt = std::make_unique<OpenCascadeWrapper>();
    if (has_opencascade_support()) {
        spdlog::info("OpenCASCADE support enabled");
    } else {
        spdlog::info("OpenCASCADE not available - STEP/IGES import disabled");
    }
}

CADKernel::~CADKernel() {
    spdlog::info("CAD Kernel destroyed");
}

std::shared_ptr<Geometry> CADKernel::create_box(float w, float h, float d) {
    return std::make_shared<BoxGeometry>(w, h, d);
}

std::shared_ptr<Geometry> CADKernel::create_sphere(float radius, int segments) {
    return std::make_shared<SphereGeometry>(radius, segments);
}

std::shared_ptr<Geometry> CADKernel::create_cylinder(float radius, float height, int segments) {
    return std::make_shared<CylinderGeometry>(radius, height, segments);
}

std::shared_ptr<Geometry> CADKernel::create_cone(float radius, float height, int segments) {
    // Simplified cone as cylinder with different top radius
    auto geom = std::make_shared<CylinderGeometry>(radius, height, segments);
    // Collapse top vertices to center
    for (size_t i = 0; i < geom->mesh.vertices.size(); i += 2) {
        geom->mesh.vertices[i + 1] = {0, height / 2, 0};
    }
    geom->mesh.calculate_normals();
    return geom;
}

std::shared_ptr<Geometry> CADKernel::create_mesh(const MeshData& mesh_data) {
    auto geom = std::make_shared<Geometry>();
    geom->type = GeometryType::CustomMesh;
    geom->mesh = mesh_data;
    return geom;
}

// ============================================================================
// STL File I/O
// ============================================================================

bool CADKernel::import_stl(const std::string& path, MeshData& out_mesh) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        m_error = "Failed to open file: " + path;
        spdlog::error(m_error);
        return false;
    }

    // Check if binary or ASCII
    char header[5];
    file.read(header, 5);
    file.close();

    if (std::strncmp(header, "solid", 5) == 0) {
        return read_stl_ascii(path, out_mesh);
    } else {
        return read_stl_binary(path, out_mesh);
    }
}

bool CADKernel::read_stl_ascii(const std::string& path, MeshData& mesh) {
    std::ifstream file(path);
    if (!file.is_open()) {
        m_error = "Failed to open file: " + path;
        return false;
    }

    mesh.clear();
    std::string line;

    while (std::getline(file, line)) {
        // Simple parsing - look for "vertex" keywords
        if (line.find("vertex") != std::string::npos) {
            float x, y, z;
            if (sscanf(line.c_str(), " vertex %f %f %f", &x, &y, &z) == 3) {
                mesh.vertices.push_back({x, y, z});
            }
        }
    }

    // Create triangles from vertices
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        if (i + 2 < mesh.vertices.size()) {
            mesh.triangles.push_back({(uint32_t)i, (uint32_t)(i + 1), (uint32_t)(i + 2)});
        }
    }

    mesh.calculate_normals();
    spdlog::info("Imported STL ASCII: {} triangles", mesh.triangles.size());
    return true;
}

bool CADKernel::read_stl_binary(const std::string& path, MeshData& mesh) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        m_error = "Failed to open file: " + path;
        return false;
    }

    mesh.clear();

    // Skip 80-byte header
    file.seekg(80, std::ios::beg);

    // Read triangle count
    uint32_t num_triangles;
    file.read(reinterpret_cast<char*>(&num_triangles), 4);

    // Read triangles
    for (uint32_t i = 0; i < num_triangles; ++i) {
        // Skip normal (3 floats)
        file.seekg(12, std::ios::cur);

        // Read 3 vertices
        uint32_t base_idx = mesh.vertices.size();
        for (int j = 0; j < 3; ++j) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), 4);
            file.read(reinterpret_cast<char*>(&y), 4);
            file.read(reinterpret_cast<char*>(&z), 4);
            mesh.vertices.push_back({x, y, z});
        }

        mesh.triangles.push_back({base_idx, base_idx + 1, base_idx + 2});

        // Skip attribute byte count
        file.seekg(2, std::ios::cur);
    }

    mesh.calculate_normals();
    spdlog::info("Imported STL Binary: {} triangles", mesh.triangles.size());
    return true;
}

bool CADKernel::export_stl(const std::string& path, const MeshData& mesh) {
    // Use binary for larger files
    if (mesh.triangles.size() > 1000) {
        return write_stl_binary(path, mesh);
    } else {
        return write_stl_ascii(path, mesh);
    }
}

bool CADKernel::write_stl_ascii(const std::string& path, const MeshData& mesh) {
    std::ofstream file(path);
    if (!file.is_open()) {
        m_error = "Failed to create file: " + path;
        return false;
    }

    file << "solid mechatron\n";

    for (const auto& tri : mesh.triangles) {
        if (tri.v0 >= mesh.vertices.size() ||
            tri.v1 >= mesh.vertices.size() ||
            tri.v2 >= mesh.vertices.size()) {
            continue;
        }

        const Vec3& v0 = mesh.vertices[tri.v0];
        const Vec3& v1 = mesh.vertices[tri.v1];
        const Vec3& v2 = mesh.vertices[tri.v2];

        file << "facet normal 0 0 0\n";
        file << "  outer loop\n";
        file << "    vertex " << v0.x << " " << v0.y << " " << v0.z << "\n";
        file << "    vertex " << v1.x << " " << v1.y << " " << v1.z << "\n";
        file << "    vertex " << v2.x << " " << v2.y << " " << v2.z << "\n";
        file << "  endloop\n";
        file << "endfacet\n";
    }

    file << "endsolid mechatron\n";
    spdlog::info("Exported STL ASCII: {} triangles", mesh.triangles.size());
    return true;
}

bool CADKernel::write_stl_binary(const std::string& path, const MeshData& mesh) {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        m_error = "Failed to create file: " + path;
        return false;
    }

    // Write 80-byte header
    char header[80] = {0};
    std::strncpy(header, "MECHATRON STL BINARY", 22);
    file.write(header, 80);

    // Write triangle count
    uint32_t num_triangles = mesh.triangles.size();
    file.write(reinterpret_cast<const char*>(&num_triangles), 4);

    // Write triangles
    for (const auto& tri : mesh.triangles) {
        if (tri.v0 >= mesh.vertices.size() ||
            tri.v1 >= mesh.vertices.size() ||
            tri.v2 >= mesh.vertices.size()) {
            continue;
        }

        const Vec3& v0 = mesh.vertices[tri.v0];
        const Vec3& v1 = mesh.vertices[tri.v1];
        const Vec3& v2 = mesh.vertices[tri.v2];

        // Calculate normal
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        Vec3 normal = edge1.cross(edge2);

        // Write normal
        file.write(reinterpret_cast<const char*>(&normal.x), 4);
        file.write(reinterpret_cast<const char*>(&normal.y), 4);
        file.write(reinterpret_cast<const char*>(&normal.z), 4);

        // Write vertices
        file.write(reinterpret_cast<const char*>(&v0.x), 4);
        file.write(reinterpret_cast<const char*>(&v0.y), 4);
        file.write(reinterpret_cast<const char*>(&v0.z), 4);
        file.write(reinterpret_cast<const char*>(&v1.x), 4);
        file.write(reinterpret_cast<const char*>(&v1.y), 4);
        file.write(reinterpret_cast<const char*>(&v1.z), 4);
        file.write(reinterpret_cast<const char*>(&v2.x), 4);
        file.write(reinterpret_cast<const char*>(&v2.y), 4);
        file.write(reinterpret_cast<const char*>(&v2.z), 4);

        // Write attribute byte count (0)
        uint16_t attr_count = 0;
        file.write(reinterpret_cast<const char*>(&attr_count), 2);
    }

    spdlog::info("Exported STL Binary: {} triangles", mesh.triangles.size());
    return true;
}

// ============================================================================
// OBJ File I/O (simplified)
// ============================================================================

bool CADKernel::import_obj(const std::string& path, MeshData& out_mesh) {
    std::ifstream file(path);
    if (!file.is_open()) {
        m_error = "Failed to open file: " + path;
        return false;
    }

    out_mesh.clear();
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            float x, y, z;
            if (iss >> x >> y >> z) {
                out_mesh.vertices.push_back({x, y, z});
            }
        } else if (type == "f") {
            // Simplified face parsing (vertices only)
            std::string v0, v1, v2;
            if (iss >> v0 >> v1 >> v2) {
                // Parse vertex indices (OBJ is 1-based)
                uint32_t i0 = std::stoi(v0) - 1;
                uint32_t i1 = std::stoi(v1) - 1;
                uint32_t i2 = std::stoi(v2) - 1;
                out_mesh.triangles.push_back({i0, i1, i2});
            }
        }
    }

    out_mesh.calculate_normals();
    spdlog::info("Imported OBJ: {} vertices, {} triangles",
                 out_mesh.vertices.size(), out_mesh.triangles.size());
    return true;
}

bool CADKernel::export_obj(const std::string& path, const MeshData& mesh) {
    std::ofstream file(path);
    if (!file.is_open()) {
        m_error = "Failed to create file: " + path;
        return false;
    }

    file << "# MECHATRON OBJ Export\n";

    // Write vertices
    for (const auto& v : mesh.vertices) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }

    // Write faces (OBJ is 1-based)
    for (const auto& tri : mesh.triangles) {
        file << "f " << (tri.v0 + 1) << " " << (tri.v1 + 1) << " " << (tri.v2 + 1) << "\n";
    }

    spdlog::info("Exported OBJ: {} vertices, {} faces",
                 mesh.vertices.size(), mesh.triangles.size());
    return true;
}

// ============================================================================
// STEP/IGES File I/O (requires OpenCASCADE)
// ============================================================================

bool CADKernel::has_opencascade_support() const {
    return m_occt && OpenCascadeWrapper::is_available();
}

bool CADKernel::import_step(const std::string& path, MeshData& out_mesh) {
    if (!m_occt) {
        m_error = "OpenCASCADE wrapper not initialized";
        return false;
    }

    ImportResult result = m_occt->import_step(path);
    if (!result.success) {
        m_error = result.error_message;
        return false;
    }

    out_mesh = result.mesh;
    return true;
}

bool CADKernel::import_iges(const std::string& path, MeshData& out_mesh) {
    if (!m_occt) {
        m_error = "OpenCASCADE wrapper not initialized";
        return false;
    }

    ImportResult result = m_occt->import_iges(path);
    if (!result.success) {
        m_error = result.error_message;
        return false;
    }

    out_mesh = result.mesh;
    return true;
}

bool CADKernel::import_brep(const std::string& path, MeshData& out_mesh) {
    if (!m_occt) {
        m_error = "OpenCASCADE wrapper not initialized";
        return false;
    }

    ImportResult result = m_occt->import_brep(path);
    if (!result.success) {
        m_error = result.error_message;
        return false;
    }

    out_mesh = result.mesh;
    return true;
}

bool CADKernel::export_step(const std::string& path, const MeshData& mesh) {
    if (!m_occt) {
        m_error = "OpenCASCADE wrapper not initialized";
        return false;
    }

    return m_occt->export_step(path, mesh);
}

// ============================================================================
// Boolean Operations (simplified placeholder)
// ============================================================================

bool CADKernel::union_meshes(const MeshData& a, const MeshData& b, MeshData& result) {
    if (m_occt && has_opencascade_support()) {
        return m_occt->boolean_union(a, b, result);
    }

    m_error = "CSG union requires OpenCASCADE - install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::warn(m_error);
    return false;
}

bool CADKernel::subtract_meshes(const MeshData& a, const MeshData& b, MeshData& result) {
    if (m_occt && has_opencascade_support()) {
        return m_occt->boolean_subtract(a, b, result);
    }

    m_error = "CSG subtract requires OpenCASCADE - install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::warn(m_error);
    return false;
}

bool CADKernel::intersect_meshes(const MeshData& a, const MeshData& b, MeshData& result) {
    if (m_occt && has_opencascade_support()) {
        return m_occt->boolean_intersect(a, b, result);
    }

    m_error = "CSG intersect requires OpenCASCADE - install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::warn(m_error);
    return false;
}

void CADKernel::simplify_mesh(MeshData& mesh, float target_ratio) {
    if (m_occt && has_opencascade_support()) {
        if (m_occt->mesh_simplification(mesh, target_ratio)) {
            spdlog::info("Mesh simplified to {}% of original", target_ratio * 100);
            return;
        }
    }
    spdlog::warn("Mesh simplification not available (requires OpenCASCADE)");
}

void CADKernel::smooth_mesh(MeshData& mesh, int iterations) {
    // TODO: Implement Laplacian smoothing
    spdlog::warn("Mesh smoothing not yet implemented");
}

} // namespace mechatron
