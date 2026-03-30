#pragma once

#include "CADKernel.hpp"
#include <string>
#include <memory>

// Forward declarations for OpenCASCADE types (avoid including headers when not using OCCT)
#ifdef MECHATRON_USE_OPENCASCADE
// OpenCASCADE headers will be included here when enabled
namespace OCCT {
    class TopoDS_Shape;
    class StepReader;
    class IgesReader;
    class StlReader;
}
#endif

namespace mechatron {

// STEP/IGES import result
struct ImportResult {
    bool success = false;
    std::string error_message;
    MeshData mesh;
    size_t num_shapes = 0;
    size_t num_faces = 0;
    size_t num_edges = 0;
};

// OpenCASCADE Wrapper - provides CAD file import and boolean operations
class OpenCascadeWrapper {
public:
    OpenCascadeWrapper();
    ~OpenCascadeWrapper();

    // Check if OpenCASCADE is available
    static bool is_available();

    // STEP file import (.stp, .step)
    ImportResult import_step(const std::string& path);

    // IGES file import (.igs, .iges)
    ImportResult import_iges(const std::string& path);

    // BREP file import (.brep)
    ImportResult import_brep(const std::string& path);

    // Export to STEP
    bool export_step(const std::string& path, const MeshData& mesh);

    // Boolean operations on meshes (using OpenCASCADE)
    bool boolean_union(const MeshData& a, const MeshData& b, MeshData& result);
    bool boolean_subtract(const MeshData& a, const MeshData& b, MeshData& result);
    bool boolean_intersect(const MeshData& a, const MeshData& b, MeshData& result);

    // Mesh operations
    bool mesh_simplification(MeshData& mesh, float target_ratio);
    bool mesh_refinement(MeshData& mesh, float max_edge_length);

    // Get last error
    const std::string& error() const { return m_error; }

    // Convert OpenCASCADE shape to mesh
    static bool shape_to_mesh(const void* occt_shape, MeshData& mesh, float linear_deflection = 0.1f);

    // Convert mesh to OpenCASCADE shape
    static void* mesh_to_shape(const MeshData& mesh);

private:
    std::string m_error;
    bool m_initialized;

    // Initialize OpenCASCADE
    bool initialize();

#ifdef MECHATRON_USE_OPENCASCADE
    // Internal shape handling
    void* m_last_shape;  // Pointer to TopoDS_Shape
#endif
};

// Fallback implementation when OpenCASCADE is not available
class OpenCascadeFallback {
public:
    static ImportResult import_step(const std::string& path);
    static ImportResult import_iges(const std::string& path);
    static ImportResult import_brep(const std::string& path);
    static bool boolean_union(const MeshData& a, const MeshData& b, MeshData& result);
    static bool boolean_subtract(const MeshData& a, const MeshData& b, MeshData& result);
    static bool boolean_intersect(const MeshData& a, const MeshData& b, MeshData& result);
};

} // namespace mechatron
