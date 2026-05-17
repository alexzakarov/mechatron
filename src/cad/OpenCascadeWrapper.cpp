#include "OpenCascadeWrapper.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <unordered_map>
#include <vector>

#ifdef MECHATRON_USE_OPENCASCADE

// OpenCASCADE headers
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IGESControl_Reader.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Tool.hxx>
#include <TopLoc_Location.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message_ProgressRange.hxx>
#include <StepData_StepModel.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepOffsetAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <gp_XYZ.hxx>
#include <gp_Pnt.hxx>
#include <Poly_Array1OfTriangle.hxx>

#endif

namespace mechatron {

// ============================================================================
// OpenCascadeWrapper Implementation
// ============================================================================

OpenCascadeWrapper::OpenCascadeWrapper() : m_initialized(false)
#ifdef MECHATRON_USE_OPENCASCADE
    , m_last_shape(nullptr)
#endif
{
#ifdef MECHATRON_USE_OPENCASCADE
    m_initialized = initialize();
    if (m_initialized) {
        spdlog::info("OpenCASCADE initialized successfully");
    } else {
        spdlog::error("OpenCASCADE initialization failed - CAD import/export features will be unavailable");
        spdlog::error("To enable OpenCASCADE support:");
        spdlog::error("  1. Install OpenCASCADE (version 7.6.0 or later recommended)");
        spdlog::error("  2. Set OpenCASCADE_DIR environment variable to installation path");
        spdlog::error("  3. Rebuild MECHATRON with -DMECHATRON_USE_OPENCASCADE=ON");
        spdlog::error("  4. Alternative: Linux: sudo apt-get install libopencascade-7-dev");
        spdlog::error("             macOS: brew install opencascade");
        spdlog::error("             Windows: Download from https://dev.opencascade.org/");
    }
#else
    spdlog::warn("OpenCASCADE support not enabled - CAD features limited");
    spdlog::warn("To enable full CAD support, rebuild with: cmake -DMECHATRON_USE_OPENCASCADE=ON ..");
#endif
}

OpenCascadeWrapper::~OpenCascadeWrapper() {
#ifdef MECHATRON_USE_OPENCASCADE
    // Cleanup OpenCASCADE resources
    if (m_last_shape) {
        auto* shape = static_cast<TopoDS_Shape*>(m_last_shape);
        delete shape;
        m_last_shape = nullptr;
    }
#endif
}

bool OpenCascadeWrapper::is_available() {
#ifdef MECHATRON_USE_OPENCASCADE
    return true;
#else
    return false;
#endif
}

bool OpenCascadeWrapper::initialize() {
#ifdef MECHATRON_USE_OPENCASCADE
    // Initialize OpenCASCADE subsystems
    // Modern OCCT doesn't require explicit initialization
    // Static initialization handles most things
    return true;
#else
    return false;
#endif
}

ImportResult OpenCascadeWrapper::import_step(const std::string& path) {
    ImportResult result;

#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::info("Importing STEP file: {}", path);

    // Check if file exists
    std::ifstream file(path);
    if (!file.good()) {
        result.success = false;
        result.error_message = "STEP file not found: " + path;
        spdlog::error(result.error_message);
        return result;
    }
    file.close();

    try {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());

        if (status != IFSelect_RetDone) {
            result.success = false;
            result.error_message = "Failed to read STEP file";
            spdlog::error(result.error_message);
            return result;
        }

        // Transfer roots
        reader.TransferRoots();
        int num_shapes = reader.NbShapes();
        result.num_shapes = num_shapes;

        if (num_shapes == 0) {
            result.success = false;
            result.error_message = "No shapes found in STEP file";
            spdlog::warn(result.error_message);
            return result;
        }

        // OneShape() returns a compound when multiple transferred roots exist,
        // which preserves complete STEP assemblies instead of importing only
        // the first shape.
        TopoDS_Shape shape = reader.OneShape();

        TopExp_Explorer faceExp(shape, TopAbs_FACE);
        result.num_faces = 0;
        while (faceExp.More()) {
            result.num_faces++;
            faceExp.Next();
        }

        TopExp_Explorer edgeExp(shape, TopAbs_EDGE);
        result.num_edges = 0;
        while (edgeExp.More()) {
            result.num_edges++;
            edgeExp.Next();
        }

        // Convert to mesh
        if (shape_to_mesh(&shape, result.mesh, 0.1f)) {
            result.success = true;
            spdlog::info("STEP import successful: {} vertices, {} triangles",
                        result.mesh.vertices.size(), result.mesh.triangles.size());
        } else {
            result.success = false;
            result.error_message = "Failed to triangulate STEP shape";
            spdlog::error(result.error_message);
        }

    } catch (const Standard_Failure& e) {
        result.success = false;
        result.error_message = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(result.error_message);
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = std::string("Exception: ") + e.what();
        spdlog::error(result.error_message);
    }

#else
    spdlog::error("STEP import failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    result.success = false;
    result.error_message = "OpenCASCADE support not enabled. Rebuild with -DMECHATRON_USE_OPENCASCADE=ON.";
    return result;
#endif

    return result;
}

ImportResult OpenCascadeWrapper::import_iges(const std::string& path) {
    ImportResult result;

#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::info("Importing IGES file: {}", path);

    // Check if file exists
    std::ifstream file(path);
    if (!file.good()) {
        result.success = false;
        result.error_message = "IGES file not found: " + path;
        spdlog::error(result.error_message);
        return result;
    }
    file.close();

    try {
        IGESControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());

        if (status != IFSelect_RetDone) {
            result.success = false;
            result.error_message = "Failed to read IGES file";
            spdlog::error(result.error_message);
            return result;
        }

        // Transfer roots
        reader.TransferRoots();
        int num_shapes = reader.NbShapes();
        result.num_shapes = num_shapes;

        if (num_shapes == 0) {
            result.success = false;
            result.error_message = "No shapes found in IGES file";
            spdlog::warn(result.error_message);
            return result;
        }

        // Get the shape
        TopoDS_Shape shape = reader.OneShape();

        // Count faces and edges
        TopExp_Explorer faceExp(shape, TopAbs_FACE);
        result.num_faces = 0;
        while (faceExp.More()) {
            result.num_faces++;
            faceExp.Next();
        }

        TopExp_Explorer edgeExp(shape, TopAbs_EDGE);
        result.num_edges = 0;
        while (edgeExp.More()) {
            result.num_edges++;
            edgeExp.Next();
        }

        // Convert to mesh
        if (shape_to_mesh(&shape, result.mesh, 0.1f)) {
            result.success = true;
            spdlog::info("IGES import successful: {} vertices, {} triangles",
                        result.mesh.vertices.size(), result.mesh.triangles.size());
        } else {
            result.success = false;
            result.error_message = "Failed to triangulate IGES shape";
            spdlog::error(result.error_message);
        }

    } catch (const Standard_Failure& e) {
        result.success = false;
        result.error_message = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(result.error_message);
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = std::string("Exception: ") + e.what();
        spdlog::error(result.error_message);
    }

#else
    spdlog::error("IGES import failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    result.success = false;
    result.error_message = "OpenCASCADE support not enabled. Rebuild with -DMECHATRON_USE_OPENCASCADE=ON.";
    return result;
#endif

    return result;
}

ImportResult OpenCascadeWrapper::import_brep(const std::string& path) {
    ImportResult result;

#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::info("Importing BREP file: {}", path);

    // Check if file exists
    std::ifstream file(path);
    if (!file.good()) {
        result.success = false;
        result.error_message = "BREP file not found: " + path;
        spdlog::error(result.error_message);
        return result;
    }
    file.close();

    try {
        BRep_Builder builder;
        TopoDS_Shape shape;

        if (!BRepTools::Read(shape, path.c_str(), builder)) {
            result.success = false;
            result.error_message = "Failed to read BREP file";
            spdlog::error(result.error_message);
            return result;
        }

        // Count faces and edges
        TopExp_Explorer faceExp(shape, TopAbs_FACE);
        result.num_faces = 0;
        while (faceExp.More()) {
            result.num_faces++;
            faceExp.Next();
        }

        // Convert to mesh
        if (shape_to_mesh(&shape, result.mesh, 0.1f)) {
            result.success = true;
            spdlog::info("BREP import successful: {} vertices, {} triangles",
                        result.mesh.vertices.size(), result.mesh.triangles.size());
        } else {
            result.success = false;
            result.error_message = "Failed to triangulate BREP shape";
            spdlog::error(result.error_message);
        }

    } catch (const Standard_Failure& e) {
        result.success = false;
        result.error_message = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(result.error_message);
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = std::string("Exception: ") + e.what();
        spdlog::error(result.error_message);
    }

#else
    spdlog::error("BREP import failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    result.success = false;
    result.error_message = "OpenCASCADE support not enabled. Rebuild with -DMECHATRON_USE_OPENCASCADE=ON.";
    return result;
#endif

    return result;
}

bool OpenCascadeWrapper::export_step(const std::string& path, const MeshData& mesh) {
#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::info("Exporting to STEP: {}", path);

    try {
        // Convert mesh to shape
        void* shape_ptr = mesh_to_shape(mesh);
        if (!shape_ptr) {
            m_error = "Failed to convert mesh to shape";
            return false;
        }

        auto* occt_shape = static_cast<TopoDS_Shape*>(shape_ptr);

        STEPControl_Writer writer;
        IFSelect_ReturnStatus status = writer.Transfer(*occt_shape, static_cast<STEPControl_StepModelType>(0));  // STEPControl_Assemble

        if (status != IFSelect_RetDone) {
            m_error = "Failed to transfer shape to STEP format";
            delete occt_shape;
            return false;
        }

        status = writer.Write(path.c_str());
        delete occt_shape;

        if (status != IFSelect_RetDone) {
            m_error = "Failed to write STEP file";
            return false;
        }

        spdlog::info("STEP export successful: {}", path);
        return true;

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    } catch (const std::exception& e) {
        m_error = std::string("Exception: ") + e.what();
        spdlog::error(m_error);
        return false;
    }

#else
    m_error = "OpenCASCADE not available";
    return false;
#endif
}

bool OpenCascadeWrapper::boolean_union(const MeshData& a, const MeshData& b, MeshData& result) {
#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::debug("Performing boolean union");

    try {
        // Convert meshes to shapes
        void* shape_a_ptr = mesh_to_shape(a);
        void* shape_b_ptr = mesh_to_shape(b);

        if (!shape_a_ptr || !shape_b_ptr) {
            m_error = "Failed to convert meshes to shapes";
            if (shape_a_ptr) delete static_cast<TopoDS_Shape*>(shape_a_ptr);
            if (shape_b_ptr) delete static_cast<TopoDS_Shape*>(shape_b_ptr);
            return false;
        }

        std::unique_ptr<TopoDS_Shape> shape_a(static_cast<TopoDS_Shape*>(shape_a_ptr));
        std::unique_ptr<TopoDS_Shape> shape_b(static_cast<TopoDS_Shape*>(shape_b_ptr));

        // Perform boolean union
        BRepAlgoAPI_Fuse fuse(*shape_a, *shape_b);
        fuse.Build();

        if (!fuse.IsDone()) {
            m_error = "Boolean fuse operation failed";
            return false;
        }

        TopoDS_Shape result_shape = fuse.Shape();

        // Convert back to mesh
        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean union successful: {} vertices", result.vertices.size());
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    } catch (const std::exception& e) {
        m_error = std::string("Exception: ") + e.what();
        spdlog::error(m_error);
        return false;
    }

#else
    spdlog::error("Boolean union failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    return false;
#endif
}

bool OpenCascadeWrapper::boolean_subtract(const MeshData& a, const MeshData& b, MeshData& result) {
#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::debug("Performing boolean subtract");

    try {
        void* shape_a_ptr = mesh_to_shape(a);
        void* shape_b_ptr = mesh_to_shape(b);

        if (!shape_a_ptr || !shape_b_ptr) {
            m_error = "Failed to convert meshes to shapes";
            if (shape_a_ptr) delete static_cast<TopoDS_Shape*>(shape_a_ptr);
            if (shape_b_ptr) delete static_cast<TopoDS_Shape*>(shape_b_ptr);
            return false;
        }

        std::unique_ptr<TopoDS_Shape> shape_a(static_cast<TopoDS_Shape*>(shape_a_ptr));
        std::unique_ptr<TopoDS_Shape> shape_b(static_cast<TopoDS_Shape*>(shape_b_ptr));

        BRepAlgoAPI_Cut cut(*shape_a, *shape_b);
        cut.Build();

        if (!cut.IsDone()) {
            m_error = "Boolean cut operation failed";
            return false;
        }

        TopoDS_Shape result_shape = cut.Shape();

        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean subtract successful: {} vertices", result.vertices.size());
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    } catch (const std::exception& e) {
        m_error = std::string("Exception: ") + e.what();
        spdlog::error(m_error);
        return false;
    }

#else
    spdlog::error("Boolean subtract failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    return false;
#endif
}

bool OpenCascadeWrapper::boolean_intersect(const MeshData& a, const MeshData& b, MeshData& result) {
#ifdef MECHATRON_USE_OPENCASCADE
    spdlog::debug("Performing boolean intersect");

    try {
        void* shape_a_ptr = mesh_to_shape(a);
        void* shape_b_ptr = mesh_to_shape(b);

        if (!shape_a_ptr || !shape_b_ptr) {
            m_error = "Failed to convert meshes to shapes";
            if (shape_a_ptr) delete static_cast<TopoDS_Shape*>(shape_a_ptr);
            if (shape_b_ptr) delete static_cast<TopoDS_Shape*>(shape_b_ptr);
            return false;
        }

        std::unique_ptr<TopoDS_Shape> shape_a(static_cast<TopoDS_Shape*>(shape_a_ptr));
        std::unique_ptr<TopoDS_Shape> shape_b(static_cast<TopoDS_Shape*>(shape_b_ptr));

        BRepAlgoAPI_Common common(*shape_a, *shape_b);
        common.Build();

        if (!common.IsDone()) {
            m_error = "Boolean common operation failed";
            return false;
        }

        TopoDS_Shape result_shape = common.Shape();

        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean intersect successful: {} vertices", result.vertices.size());
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    } catch (const std::exception& e) {
        m_error = std::string("Exception: ") + e.what();
        spdlog::error(m_error);
        return false;
    }

#else
    spdlog::error("Boolean intersect failed: OpenCASCADE support not enabled. Please rebuild with MECHATRON_USE_OPENCASCADE=ON.");
    return false;
#endif
}

bool OpenCascadeWrapper::mesh_simplification(MeshData& mesh, float target_ratio) {
#ifdef MECHATRON_USE_OPENCASCADE
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        m_error = "Empty mesh";
        return false;
    }

    if (target_ratio >= 1.0f) {
        return true;  // No simplification needed
    }

    // Calculate target vertex count
    size_t target_vertices = static_cast<size_t>(mesh.vertices.size() * target_ratio);
    if (target_vertices < 4) {
        m_error = "Target vertex count too small (minimum 4)";
        return false;
    }

    // Simple vertex clustering approach
    // Calculate mesh bounds
    Vec3 min_pos = mesh.vertices[0];
    Vec3 max_pos = mesh.vertices[0];
    for (const auto& v : mesh.vertices) {
        min_pos.x = std::min(min_pos.x, v.x);
        min_pos.y = std::min(min_pos.y, v.y);
        min_pos.z = std::min(min_pos.z, v.z);
        max_pos.x = std::max(max_pos.x, v.x);
        max_pos.y = std::max(max_pos.y, v.y);
        max_pos.z = std::max(max_pos.z, v.z);
    }

    Vec3 bounds = max_pos - min_pos;
    float max_bound = std::max({bounds.x, bounds.y, bounds.z});

    // Calculate grid cell size based on target reduction
    float cell_size = max_bound * std::cbrt(1.0f / target_vertices);

    // Create spatial hash grid for vertex clustering
    struct GridCell {
        std::vector<size_t> vertex_indices;
        Vec3 centroid;
        Vec3 normal;
    };

    std::unordered_map<size_t, GridCell> grid;

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& pos = mesh.vertices[i];

        // Calculate grid cell coordinates
        size_t gx = static_cast<size_t>((pos.x - min_pos.x) / cell_size);
        size_t gy = static_cast<size_t>((pos.y - min_pos.y) / cell_size);
        size_t gz = static_cast<size_t>((pos.z - min_pos.z) / cell_size);

        // Create hash key
        size_t hash = gx * 73856093 ^ gy * 19349663 ^ gz * 83492791;

        auto& cell = grid[hash];
        cell.vertex_indices.push_back(i);
        cell.centroid = cell.centroid + pos;
        if (i < mesh.normals.size()) {
            cell.normal = cell.normal + mesh.normals[i];
        }
    }

    // Create simplified mesh data
    std::vector<Vec3> simplified_vertices;
    std::vector<Vec3> simplified_normals;
    simplified_vertices.reserve(grid.size());
    simplified_normals.reserve(grid.size());

    std::unordered_map<size_t, size_t> old_to_new_vertex_map;

    for (auto& [hash, cell] : grid) {
        if (cell.vertex_indices.empty()) continue;

        // Average position and normal
        float divisor = static_cast<float>(cell.vertex_indices.size());
        Vec3 avg_pos = {cell.centroid.x / divisor, cell.centroid.y / divisor, cell.centroid.z / divisor};
        Vec3 avg_normal = cell.normal;
        float len = std::sqrt(avg_normal.x * avg_normal.x + avg_normal.y * avg_normal.y + avg_normal.z * avg_normal.z);
        if (len > 0.0001f) {
            float inv_len = 1.0f / len;
            avg_normal = {avg_normal.x * inv_len, avg_normal.y * inv_len, avg_normal.z * inv_len};
        }

        size_t new_idx = simplified_vertices.size();
        simplified_vertices.push_back(avg_pos);
        simplified_normals.push_back(avg_normal);

        // Map old vertices to new vertex
        for (size_t old_idx : cell.vertex_indices) {
            old_to_new_vertex_map[old_idx] = new_idx;
        }
    }

    // Rebuild triangles
    std::vector<Triangle> simplified_triangles;
    simplified_triangles.reserve(mesh.triangles.size());

    for (const auto& tri : mesh.triangles) {
        auto it0 = old_to_new_vertex_map.find(tri.v0);
        auto it1 = old_to_new_vertex_map.find(tri.v1);
        auto it2 = old_to_new_vertex_map.find(tri.v2);

        if (it0 != old_to_new_vertex_map.end() &&
            it1 != old_to_new_vertex_map.end() &&
            it2 != old_to_new_vertex_map.end()) {
            // Only add triangle if it has 3 distinct vertices
            if (it0->second != it1->second &&
                it1->second != it2->second &&
                it0->second != it2->second) {
                simplified_triangles.push_back({static_cast<uint32_t>(it0->second),
                                                  static_cast<uint32_t>(it1->second),
                                                  static_cast<uint32_t>(it2->second)});
            }
        }
    }

    // Update mesh with simplified data
    mesh.vertices = std::move(simplified_vertices);
    mesh.normals = std::move(simplified_normals);
    mesh.triangles = std::move(simplified_triangles);

    // Clear tex_coords as they're no longer valid
    mesh.tex_coords.clear();

    spdlog::info("[OpenCascade] Mesh simplified: {} -> {} vertices (target ratio: {:.2f})",
                 old_to_new_vertex_map.size(), mesh.vertices.size(), target_ratio);

    return true;
#else
    (void)mesh;
    (void)target_ratio;
    m_error = "OpenCASCADE support not enabled. Rebuild with -DMECHATRON_USE_OPENCASCADE=ON.";
    return false;
#endif
}

bool OpenCascadeWrapper::mesh_refinement(MeshData& mesh, float max_edge_length) {
#ifdef MECHATRON_USE_OPENCASCADE
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        m_error = "Empty mesh";
        return false;
    }
    if (max_edge_length <= 0.0f) {
        m_error = "max_edge_length must be positive";
        return false;
    }

    struct EdgeKey {
        uint32_t v0, v1;
        EdgeKey(uint32_t a, uint32_t b) : v0(std::min(a, b)), v1(std::max(a, b)) {}
        bool operator==(const EdgeKey& o) const { return v0 == o.v0 && v1 == o.v1; }
        struct Hash {
            size_t operator()(const EdgeKey& k) const {
                return (static_cast<size_t>(k.v0) * 2654435761u) ^ static_cast<size_t>(k.v1);
            }
        };
    };

    auto edge_length = [&](uint32_t a, uint32_t b) -> float {
        if (a >= mesh.vertices.size() || b >= mesh.vertices.size()) return 0.0f;
        return (mesh.vertices[a] - mesh.vertices[b]).length();
    };

    constexpr int max_iterations = 8;
    bool refined_any = false;
    for (int iter = 0; iter < max_iterations; ++iter) {
        MeshData refined;
        refined.vertices = mesh.vertices;
        std::unordered_map<EdgeKey, uint32_t, EdgeKey::Hash> midpoints;

        auto midpoint = [&](uint32_t a, uint32_t b) -> uint32_t {
            EdgeKey key(a, b);
            auto it = midpoints.find(key);
            if (it != midpoints.end()) return it->second;
            Vec3 mid = (mesh.vertices[a] + mesh.vertices[b]) * 0.5f;
            uint32_t idx = static_cast<uint32_t>(refined.vertices.size());
            refined.vertices.push_back(mid);
            midpoints.emplace(key, idx);
            return idx;
        };

        bool changed = false;
        for (const auto& tri : mesh.triangles) {
            if (tri.v0 >= mesh.vertices.size() || tri.v1 >= mesh.vertices.size() || tri.v2 >= mesh.vertices.size()) {
                continue;
            }

            const bool split_ab = edge_length(tri.v0, tri.v1) > max_edge_length;
            const bool split_bc = edge_length(tri.v1, tri.v2) > max_edge_length;
            const bool split_ca = edge_length(tri.v2, tri.v0) > max_edge_length;
            const int split_count = (split_ab ? 1 : 0) + (split_bc ? 1 : 0) + (split_ca ? 1 : 0);

            if (split_count == 0) {
                refined.triangles.push_back(tri);
                continue;
            }

            changed = true;
            refined_any = true;
            const uint32_t a = tri.v0;
            const uint32_t b = tri.v1;
            const uint32_t c = tri.v2;

            if (split_count == 3) {
                uint32_t ab = midpoint(a, b);
                uint32_t bc = midpoint(b, c);
                uint32_t ca = midpoint(c, a);
                refined.triangles.push_back({a, ab, ca});
                refined.triangles.push_back({ab, b, bc});
                refined.triangles.push_back({ca, bc, c});
                refined.triangles.push_back({ab, bc, ca});
            } else if (split_count == 1) {
                if (split_ab) {
                    uint32_t ab = midpoint(a, b);
                    refined.triangles.push_back({a, ab, c});
                    refined.triangles.push_back({ab, b, c});
                } else if (split_bc) {
                    uint32_t bc = midpoint(b, c);
                    refined.triangles.push_back({b, bc, a});
                    refined.triangles.push_back({bc, c, a});
                } else {
                    uint32_t ca = midpoint(c, a);
                    refined.triangles.push_back({c, ca, b});
                    refined.triangles.push_back({ca, a, b});
                }
            } else {
                if (!split_ab) {
                    uint32_t bc = midpoint(b, c);
                    uint32_t ca = midpoint(c, a);
                    refined.triangles.push_back({a, b, ca});
                    refined.triangles.push_back({b, bc, ca});
                    refined.triangles.push_back({bc, c, ca});
                } else if (!split_bc) {
                    uint32_t ab = midpoint(a, b);
                    uint32_t ca = midpoint(c, a);
                    refined.triangles.push_back({b, c, ab});
                    refined.triangles.push_back({c, ca, ab});
                    refined.triangles.push_back({ca, a, ab});
                } else {
                    uint32_t ab = midpoint(a, b);
                    uint32_t bc = midpoint(b, c);
                    refined.triangles.push_back({c, a, bc});
                    refined.triangles.push_back({a, ab, bc});
                    refined.triangles.push_back({ab, b, bc});
                }
            }
        }

        if (!changed) break;
        mesh = std::move(refined);
    }

    mesh.normals.clear();
    mesh.calculate_normals();
    return refined_any;
#else
    (void)max_edge_length;
    return false;
#endif
}

bool OpenCascadeWrapper::shape_to_mesh(const void* occt_shape, MeshData& mesh, float linear_deflection) {
#ifdef MECHATRON_USE_OPENCASCADE
    if (!occt_shape) {
        return false;
    }

    try {
        const auto* shape = static_cast<const TopoDS_Shape*>(occt_shape);

        // Triangulate the shape
        BRepMesh_IncrementalMesh meshizer(*shape, linear_deflection);
        meshizer.Perform();

        if (!meshizer.IsDone()) {
            return false;
        }

        // Clear existing mesh data
        mesh.clear();

        // Extract triangles from all faces
        TopExp_Explorer faceExp(*shape, TopAbs_FACE);

        while (faceExp.More()) {
            const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
            TopLoc_Location location;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

            if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
                faceExp.Next();
                continue;
            }

            const gp_Trsf& transform = location.Transformation();
            int num_triangles = triangulation->NbTriangles();
            int num_nodes = triangulation->NbNodes();

            // Get triangle indices
            int index_offset = mesh.vertices.size();

            for (int i = 1; i <= num_nodes; ++i) {
                gp_XYZ point = triangulation->Node(i).XYZ();
                transform.Transforms(point);

                mesh.vertices.push_back(Vec3{
                    static_cast<float>(point.X()),
                    static_cast<float>(point.Y()),
                    static_cast<float>(point.Z())
                });
            }

            for (int i = 1; i <= num_triangles; ++i) {
                Poly_Triangle triangle = triangulation->Triangle(i);
                int n1, n2, n3;
                triangle.Get(n1, n2, n3);

                // Check face orientation
                if (face.Orientation() == TopAbs_REVERSED) {
                    std::swap(n1, n3);
                }

                mesh.triangles.push_back({static_cast<uint32_t>(index_offset + n1 - 1),
                                          static_cast<uint32_t>(index_offset + n2 - 1),
                                          static_cast<uint32_t>(index_offset + n3 - 1)});
            }

            faceExp.Next();
        }

        if (mesh.vertices.empty() || mesh.triangles.empty()) {
            return false;
        }

        mesh.unify_vertices();
        mesh.calculate_normals();
        return true;

    } catch (const Standard_Failure& e) {
        spdlog::error("OpenCASCADE exception in shape_to_mesh: {}", e.GetMessageString());
        return false;
    }
#else
    (void)occt_shape;
    (void)linear_deflection;
    return false;
#endif
}

void* OpenCascadeWrapper::mesh_to_shape(const MeshData& mesh) {
#ifdef MECHATRON_USE_OPENCASCADE
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        spdlog::warn("Cannot convert empty mesh to shape");
        return nullptr;
    }

    try {
        // Build topology from mesh triangles
        // Create faces from triangles and combine into a shell/solid

        BRepOffsetAPI_Sewing sewer;
        sewer.SetTolerance(0.001);

        std::vector<TopoDS_Face> faces;
        faces.reserve(mesh.triangles.size());

        for (const auto& tri : mesh.triangles) {
            if (tri.v0 >= mesh.vertices.size() ||
                tri.v1 >= mesh.vertices.size() ||
                tri.v2 >= mesh.vertices.size()) {
                continue;
            }

            const Vec3& v0 = mesh.vertices[tri.v0];
            const Vec3& v1 = mesh.vertices[tri.v1];
            const Vec3& v2 = mesh.vertices[tri.v2];

            // Create triangle polygon
            BRepBuilderAPI_MakePolygon poly_maker;
            poly_maker.Add(gp_Pnt(v0.x, v0.y, v0.z));
            poly_maker.Add(gp_Pnt(v1.x, v1.y, v1.z));
            poly_maker.Add(gp_Pnt(v2.x, v2.y, v2.z));
            poly_maker.Close();

            if (!poly_maker.IsDone()) {
                continue;
            }

            // Create face from polygon
            BRepBuilderAPI_MakeFace face_maker(poly_maker.Wire());
            if (!face_maker.IsDone()) {
                continue;
            }

            TopoDS_Face face = face_maker.Face();
            faces.push_back(face);
            sewer.Add(face);
        }

        if (faces.empty()) {
            spdlog::warn("No valid faces created from mesh");
            return nullptr;
        }

        // Sew faces together
        sewer.Perform();

        // Get the sewed shape
        TopoDS_Shape sewed_shape = sewer.SewedShape();

        if (sewed_shape.IsNull()) {
            // If sewing failed, return compound of faces
            BRep_Builder builder;
            TopoDS_Compound compound;
            builder.MakeCompound(compound);

            for (const auto& face : faces) {
                builder.Add(compound, face);
            }

            return new TopoDS_Shape(compound);
        }

        if (sewed_shape.ShapeType() == TopAbs_SHELL) {
            TopoDS_Shell shell = TopoDS::Shell(sewed_shape);
            BRepBuilderAPI_MakeSolid solid_maker(shell);
            if (solid_maker.IsDone()) {
                return new TopoDS_Shape(solid_maker.Solid());
            }
        }

        return new TopoDS_Shape(sewed_shape);

    } catch (const Standard_Failure& e) {
        spdlog::error("OpenCASCADE exception in mesh_to_shape: {}", e.GetMessageString());
        return nullptr;
    } catch (const std::exception& e) {
        spdlog::error("Exception in mesh_to_shape: {}", e.what());
        return nullptr;
    }
#else
    (void)mesh;
    return nullptr;
#endif
}

// ============================================================================
// Fallback Implementation (when OpenCASCADE is not available)
// ============================================================================

ImportResult OpenCascadeFallback::import_step(const std::string& path) {
    ImportResult result;
    result.success = false;
    result.error_message = "STEP import requires OpenCASCADE. Please install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::error(result.error_message);
    spdlog::error("To install OpenCASCADE:");
    spdlog::error("  Windows: Download from https://dev.opencascade.org/");
    spdlog::error("  Linux: sudo apt-get install libopencascade-occt-dev");
    spdlog::error("  macOS: brew install opencascade");
    return result;
}

ImportResult OpenCascadeFallback::import_iges(const std::string& path) {
    ImportResult result;
    result.success = false;
    result.error_message = "IGES import requires OpenCASCADE. Please install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::error(result.error_message);
    return result;
}

ImportResult OpenCascadeFallback::import_brep(const std::string& path) {
    ImportResult result;
    result.success = false;
    result.error_message = "BREP import requires OpenCASCADE. Please install OpenCASCADE and rebuild with MECHATRON_USE_OPENCASCADE=ON";
    spdlog::error(result.error_message);
    return result;
}

bool OpenCascadeFallback::boolean_union(const MeshData& a, const MeshData& b, MeshData& result) {
    (void)a; (void)b; (void)result;
    spdlog::error("Boolean union requires OpenCASCADE");
    return false;
}

bool OpenCascadeFallback::boolean_subtract(const MeshData& a, const MeshData& b, MeshData& result) {
    (void)a; (void)b; (void)result;
    spdlog::error("Boolean subtract requires OpenCASCADE");
    return false;
}

bool OpenCascadeFallback::boolean_intersect(const MeshData& a, const MeshData& b, MeshData& result) {
    (void)a; (void)b; (void)result;
    spdlog::error("Boolean intersect requires OpenCASCADE");
    return false;
}

} // namespace mechatron
