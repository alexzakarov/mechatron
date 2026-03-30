#include "OpenCascadeWrapper.hpp"
#include <spdlog/spdlog.h>
#include <fstream>

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
        spdlog::warn("OpenCASCADE initialization failed, using fallback");
    }
#else
    spdlog::info("OpenCASCADE support not enabled (compile with MECHATRON_USE_OPENCASCADE)");
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

        // Get the first shape (or combine all)
        TopoDS_Shape shape;
        if (num_shapes == 1) {
            shape = reader.OneShape();
        } else {
            // For multiple shapes, use the first one for now
            shape = reader.Shape(1);
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
    spdlog::info("OpenCASCADE not available, using fallback for STEP import");
    result = OpenCascadeFallback::import_step(path);
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
    spdlog::info("OpenCASCADE not available, using fallback for IGES import");
    result = OpenCascadeFallback::import_iges(path);
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
    spdlog::info("OpenCASCADE not available, using fallback for BREP import");
    result = OpenCascadeFallback::import_brep(path);
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

        auto* shape_a = static_cast<TopoDS_Shape*>(shape_a_ptr);
        auto* shape_b = static_cast<TopoDS_Shape*>(shape_b_ptr);

        // Perform boolean union
        BRepAlgoAPI_Fuse fuse(*shape_a, *shape_b);
        fuse.Build();

        if (!fuse.IsDone()) {
            m_error = "Boolean fuse operation failed";
            delete shape_a;
            delete shape_b;
            return false;
        }

        TopoDS_Shape result_shape = fuse.Shape();

        // Convert back to mesh
        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean union successful: {} vertices", result.vertices.size());
            delete shape_a;
            delete shape_b;
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            delete shape_a;
            delete shape_b;
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    }

#else
    return OpenCascadeFallback::boolean_union(a, b, result);
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

        auto* shape_a = static_cast<TopoDS_Shape*>(shape_a_ptr);
        auto* shape_b = static_cast<TopoDS_Shape*>(shape_b_ptr);

        BRepAlgoAPI_Cut cut(*shape_a, *shape_b);
        cut.Build();

        if (!cut.IsDone()) {
            m_error = "Boolean cut operation failed";
            delete shape_a;
            delete shape_b;
            return false;
        }

        TopoDS_Shape result_shape = cut.Shape();

        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean subtract successful: {} vertices", result.vertices.size());
            delete shape_a;
            delete shape_b;
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            delete shape_a;
            delete shape_b;
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    }

#else
    return OpenCascadeFallback::boolean_subtract(a, b, result);
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

        auto* shape_a = static_cast<TopoDS_Shape*>(shape_a_ptr);
        auto* shape_b = static_cast<TopoDS_Shape*>(shape_b_ptr);

        BRepAlgoAPI_Common common(*shape_a, *shape_b);
        common.Build();

        if (!common.IsDone()) {
            m_error = "Boolean common operation failed";
            delete shape_a;
            delete shape_b;
            return false;
        }

        TopoDS_Shape result_shape = common.Shape();

        if (shape_to_mesh(&result_shape, result, 0.1f)) {
            spdlog::debug("Boolean intersect successful: {} vertices", result.vertices.size());
            delete shape_a;
            delete shape_b;
            return true;
        } else {
            m_error = "Failed to convert result to mesh";
            delete shape_a;
            delete shape_b;
            return false;
        }

    } catch (const Standard_Failure& e) {
        m_error = std::string("OpenCASCADE exception: ") + e.GetMessageString();
        spdlog::error(m_error);
        return false;
    }

#else
    return OpenCascadeFallback::boolean_intersect(a, b, result);
#endif
}

bool OpenCascadeWrapper::mesh_simplification(MeshData& mesh, float target_ratio) {
#ifdef MECHATRON_USE_OPENCASCADE
    // Use OpenCASCADE mesh simplification algorithms
    // This is a placeholder - would require MEShVS or decimation algorithms
    (void)target_ratio;
    m_error = "Mesh simplification not yet implemented";
    return false;
#else
    (void)target_ratio;
    return false;
#endif
}

bool OpenCascadeWrapper::mesh_refinement(MeshData& mesh, float max_edge_length) {
#ifdef MECHATRON_USE_OPENCASCADE
    // Use OpenCASCADE mesh refinement algorithms
    (void)max_edge_length;
    m_error = "Mesh refinement not yet implemented";
    return false;
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
        mesh.vertices.clear();
        mesh.triangles.clear();

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

        return !mesh.vertices.empty() && !mesh.triangles.empty();

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
