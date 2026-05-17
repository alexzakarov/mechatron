#include <gtest/gtest.h>

#include "cad/CADKernel.hpp"
#include "cad/EditableMesh.hpp"
#include "cad/ModelAssetLibrary.hpp"
#include "cad/ModifierEngine.hpp"
#include "cad/Modifiers.hpp"
#include "cad/OpenCascadeWrapper.hpp"

using namespace mechatron;

namespace {

void expect_valid_mesh(const MeshData& mesh) {
    ASSERT_FALSE(mesh.vertices.empty());
    ASSERT_FALSE(mesh.triangles.empty());
    for (const auto& tri : mesh.triangles) {
        EXPECT_LT(tri.v0, mesh.vertices.size());
        EXPECT_LT(tri.v1, mesh.vertices.size());
        EXPECT_LT(tri.v2, mesh.vertices.size());
    }
    EXPECT_EQ(mesh.normals.size(), mesh.vertices.size());
}

} // namespace

class ModelAssetLibraryTest : public ::testing::Test {
protected:
    void TearDown() override {
        ModelAssetLibrary lib;
        lib.delete_user_asset("__cad_test_asset__");
        lib.delete_user_asset("__cad_empty_asset__");
        lib.delete_user_asset("__cad_invalid_asset__");
    }
};

TEST(CADModifiers, SubdivisionProducesValidTriangleMesh) {
    CADKernel cad;
    ModifierEngine engine;

    auto box = cad.create_box(1.0f, 1.0f, 1.0f);
    MeshData out;
    ModifierStack stack;
    stack.modifiers.push_back(ModifierSubdivision{1});

    std::string error;
    ASSERT_TRUE(engine.apply(cad, box->mesh, stack,
        [](const std::string&, const std::string&, MeshData&) { return false; },
        out, &error)) << error;

    EXPECT_EQ(out.triangles.size(), box->mesh.triangles.size() * 4);
    EXPECT_GT(out.vertices.size(), box->mesh.vertices.size());
    expect_valid_mesh(out);
}

TEST(CADModifiers, MirrorProducesValidTriangleMesh) {
    CADKernel cad;
    ModifierEngine engine;

    auto box = cad.create_box(1.0f, 1.0f, 1.0f);
    MeshData out;
    ModifierStack stack;
    ModifierMirror mirror;
    mirror.x = true;
    mirror.y = false;
    mirror.z = false;
    stack.modifiers.push_back(mirror);

    std::string error;
    ASSERT_TRUE(engine.apply(cad, box->mesh, stack,
        [](const std::string&, const std::string&, MeshData&) { return false; },
        out, &error)) << error;

    EXPECT_EQ(out.vertices.size(), box->mesh.vertices.size() * 2);
    EXPECT_EQ(out.triangles.size(), box->mesh.triangles.size() * 2);
    expect_valid_mesh(out);
}

TEST(CADModifiers, WeldMergesDuplicateVerticesAndDropsDegenerateFaces) {
    CADKernel cad;
    ModifierEngine engine;

    MeshData mesh;
    mesh.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0.00001f, 0, 0}
    };
    mesh.triangles = {
        {0, 1, 2},
        {0, 3, 1}
    };
    mesh.calculate_normals();

    ModifierStack stack;
    stack.modifiers.push_back(ModifierWeld{0.001f});

    MeshData out;
    std::string error;
    ASSERT_TRUE(engine.apply(cad, mesh, stack,
        [](const std::string&, const std::string&, MeshData&) { return false; },
        out, &error)) << error;

    EXPECT_EQ(out.vertices.size(), 3u);
    ASSERT_EQ(out.triangles.size(), 1u);
    expect_valid_mesh(out);
}

TEST(CADModifiers, ArrayDuplicatesMeshWithOffset) {
    CADKernel cad;
    ModifierEngine engine;

    auto box = cad.create_box(1.0f, 1.0f, 1.0f);
    ModifierStack stack;
    ModifierArray array;
    array.count = 3;
    array.offset = {2.0f, 0.0f, 0.0f};
    stack.modifiers.push_back(array);

    MeshData out;
    std::string error;
    ASSERT_TRUE(engine.apply(cad, box->mesh, stack,
        [](const std::string&, const std::string&, MeshData&) { return false; },
        out, &error)) << error;

    EXPECT_EQ(out.vertices.size(), box->mesh.vertices.size() * 3);
    EXPECT_EQ(out.triangles.size(), box->mesh.triangles.size() * 3);
    expect_valid_mesh(out);

    Vec3 min, max;
    out.get_bounds(min, max);
    EXPECT_GT(max.x - min.x, 4.5f);
}

TEST(CADModifiers, SolidifyAddsThicknessToOpenMesh) {
    CADKernel cad;
    ModifierEngine engine;

    MeshData tri;
    tri.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0}
    };
    tri.triangles = {{0, 1, 2}};
    tri.calculate_normals();

    ModifierStack stack;
    stack.modifiers.push_back(ModifierSolidify{0.1f, 0.0f});

    MeshData out;
    std::string error;
    ASSERT_TRUE(engine.apply(cad, tri, stack,
        [](const std::string&, const std::string&, MeshData&) { return false; },
        out, &error)) << error;

    EXPECT_EQ(out.vertices.size(), 6u);
    EXPECT_EQ(out.triangles.size(), 8u);
    expect_valid_mesh(out);
}

TEST(CADKernel, SmoothMeshKeepsValidTopology) {
    CADKernel cad;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);
    const size_t vertex_count = box->mesh.vertices.size();
    const size_t triangle_count = box->mesh.triangles.size();

    cad.smooth_mesh(box->mesh, 2);

    EXPECT_EQ(box->mesh.vertices.size(), vertex_count);
    EXPECT_EQ(box->mesh.triangles.size(), triangle_count);
    expect_valid_mesh(box->mesh);
}

TEST(CADKernel, UnifyVerticesDropsInvalidAndDegenerateTriangles) {
    MeshData mesh;
    mesh.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 0}
    };
    mesh.triangles = {
        {0, 1, 2},
        {0, 0, 1},
        {0, 1, 9},
        {0, 3, 1}
    };

    mesh.unify_vertices();
    mesh.calculate_normals();

    ASSERT_EQ(mesh.triangles.size(), 1u);
    expect_valid_mesh(mesh);
}

TEST(EditableMesh, InsetFacesKeepsValidTopology) {
    CADKernel cad;
    EditableMesh editable;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);

    ASSERT_TRUE(editable.from_meshdata(box->mesh));
    auto created_faces = editable.inset_faces({0}, 0.25f, 0.05f);

    EXPECT_FALSE(created_faces.empty());
    MeshData mesh = editable.to_meshdata();
    expect_valid_mesh(mesh);
}

TEST(EditableMesh, PolygonSelectUsesActiveSelectionMode) {
    CADKernel cad;
    EditableMesh editable;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);

    ASSERT_TRUE(editable.from_meshdata(box->mesh));
    editable.set_select_mode(SelectMode::Vertex);
    std::vector<std::array<float, 2>> polygon = {
        {-1.0f, -1.0f},
        {0.1f, -1.0f},
        {0.1f, 0.1f},
        {-1.0f, 0.1f}
    };
    editable.polygon_select(polygon, SelectionOp::Replace,
        [](const Vec3& v, float& sx, float& sy) {
            sx = v.x;
            sy = v.y;
            return true;
        });

    EXPECT_FALSE(editable.selected_vertices().empty());
    for (uint32_t idx : editable.selected_vertices()) {
        ASSERT_LT(idx, editable.vertices().size());
        EXPECT_LE(editable.vertices()[idx].x, 0.1f);
        EXPECT_LE(editable.vertices()[idx].y, 0.1f);
    }
}

TEST(EditableMesh, RejectsMeshDataWithoutValidTriangles) {
    EditableMesh editable;
    MeshData invalid;
    invalid.vertices = {
        {0, 0, 0},
        {1, 0, 0}
    };
    invalid.triangles = {
        {0, 1, 9},
        {0, 0, 1}
    };

    EXPECT_FALSE(editable.from_meshdata(invalid));
    EXPECT_EQ(editable.vertex_count(), 0u);
    EXPECT_EQ(editable.triangle_count(), 0u);
}

TEST(EditableMesh, DeleteLooseVerticesCompactsMesh) {
    EditableMesh editable;
    MeshData mesh;
    mesh.vertices = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {5, 5, 5}
    };
    mesh.triangles = {{0, 1, 2}};

    ASSERT_TRUE(editable.from_meshdata(mesh));
    ASSERT_EQ(editable.vertex_count(), 4u);
    editable.delete_loose_vertices();

    EXPECT_EQ(editable.vertex_count(), 3u);
    EXPECT_EQ(editable.triangle_count(), 1u);
    expect_valid_mesh(editable.to_meshdata());
}

TEST(OpenCascadeWrapper, MeshRefinementSplitsLongEdges) {
    if (!OpenCascadeWrapper::is_available()) {
        GTEST_SKIP() << "OpenCASCADE support is not available in this build.";
    }

    CADKernel cad;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);
    const size_t original_vertices = box->mesh.vertices.size();

    OpenCascadeWrapper wrapper;
    ASSERT_TRUE(wrapper.mesh_refinement(box->mesh, 0.4f)) << wrapper.error();
    EXPECT_GT(box->mesh.vertices.size(), original_vertices);
    expect_valid_mesh(box->mesh);
}

TEST(CADModifiers, BooleanUnionProducesValidMeshWhenOpenCascadeIsAvailable) {
    CADKernel cad;
    if (!cad.has_opencascade_support()) {
        GTEST_SKIP() << "OpenCASCADE support is not available in this build.";
    }

    ModifierEngine engine;
    auto a = cad.create_box(1.0f, 1.0f, 1.0f);
    auto b = cad.create_box(1.0f, 1.0f, 1.0f);
    for (auto& v : b->mesh.vertices) {
        v.x += 0.25f;
    }
    b->mesh.calculate_normals();

    ModifierBoolean boolean;
    boolean.op = ModifierBoolean::Op::Union;
    boolean.other_asset_id = "operand";
    boolean.other_scope = "user";

    ModifierStack stack;
    stack.modifiers.push_back(boolean);

    MeshData out;
    std::string error;
    ASSERT_TRUE(engine.apply(cad, a->mesh, stack,
        [&](const std::string& asset_id, const std::string&, MeshData& mesh) {
            if (asset_id != "operand") return false;
            mesh = b->mesh;
            return true;
        },
        out, &error)) << error;

    expect_valid_mesh(out);
}

TEST_F(ModelAssetLibraryTest, SaveLoadUserAssetRoundTrip) {
    CADKernel cad;
    ModelAssetLibrary lib;

    auto box = cad.create_box(0.25f, 0.5f, 0.75f);
    ModelAssetMeta meta;
    meta.id = "__cad_test_asset__";
    meta.scope = "user";
    meta.label = "CAD Test Asset";
    meta.format = "stl";
    meta.modifiers.modifiers.push_back(ModifierWeld{0.001f});
    meta.modifiers.modifiers.push_back(ModifierArray{3, Vec3{0.5f, 0.0f, 0.0f}});
    meta.modifiers.modifiers.push_back(ModifierSolidify{0.05f, 0.0f});

    std::string error;
    ASSERT_TRUE(lib.save_mesh_as_asset(cad, box->mesh, meta.id, "user", meta, &error)) << error;
    EXPECT_TRUE(lib.asset_exists(meta.id, "user"));

    ModelAssetMeta loaded_meta;
    ASSERT_TRUE(lib.load_asset_meta(meta.id, "user", loaded_meta, &error)) << error;
    EXPECT_EQ(loaded_meta.id, meta.id);
    EXPECT_EQ(loaded_meta.scope, "user");
    ASSERT_EQ(loaded_meta.modifiers.modifiers.size(), 3u);
    EXPECT_TRUE(std::holds_alternative<ModifierWeld>(loaded_meta.modifiers.modifiers[0]));
    EXPECT_TRUE(std::holds_alternative<ModifierArray>(loaded_meta.modifiers.modifiers[1]));
    EXPECT_TRUE(std::holds_alternative<ModifierSolidify>(loaded_meta.modifiers.modifiers[2]));

    MeshData loaded_mesh;
    ASSERT_TRUE(lib.load_asset_mesh(cad, meta.id, "user", loaded_mesh, &error)) << error;
    expect_valid_mesh(loaded_mesh);
}

TEST_F(ModelAssetLibraryTest, RejectsUnsafeAssetIds) {
    CADKernel cad;
    ModelAssetLibrary lib;

    auto box = cad.create_box(0.25f, 0.25f, 0.25f);
    ModelAssetMeta meta;
    meta.id = "../bad";
    meta.scope = "user";
    meta.label = "Bad";
    meta.format = "stl";

    std::string error;
    EXPECT_FALSE(lib.save_mesh_as_asset(cad, box->mesh, meta.id, "user", meta, &error));
    EXPECT_FALSE(error.empty());
}

TEST_F(ModelAssetLibraryTest, RejectsEmptyAndInvalidMeshes) {
    CADKernel cad;
    ModelAssetLibrary lib;

    ModelAssetMeta meta;
    meta.id = "__cad_empty_asset__";
    meta.scope = "user";
    meta.label = "Empty";
    meta.format = "stl";

    MeshData empty;
    std::string error;
    EXPECT_FALSE(lib.save_mesh_as_asset(cad, empty, meta.id, "user", meta, &error));
    EXPECT_FALSE(error.empty());

    MeshData invalid;
    invalid.vertices.push_back({0, 0, 0});
    invalid.vertices.push_back({1, 0, 0});
    invalid.triangles.push_back(Triangle{0, 1, 2});
    meta.id = "__cad_invalid_asset__";
    error.clear();
    EXPECT_FALSE(lib.save_mesh_as_asset(cad, invalid, meta.id, "user", meta, &error));
    EXPECT_FALSE(error.empty());
}
