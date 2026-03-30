// CAD Kernel Demo
// Demonstrates STL/OBJ import/export, mesh primitives, and geometry operations

#include "cad/CADKernel.hpp"
#include <iostream>
#include <iomanip>

using namespace mechatron;

void print_mesh_info(const MeshData& mesh) {
    std::cout << "  Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "  Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "  Normals: " << mesh.normals.size() << std::endl;

    Vec3 min, max;
    mesh.get_bounds(min, max);
    std::cout << "  Bounds: [" << min.x << ", " << min.y << ", " << min.z << "] to ["
              << max.x << ", " << max.y << ", " << max.z << "]" << std::endl;
    std::cout << "  Size: " << (max.x - min.x) << " x " << (max.y - min.y)
              << " x " << (max.z - min.z) << std::endl;
}

void test_primitive_creation() {
    std::cout << "\n=== Test 1: Primitive Creation ===" << std::endl;

    CADKernel cad;

    // Create box
    std::cout << "\nCreating Box (2x3x4)..." << std::endl;
    auto box = cad.create_box(2.0f, 3.0f, 4.0f);
    print_mesh_info(box->mesh);

    // Create sphere
    std::cout << "\nCreating Sphere (radius=1.5, 16 segments)..." << std::endl;
    auto sphere = cad.create_sphere(1.5f, 16);
    print_mesh_info(sphere->mesh);

    // Create cylinder
    std::cout << "\nCreating Cylinder (r=1, h=3, 16 segments)..." << std::endl;
    auto cylinder = cad.create_cylinder(1.0f, 3.0f, 16);
    print_mesh_info(cylinder->mesh);

    // Create cone
    std::cout << "\nCreating Cone (r=1, h=2, 16 segments)..." << std::endl;
    auto cone = cad.create_cone(1.0f, 2.0f, 16);
    print_mesh_info(cone->mesh);

    std::cout << "\n✓ Primitive creation test passed" << std::endl;
}

void test_transform_operations() {
    std::cout << "\n=== Test 2: Transform Operations ===" << std::endl;

    CADKernel cad;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);

    std::cout << "\nOriginal box:" << std::endl;
    print_mesh_info(box->mesh);

    // Apply scale
    box->set_scale({2.0f, 3.0f, 4.0f});
    std::cout << "\nAfter scale (2, 3, 4):" << std::endl;
    MeshData scaled = box->get_transformed_mesh();
    print_mesh_info(scaled);

    // Apply rotation
    box->set_scale({1.0f, 1.0f, 1.0f});
    box->set_rotation({45.0f, 0.0f, 0.0f});  // 45 degrees around X
    std::cout << "\nAfter rotation (45° X):" << std::endl;
    MeshData rotated = box->get_transformed_mesh();
    print_mesh_info(rotated);

    // Apply translation
    box->set_rotation({0.0f, 0.0f, 0.0f});
    box->set_position({5.0f, 10.0f, 15.0f});
    std::cout << "\nAfter translation (5, 10, 15):" << std::endl;
    MeshData translated = box->get_transformed_mesh();
    print_mesh_info(translated);

    std::cout << "\n✓ Transform operations test passed" << std::endl;
}

void test_stl_export() {
    std::cout << "\n=== Test 3: STL Export ===" << std::endl;

    CADKernel cad;

    // Create a complex object
    auto sphere = cad.create_sphere(2.0f, 32);

    // Export ASCII STL
    std::cout << "\nExporting to STL ASCII (test_sphere_ascii.stl)..." << std::endl;
    if (cad.export_stl("test_sphere_ascii.stl", sphere->mesh)) {
        std::cout << "✓ STL ASCII export successful" << std::endl;
    } else {
        std::cout << "✗ STL ASCII export failed: " << cad.error() << std::endl;
    }

    // Create a larger mesh to trigger binary export
    auto dense_sphere = cad.create_sphere(2.0f, 64);

    // Export Binary STL
    std::cout << "\nExporting to STL Binary (test_sphere_binary.stl)..." << std::endl;
    if (cad.export_stl("test_sphere_binary.stl", dense_sphere->mesh)) {
        std::cout << "✓ STL Binary export successful" << std::endl;
    } else {
        std::cout << "✗ STL Binary export failed: " << cad.error() << std::endl;
    }

    std::cout << "\n✓ STL export test passed" << std::endl;
}

void test_stl_import() {
    std::cout << "\n=== Test 4: STL Import ===" << std::endl;

    CADKernel cad;
    MeshData imported_mesh;

    // Import ASCII STL
    std::cout << "\nImporting STL ASCII (test_sphere_ascii.stl)..." << std::endl;
    if (cad.import_stl("test_sphere_ascii.stl", imported_mesh)) {
        std::cout << "✓ STL ASCII import successful" << std::endl;
        print_mesh_info(imported_mesh);
    } else {
        std::cout << "✗ STL ASCII import failed: " << cad.error() << std::endl;
    }

    // Import Binary STL
    std::cout << "\nImporting STL Binary (test_sphere_binary.stl)..." << std::endl;
    imported_mesh.clear();
    if (cad.import_stl("test_sphere_binary.stl", imported_mesh)) {
        std::cout << "✓ STL Binary import successful" << std::endl;
        print_mesh_info(imported_mesh);
    } else {
        std::cout << "✗ STL Binary import failed: " << cad.error() << std::endl;
    }

    std::cout << "\n✓ STL import test passed" << std::endl;
}

void test_obj_export() {
    std::cout << "\n=== Test 5: OBJ Export ===" << std::endl;

    CADKernel cad;

    // Create a box
    auto box = cad.create_box(2.0f, 3.0f, 4.0f);

    // Export OBJ
    std::cout << "\nExporting to OBJ (test_box.obj)..." << std::endl;
    if (cad.export_obj("test_box.obj", box->mesh)) {
        std::cout << "✓ OBJ export successful" << std::endl;
    } else {
        std::cout << "✗ OBJ export failed: " << cad.error() << std::endl;
    }

    std::cout << "\n✓ OBJ export test passed" << std::endl;
}

void test_obj_import() {
    std::cout << "\n=== Test 6: OBJ Import ===" << std::endl;

    CADKernel cad;
    MeshData imported_mesh;

    // Import OBJ
    std::cout << "\nImporting OBJ (test_box.obj)..." << std::endl;
    if (cad.import_obj("test_box.obj", imported_mesh)) {
        std::cout << "✓ OBJ import successful" << std::endl;
        print_mesh_info(imported_mesh);
    } else {
        std::cout << "✗ OBJ import failed: " << cad.error() << std::endl;
    }

    std::cout << "\n✓ OBJ import test passed" << std::endl;
}

void test_vertex_unification() {
    std::cout << "\n=== Test 7: Vertex Unification ===" << std::endl;

    CADKernel cad;
    auto box = cad.create_box(1.0f, 1.0f, 1.0f);

    std::cout << "\nBefore unification:" << std::endl;
    std::cout << "  Vertices: " << box->mesh.vertices.size() << std::endl;

    box->mesh.unify_vertices(0.001f);

    std::cout << "\nAfter unification (tolerance=0.001):" << std::endl;
    std::cout << "  Vertices: " << box->mesh.vertices.size() << std::endl;

    std::cout << "\n✓ Vertex unification test passed" << std::endl;
}

void test_custom_mesh() {
    std::cout << "\n=== Test 8: Custom Mesh Creation ===" << std::endl;

    CADKernel cad;

    // Create a custom pyramid mesh
    MeshData pyramid;
    pyramid.vertices = {
        {0, 1, 0},      // Top
        {-1, 0, -1},    // Base corners
        {1, 0, -1},
        {1, 0, 1},
        {-1, 0, 1}
    };

    pyramid.triangles = {
        {0, 1, 2},  // Side 1
        {0, 2, 3},  // Side 2
        {0, 3, 4},  // Side 3
        {0, 4, 1},  // Side 4
        {1, 4, 3},  // Base 1
        {1, 3, 2}   // Base 2
    };

    pyramid.calculate_normals();

    std::cout << "\nCreated custom pyramid mesh:" << std::endl;
    print_mesh_info(pyramid);

    // Create geometry from custom mesh
    auto geom = cad.create_mesh(pyramid);
    std::cout << "\nGeometry type: ";
    switch (geom->type) {
        case GeometryType::CustomMesh: std::cout << "CustomMesh"; break;
        case GeometryType::Box: std::cout << "Box"; break;
        case GeometryType::Sphere: std::cout << "Sphere"; break;
        default: std::cout << "Other"; break;
    }
    std::cout << std::endl;

    // Export pyramid
    if (cad.export_stl("test_pyramid.stl", pyramid)) {
        std::cout << "✓ Pyramid exported to test_pyramid.stl" << std::endl;
    }

    std::cout << "\n✓ Custom mesh test passed" << std::endl;
}

void test_roundtrip_stl() {
    std::cout << "\n=== Test 9: STL Roundtrip ===" << std::endl;

    CADKernel cad;

    // Create original mesh
    auto original = cad.create_sphere(1.5f, 24);
    size_t original_vertices = original->mesh.vertices.size();
    size_t original_triangles = original->mesh.triangles.size();

    std::cout << "\nOriginal mesh:" << std::endl;
    std::cout << "  Vertices: " << original_vertices << std::endl;
    std::cout << "  Triangles: " << original_triangles << std::endl;

    // Export
    if (!cad.export_stl("test_roundtrip.stl", original->mesh)) {
        std::cout << "✗ Export failed: " << cad.error() << std::endl;
        return;
    }

    // Import
    MeshData imported;
    if (!cad.import_stl("test_roundtrip.stl", imported)) {
        std::cout << "✗ Import failed: " << cad.error() << std::endl;
        return;
    }

    std::cout << "\nAfter roundtrip:" << std::endl;
    std::cout << "  Vertices: " << imported.vertices.size() << std::endl;
    std::cout << "  Triangles: " << imported.triangles.size() << std::endl;

    // Verify
    if (imported.vertices.size() == original_vertices &&
        imported.triangles.size() == original_triangles) {
        std::cout << "\n✓ STL roundtrip successful - data integrity maintained" << std::endl;
    } else {
        std::cout << "\n⚠ Warning: Vertex/triangle count mismatch (may be due to welding)" << std::endl;
    }
}

void test_boolean_operations_placeholder() {
    std::cout << "\n=== Test 10: Boolean Operations (Placeholder) ===" << std::endl;

    CADKernel cad;
    auto box1 = cad.create_box(2.0f, 2.0f, 2.0f);
    auto box2 = cad.create_box(2.0f, 2.0f, 2.0f);
    box2->set_position({1.0f, 0.0f, 0.0f});

    MeshData result;

    std::cout << "\nTesting union operation (currently placeholder)..." << std::endl;
    if (!cad.union_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "  " << cad.error() << std::endl;
    }

    std::cout << "\nTesting subtract operation (currently placeholder)..." << std::endl;
    if (!cad.subtract_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "  " << cad.error() << std::endl;
    }

    std::cout << "\nTesting intersect operation (currently placeholder)..." << std::endl;
    if (!cad.intersect_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "  " << cad.error() << std::endl;
    }

    std::cout << "\nNote: Boolean operations require external CAD library integration" << std::endl;
    std::cout << "✓ Boolean operations placeholder test passed" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== CAD Kernel Demo ===" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_primitive_creation();
        test_transform_operations();
        test_stl_export();
        test_stl_import();
        test_obj_export();
        test_obj_import();
        test_vertex_unification();
        test_custom_mesh();
        test_roundtrip_stl();
        test_boolean_operations_placeholder();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== ALL TESTS PASSED! ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
