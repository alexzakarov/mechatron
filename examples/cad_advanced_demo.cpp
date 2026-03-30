// CAD Advanced Demo - STEP/IGES Import and CSG Operations
// Demonstrates OpenCASCADE integration for CAD file import and boolean operations

#include "cad/CADKernel.hpp"
#include <iostream>
#include <iomanip>

using namespace mechatron;

void print_mesh_info(const std::string& name, const MeshData& mesh) {
    Vec3 min, max;
    mesh.get_bounds(min, max);

    std::cout << "  " << name << ":" << std::endl;
    std::cout << "    Vertices: " << mesh.vertices.size() << std::endl;
    std::cout << "    Triangles: " << mesh.triangles.size() << std::endl;
    std::cout << "    Bounds: [" << min.x << ", " << min.y << ", " << min.z << "] to ["
              << max.x << ", " << max.y << ", " << max.z << "]" << std::endl;
    std::cout << "    Size: " << (max.x - min.x) << " x " << (max.y - min.y)
              << " x " << (max.z - min.z) << std::endl;
}

void test_opencascade_support() {
    std::cout << "\n=== Test 1: OpenCASCADE Support Check ===" << std::endl;

    CADKernel cad;

    if (cad.has_opencascade_support()) {
        std::cout << "✓ OpenCASCADE is available" << std::endl;
        std::cout << "  - STEP/IGES import: Supported" << std::endl;
        std::cout << "  - CSG operations: Supported" << std::endl;
        std::cout << "  - Mesh processing: Supported" << std::endl;
    } else {
        std::cout << "⚠ OpenCASCADE is not available" << std::endl;
        std::cout << "  To enable OpenCASCADE support:" << std::endl;
        std::cout << "  1. Install OpenCASCADE:" << std::endl;
        std::cout << "     Windows: Download from https://dev.opencascade.org/" << std::endl;
        std::cout << "     Linux: sudo apt-get install libopencascade-occt-dev" << std::endl;
        std::cout << "     macOS: brew install opencascade" << std::endl;
        std::cout << "  2. Rebuild with: cmake -DMECHATRON_USE_OPENCASCADE=ON .." << std::endl;
    }
}

void test_step_import() {
    std::cout << "\n=== Test 2: STEP File Import ===" << std::endl;

    CADKernel cad;
    MeshData imported_mesh;

    // Try to import a STEP file
    std::string test_file = "test_part.step";

    std::cout << "Attempting to import: " << test_file << std::endl;

    if (cad.import_step(test_file, imported_mesh)) {
        std::cout << "✓ STEP import successful" << std::endl;
        print_mesh_info("Imported Part", imported_mesh);
    } else {
        std::cout << "⚠ STEP import failed: " << cad.error() << std::endl;
        std::cout << "  This is expected if:" << std::endl;
        std::cout << "  - OpenCASCADE is not installed" << std::endl;
        std::cout << "  - The file doesn't exist" << std::endl;
        std::cout << "  - The file is not a valid STEP file" << std::endl;
    }
}

void test_iges_import() {
    std::cout << "\n=== Test 3: IGES File Import ===" << std::endl;

    CADKernel cad;
    MeshData imported_mesh;

    std::string test_file = "test_part.iges";

    std::cout << "Attempting to import: " << test_file << std::endl;

    if (cad.import_iges(test_file, imported_mesh)) {
        std::cout << "✓ IGES import successful" << std::endl;
        print_mesh_info("Imported Part", imported_mesh);
    } else {
        std::cout << "⚠ IGES import failed: " << cad.error() << std::endl;
    }
}

void test_csg_operations() {
    std::cout << "\n=== Test 4: CSG Boolean Operations ===" << std::endl;

    CADKernel cad;

    // Create two overlapping boxes
    auto box1 = cad.create_box(2.0f, 2.0f, 2.0f);
    box1->set_position({-0.5f, 0, 0});

    auto box2 = cad.create_box(2.0f, 2.0f, 2.0f);
    box2->set_position({0.5f, 0, 0});

    std::cout << "Created two overlapping boxes:" << std::endl;
    print_mesh_info("Box 1", box1->mesh);
    print_mesh_info("Box 2", box2->mesh);

    MeshData result;

    // Test Union
    std::cout << "\nTesting Union (Box1 + Box2)..." << std::endl;
    if (cad.union_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "✓ Union successful" << std::endl;
        print_mesh_info("Union Result", result);
    } else {
        std::cout << "⚠ Union failed: " << cad.error() << std::endl;
    }

    // Test Subtract
    std::cout << "\nTesting Subtract (Box1 - Box2)..." << std::endl;
    if (cad.subtract_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "✓ Subtract successful" << std::endl;
        print_mesh_info("Subtract Result", result);
    } else {
        std::cout << "⚠ Subtract failed: " << cad.error() << std::endl;
    }

    // Test Intersect
    std::cout << "\nTesting Intersect (Box1 ∩ Box2)..." << std::endl;
    if (cad.intersect_meshes(box1->mesh, box2->mesh, result)) {
        std::cout << "✓ Intersect successful" << std::endl;
        print_mesh_info("Intersect Result", result);
    } else {
        std::cout << "⚠ Intersect failed: " << cad.error() << std::endl;
    }
}

void test_mesh_processing() {
    std::cout << "\n=== Test 5: Mesh Processing ===" << std::endl;

    CADKernel cad;

    // Create a dense sphere
    auto sphere = cad.create_sphere(1.5f, 64);

    std::cout << "Original dense sphere:" << std::endl;
    print_mesh_info("Sphere", sphere->mesh);

    // Test simplification
    std::cout << "\nTesting mesh simplification (50% target)..." << std::endl;
    MeshData simplified = sphere->mesh;
    cad.simplify_mesh(simplified, 0.5f);

    std::cout << "After simplification:" << std::endl;
    print_mesh_info("Simplified", simplified);

    // Export simplified mesh
    if (cad.export_stl("simplified_sphere.stl", simplified)) {
        std::cout << "✓ Exported to simplified_sphere.stl" << std::endl;
    }
}

void test_step_export() {
    std::cout << "\n=== Test 6: STEP Export ===" << std::endl;

    CADKernel cad;

    auto box = cad.create_box(1.0f, 1.0f, 1.0f);

    std::cout << "Attempting to export to STEP..." << std::endl;
    if (cad.export_step("test_export.step", box->mesh)) {
        std::cout << "✓ STEP export successful" << std::endl;
    } else {
        std::cout << "⚠ STEP export failed: " << cad.error() << std::endl;
    }
}

void test_file_format_comparison() {
    std::cout << "\n=== Test 7: File Format Comparison ===" << std::endl;

    std::cout << "\nCAD File Format Support:" << std::endl;
    std::cout << std::left << std::setw(12) << "Format"
              << std::setw(15) << "Import"
              << std::setw(15) << "Export"
              << "Description" << std::endl;
    std::cout << std::string(70, '-') << std::endl;

    CADKernel cad;
    bool has_occt = cad.has_opencascade_support();

    std::cout << std::left << std::setw(12) << "STL"
              << std::setw(15) << "✓ Yes"
              << std::setw(15) << "✓ Yes"
              << "Mesh format (ASCII/Binary)" << std::endl;

    std::cout << std::left << std::setw(12) << "OBJ"
              << std::setw(15) << "✓ Yes"
              << std::setw(15) << "✓ Yes"
              << "Wavefront mesh format" << std::endl;

    std::cout << std::left << std::setw(12) << "STEP"
              << std::setw(15) << (has_occt ? "✓ Yes" : "✗ No (need OCCT)")
              << std::setw(15) << (has_occt ? "✓ Yes" : "✗ No (need OCCT)")
              << "NURBS-based CAD format" << std::endl;

    std::cout << std::left << std::setw(12) << "IGES"
              << std::setw(15) << (has_occt ? "✓ Yes" : "✗ No (need OCCT)")
              << std::setw(15) << "✗ No"
              << "Legacy CAD format" << std::endl;

    std::cout << std::left << std::setw(12) << "BREP"
              << std::setw(15) << (has_occt ? "✓ Yes" : "✗ No (need OCCT)")
              << std::setw(15) << "✗ No"
              << "OpenCASCADE native format" << std::endl;

    std::cout << "\nCSG Operations:" << std::endl;
    std::cout << "  Union:      " << (has_occt ? "✓ Supported" : "✗ Requires OpenCASCADE") << std::endl;
    std::cout << "  Subtract:   " << (has_occt ? "✓ Supported" : "✗ Requires OpenCASCADE") << std::endl;
    std::cout << "  Intersect:  " << (has_occt ? "✓ Supported" : "✗ Requires OpenCASCADE") << std::endl;
}

void create_sample_step_file_info() {
    std::cout << "\n=== Creating Sample STEP Files ===" << std::endl;

    std::cout << "\nTo test STEP import, you need sample STEP files." << std::endl;
    std::cout << "You can:" << std::endl;
    std::cout << "  1. Export from CAD software (SolidWorks, Fusion 360, etc.)" << std::endl;
    std::cout << "  2. Download from online repositories:" << std::endl;
    std::cout << "     - https://www.traceparts.com/" << std::endl;
    std::cout << "     - https://www.3dcontentcentral.com/" << std::endl;
    std::cout << "     - GrabCAD: https://grabcad.com/library" << std::endl;
    std::cout << "  3. Create test geometries using FreeCAD:" << std::endl;
    std::cout << "     - Open FreeCAD" << std::endl;
    std::cout << "     - Create a simple box/cylinder/sphere" << std::endl;
    std::cout << "     - File → Export → Select STEP format" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "=== CAD Advanced Demo ===" << std::endl;
    std::cout << "=== OpenCASCADE Integration ===" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_opencascade_support();
        test_step_import();
        test_iges_import();
        test_csg_operations();
        test_mesh_processing();
        test_step_export();
        test_file_format_comparison();
        create_sample_step_file_info();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== CAD ADVANCED DEMO COMPLETE ===" << std::endl;
        std::cout << "========================================" << std::endl;

        if (!CADKernel().has_opencascade_support()) {
            std::cout << "\nNote: Many features require OpenCASCADE." << std::endl;
            std::cout << "See above for installation instructions." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
