#include <gtest/gtest.h>
#include "ProjectFile.hpp"
#include <fstream>
#include <cstdio>

using namespace mechatron;

class ProjectFileTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_path = "test_project.mtrx";
    }
    void TearDown() override {
        std::remove(test_path.c_str());
    }
    std::string test_path;
};

TEST_F(ProjectFileTest, SaveAndLoad) {
    ProjectFile pf;
    pf.data() = nlohmann::json{
        {"version", "2.0"},
        {"name", "Test Project"},
        {"plugins", {"mech_machine_elements", "soft_mcu_avr"}},
        {"components", nlohmann::json::array()},
        {"simulation", {
            {"time_step_us", 1000},
            {"duration_s", 10.0}
        }}
    };

    ASSERT_TRUE(pf.save(test_path));

    ProjectFile loaded;
    ASSERT_TRUE(loaded.load(test_path));

    EXPECT_EQ(loaded.name(), "Test Project");
    EXPECT_EQ(loaded.version(), "2.0");

    auto plugins = loaded.required_plugins();
    EXPECT_EQ(plugins.size(), 2u);
    EXPECT_EQ(plugins[0], "mech_machine_elements");
    EXPECT_EQ(plugins[1], "soft_mcu_avr");
}

TEST_F(ProjectFileTest, LoadNonexistent) {
    ProjectFile pf;
    EXPECT_FALSE(pf.load("nonexistent.mtrx"));
}

TEST_F(ProjectFileTest, Defaults) {
    ProjectFile pf;
    EXPECT_EQ(pf.name(), "Untitled");
    EXPECT_EQ(pf.version(), "2.0");
    EXPECT_TRUE(pf.required_plugins().empty());
}
