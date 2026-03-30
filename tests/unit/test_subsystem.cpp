#include <gtest/gtest.h>
#include "Subsystem.hpp"
#include <nlohmann/json.hpp>

using namespace mechatron;

TEST(Subsystem, Creation) {
    Subsystem sub("motor_drive");
    EXPECT_EQ(sub.id(), "motor_drive");
}

TEST(Subsystem, AddMembers) {
    Subsystem sub("drive");
    sub.add_component("motor_1", "actuator");
    sub.add_component("driver_1", "driver");
    sub.add_component("encoder_1", "feedback");

    EXPECT_EQ(sub.members().size(), 3u);
    EXPECT_EQ(sub.members()[0].component_id, "motor_1");
    EXPECT_EQ(sub.members()[0].role, "actuator");
}

TEST(Subsystem, InternalConnections) {
    Subsystem sub("drive");
    sub.add_internal_connection("arduino.d9", "h_bridge.pwm");
    sub.add_internal_connection("encoder.a", "arduino.d2");

    EXPECT_EQ(sub.internal_connections().size(), 2u);
    EXPECT_EQ(sub.internal_connections()[0].source, "arduino.d9");
}

TEST(Subsystem, ExposedPorts) {
    Subsystem sub("drive");
    sub.add_exposed_port({"power_supply", "h_bridge", "vcc"});
    sub.add_exposed_port({"mechanical_output", "motor_1", "shaft"});

    EXPECT_EQ(sub.exposed_ports().size(), 2u);
    EXPECT_EQ(sub.exposed_ports()[0].name, "power_supply");
}

TEST(Subsystem, SerializeDeserialize) {
    Subsystem original("test_sub");
    original.set_display_name("Test Subsystem");
    original.add_component("comp_1", "role_1");
    original.add_internal_connection("a.x", "b.y");
    original.add_exposed_port({"port_1", "comp_1", "out"});

    nlohmann::json json;
    original.serialize(json);

    Subsystem loaded("temp");
    loaded.deserialize(json);

    EXPECT_EQ(loaded.id(), "test_sub");
    EXPECT_EQ(loaded.display_name(), "Test Subsystem");
    EXPECT_EQ(loaded.members().size(), 1u);
    EXPECT_EQ(loaded.internal_connections().size(), 1u);
    EXPECT_EQ(loaded.exposed_ports().size(), 1u);
}
