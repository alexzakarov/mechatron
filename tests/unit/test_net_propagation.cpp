#include <gtest/gtest.h>

#include "SimulationOrchestrator.hpp"
#include "plugins/instruments/InstrumentPlugin.hpp"

using namespace mechatron;

static Port* find_port(Component* c, std::string_view name) {
    if (!c) return nullptr;
    for (auto* p : c->get_ports()) {
        if (p && p->name() == name) return p;
    }
    return nullptr;
}

TEST(NetPropagation, DCVoltageDrivesMCURailsWhileStopped) {
    SimulationOrchestrator orch;
    orch.load_all_plugins();

    auto* vsrc = orch.create_component("elec_power", "dc_voltage", "dc_voltage_1");
    ASSERT_NE(vsrc, nullptr);
    auto* mcu = orch.create_component("soft_mcu_avr", "atmega328p", "atmega328p_1");
    ASSERT_NE(mcu, nullptr);

    Port* v_plus = find_port(vsrc, "V+");
    Port* v_gnd = find_port(vsrc, "GND");
    Port* m_vcc = find_port(mcu, "VCC");
    Port* m_gnd = find_port(mcu, "GND");
    ASSERT_NE(v_plus, nullptr);
    ASSERT_NE(v_gnd, nullptr);
    ASSERT_NE(m_vcc, nullptr);
    ASSERT_NE(m_gnd, nullptr);

    orch.connect(v_plus, m_vcc, "wire_vcc");
    orch.connect(v_gnd, m_gnd, "wire_gnd");

    // Simulation is stopped by default; update() should still run non-MCU components
    // and propagate nets so the rails are visible in the UI and for instruments.
    orch.update();

    const float* vcc = m_vcc->get_value<float>();
    const float* gnd = m_gnd->get_value<float>();
    ASSERT_NE(vcc, nullptr);
    ASSERT_NE(gnd, nullptr);
    EXPECT_NEAR(*gnd, 0.0f, 1e-4f);
    EXPECT_NEAR(*vcc, 5.0f, 1e-3f);
}

TEST(NetPropagation, OscilloscopeReadsConnectedDigitalVoltage) {
    OscilloscopeComponent scope;
    Port digital_out("D13", PortDomain::Digital, PortDirection::Bidirectional);
    digital_out.set_value(5.0f);

    auto channels = scope.get_ports();
    ASSERT_FALSE(channels.empty());
    Connection wire(&digital_out, channels[0], "wire_d13_scope");

    scope.update(0.001);

    const auto& ch1 = scope.channel_data(0);
    ASSERT_FALSE(ch1.empty());
    EXPECT_NEAR(ch1.back().voltage, 5.0f, 1e-4f);

    const float* displayed = channels[0]->get_value<float>();
    ASSERT_NE(displayed, nullptr);
    EXPECT_NEAR(*displayed, 5.0f, 1e-4f);
}
