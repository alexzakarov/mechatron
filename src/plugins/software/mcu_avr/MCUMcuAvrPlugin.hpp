#pragma once

#include "core/PluginHost.hpp"
#include <memory>
#include <vector>
#include <string_view>

namespace mechatron {

/**
 * @brief AVR MCU Plugin
 *
 * Provides AVR microcontroller emulation (ATmega328P, ATmega2560, ATtiny).
 * Uses QEMU backend for accurate instruction-level simulation.
 */
class MCUMcuAvrPlugin : public IMechatronPlugin {
public:
    std::string_view name() const override { return "soft_mcu_avr"; }
    std::string_view version() const override { return "1.0.0"; }

    std::vector<ComponentDescriptor> components() const override;
    std::unique_ptr<Component> create(std::string_view type) override;

    void on_register(PluginHost& host) override;
    void on_unregister() override;
};

} // namespace mechatron
