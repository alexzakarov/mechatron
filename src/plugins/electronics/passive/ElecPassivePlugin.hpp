#pragma once

#include "core/PluginHost.hpp"
#include <memory>
#include <vector>
#include <string_view>

namespace mechatron {

/**
 * @brief Passive Electronics Plugin
 *
 * Provides passive electronic components: resistors, capacitors, inductors.
 */
class ElecPassivePlugin : public IMechatronPlugin {
public:
    std::string_view name() const override { return "elec_passive"; }
    std::string_view version() const override { return "1.0.0"; }

    std::vector<ComponentDescriptor> components() const override;
    std::unique_ptr<Component> create(std::string_view type) override;

    void on_register(PluginHost& host) override;
    void on_unregister() override;
};

} // namespace mechatron
