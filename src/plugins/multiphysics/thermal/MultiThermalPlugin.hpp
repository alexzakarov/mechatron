#pragma once

#include "core/PluginHost.hpp"
#include <memory>
#include <vector>
#include <string_view>

namespace mechatron {

/**
 * @brief Thermal Physics Plugin
 *
 * Provides thermal simulation components: heat sources, heat sinks, convection.
 */
class MultiThermalPlugin : public IMechatronPlugin {
public:
    std::string_view name() const override { return "multi_thermal"; }
    std::string_view version() const override { return "1.0.0"; }

    std::vector<ComponentDescriptor> components() const override;
    std::unique_ptr<Component> create(std::string_view type) override;

    void on_register(PluginHost& host) override;
    void on_unregister() override;
};

} // namespace mechatron
