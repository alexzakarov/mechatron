#pragma once

#include "Component.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <unordered_map>
#include <functional>

namespace mechatron {

class PluginHost;

class IMechatronPlugin {
public:
    virtual ~IMechatronPlugin() = default;
    virtual std::string_view name() const = 0;
    virtual std::string_view version() const = 0;
    virtual std::vector<ComponentDescriptor> components() const = 0;
    virtual std::unique_ptr<Component> create(std::string_view type) = 0;
    virtual void on_register(PluginHost& host) = 0;
    virtual void on_unregister() = 0;
};

class PluginHost {
public:
    bool register_plugin(std::unique_ptr<IMechatronPlugin> plugin);
    void unregister_plugin(std::string_view name);

    IMechatronPlugin* get_plugin(std::string_view name) const;
    std::vector<IMechatronPlugin*> get_all_plugins() const;

    std::unique_ptr<Component> create_component(std::string_view plugin_name, std::string_view type) const;
    std::vector<ComponentDescriptor> get_all_component_descriptors() const;

    template <typename Func>
    void for_each_plugin(Func fn) const {
        for (const auto& [_, plugin] : m_plugins) {
            fn(*plugin);
        }
    }

private:
    std::unordered_map<std::string, std::unique_ptr<IMechatronPlugin>> m_plugins;
};

} // namespace mechatron
