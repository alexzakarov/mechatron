#include "PluginHost.hpp"
#include <spdlog/spdlog.h>

namespace mechatron {

bool PluginHost::register_plugin(std::unique_ptr<IMechatronPlugin> plugin) {
    std::string name(plugin->name());
    if (m_plugins.contains(name)) {
        spdlog::warn("Plugin '{}' already registered", name);
        return false;
    }

    plugin->on_register(*this);
    spdlog::info("Plugin registered: {} v{}", plugin->name(), plugin->version());
    m_plugins[name] = std::move(plugin);
    return true;
}

void PluginHost::unregister_plugin(std::string_view name) {
    auto it = m_plugins.find(std::string(name));
    if (it != m_plugins.end()) {
        it->second->on_unregister();
        spdlog::info("Plugin unregistered: {}", name);
        m_plugins.erase(it);
    }
}

IMechatronPlugin* PluginHost::get_plugin(std::string_view name) const {
    auto it = m_plugins.find(std::string(name));
    return it != m_plugins.end() ? it->second.get() : nullptr;
}

std::vector<IMechatronPlugin*> PluginHost::get_all_plugins() const {
    std::vector<IMechatronPlugin*> result;
    for (const auto& [_, plugin] : m_plugins) {
        result.push_back(plugin.get());
    }
    return result;
}

std::unique_ptr<Component> PluginHost::create_component(std::string_view plugin_name, std::string_view type) const {
    auto* plugin = get_plugin(plugin_name);
    if (!plugin) {
        spdlog::error("Plugin '{}' not found", plugin_name);
        return nullptr;
    }
    return plugin->create(type);
}

std::vector<ComponentDescriptor> PluginHost::get_all_component_descriptors() const {
    std::vector<ComponentDescriptor> result;
    for (const auto& [_, plugin] : m_plugins) {
        auto descs = plugin->components();
        result.insert(result.end(), descs.begin(), descs.end());
    }
    return result;
}

} // namespace mechatron
