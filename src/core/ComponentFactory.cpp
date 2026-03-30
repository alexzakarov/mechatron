#include "ComponentFactory.hpp"

namespace mechatron {

ComponentFactory& ComponentFactory::instance() {
    static ComponentFactory factory;
    return factory;
}

void ComponentFactory::register_component(const ComponentInfo& info) {
    m_types[info.type_name] = info;
}

std::unique_ptr<Component> ComponentFactory::create(std::string_view type_name) const {
    auto it = m_types.find(std::string(type_name));
    if (it != m_types.end() && it->second.factory) {
        return it->second.factory();
    }
    return nullptr;
}

const std::unordered_map<std::string, ComponentInfo>& ComponentFactory::registered_types() const {
    return m_types;
}

std::vector<const ComponentInfo*> ComponentFactory::get_by_category(std::string_view category) const {
    std::vector<const ComponentInfo*> result;
    for (const auto& [name, info] : m_types) {
        if (info.category == category) {
            result.push_back(&info);
        }
    }
    return result;
}

bool ComponentFactory::is_registered(std::string_view type_name) const {
    return m_types.find(std::string(type_name)) != m_types.end();
}

} // namespace mechatron
