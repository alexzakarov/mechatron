#pragma once

#include "Component.hpp"
#include <unordered_map>
#include <memory>
#include <string>
#include <string_view>
#include <functional>
#include <vector>

namespace mechatron {

class Registry {
public:
    Component* add(std::unique_ptr<Component> component, std::string id) {
        component->set_id(std::move(id));
        Component* ptr = component.get();
        m_components[ptr->id()] = std::move(component);
        ptr->on_register(*this);
        return ptr;
    }

    void remove(std::string_view id) {
        auto it = m_components.find(std::string(id));
        if (it != m_components.end()) {
            it->second->on_unregister();
            m_components.erase(it);
        }
    }

    Component* get(std::string_view id) {
        auto it = m_components.find(std::string(id));
        return it != m_components.end() ? it->second.get() : nullptr;
    }

    template <typename T>
    T* get_as(std::string_view id) {
        return dynamic_cast<T*>(get(id));
    }

    void for_each(std::function<void(Component&)> fn) {
        for (auto& [_, comp] : m_components) {
            fn(*comp);
        }
    }

    void for_each(std::function<void(const Component&)> fn) const {
        for (const auto& [_, comp] : m_components) {
            fn(*comp);
        }
    }

    size_t size() const { return m_components.size(); }
    void clear() { m_components.clear(); }

    // Access all components (read-only)
    const auto& all_components() const { return m_components; }

private:
    std::unordered_map<std::string, std::unique_ptr<Component>> m_components;
};

} // namespace mechatron
