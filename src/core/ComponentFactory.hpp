#pragma once

#include "Component.hpp"
#include "Registry.hpp"
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <functional>

namespace mechatron {

/**
 * @brief Component metadata for registration
 */
struct ComponentInfo {
    std::string type_name;           // e.g., "dc_motor"
    std::string category;            // e.g., "actuator"
    std::string display_name;        // e.g., "DC Motor"
    std::string description;         // Brief description
    std::function<std::unique_ptr<Component>()> factory;
};

/**
 * @brief Global component factory for creating components by type
 *
 * Allows runtime component creation from type names.
 * All component types must be registered with REGISTER_COMPONENT().
 */
class ComponentFactory {
public:
    static ComponentFactory& instance();

    /**
     * Register a component type
     */
    void register_component(const ComponentInfo& info);

    /**
     * Create component by type name
     */
    std::unique_ptr<Component> create(std::string_view type_name) const;

    /**
     * Get all registered component types
     */
    const std::unordered_map<std::string, ComponentInfo>& registered_types() const;

    /**
     * Get components by category
     */
    std::vector<const ComponentInfo*> get_by_category(std::string_view category) const;

    /**
     * Check if type is registered
     */
    bool is_registered(std::string_view type_name) const;

private:
    ComponentFactory() = default;
    std::unordered_map<std::string, ComponentInfo> m_types;
};

/**
 * @brief Helper to register components at static initialization
 */
struct ComponentRegistrar {
    ComponentRegistrar(const ComponentInfo& info) {
        ComponentFactory::instance().register_component(info);
    }
};

/**
 * @brief Macro to register component types
 *
 * Usage in .cpp file:
 * REGISTER_COMPONENT(DCMotor, "DC Motor", "DC motor with encoder support", "actuator");
 */
#define REGISTER_COMPONENT(Class, DisplayName, Description, Category) \
    namespace { \
        mechatron::ComponentRegistrar registrar_##Class({ \
            #Class, \
            Category, \
            DisplayName, \
            Description, \
            []() -> std::unique_ptr<mechatron::Component> { \
                return std::make_unique<Class>(); \
            } \
        }); \
    }

/**
 * @brief Macro to register with custom type name (different from class name)
 */
#define REGISTER_COMPONENT_CUSTOM(TypeName, Class, DisplayName, Description, Category) \
    namespace { \
        mechatron::ComponentRegistrar registrar_##Class({ \
            TypeName, \
            Category, \
            DisplayName, \
            Description, \
            []() -> std::unique_ptr<mechatron::Component> { \
                return std::make_unique<Class>(); \
            } \
        }); \
    }

} // namespace mechatron
