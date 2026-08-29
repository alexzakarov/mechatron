/*
#pragma once

#include "CircuitSimulator.hpp"

#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace mechatron {

class SpiceCurrentProbeRegistry {
public:
    struct Probe {
        std::string expression_template;
        std::string pin;
        double sign = 1.0;
    };

    struct Definition {
        std::string id;
        std::vector<std::string> component_types;
        std::vector<Probe> probes;
        std::vector<std::string> extra_save_expression_templates;
    };

    using CurrentGetter = std::function<bool(const std::string& expression, double& value)>;

    static const Definition* find(std::string_view component_type);
    static std::vector<std::string> save_expressions(std::string_view component_type,
                                                     const std::string& component_id);
    static bool apply_currents(CircuitComponent* component, const CurrentGetter& get_current);
    static std::string render_expression(std::string_view expression_template,
                                         std::string_view component_type,
                                         const std::string& component_id);
    static std::string normalize_expression(std::string expression);
};

} // namespace mechatron
