/*
#include "SpiceCurrentProbeRegistry.hpp"
#include "core/CatalogProvider.hpp"

#include <algorithm>
#include <cctype>
#include <nlohmann/json.hpp>
#include <set>
#include <spdlog/spdlog.h>
#include <unordered_map>

namespace mechatron {

namespace {

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string prefixed_spice_name(char prefix, const std::string& id) {
    if (!id.empty() &&
        std::toupper(static_cast<unsigned char>(id.front())) ==
            std::toupper(static_cast<unsigned char>(prefix))) {
        return id;
    }
    return std::string(1, prefix) + id;
}

void replace_all(std::string& value, const std::string& from, const std::string& to) {
    if (from.empty()) {
        return;
    }
    size_t pos = 0;
    while ((pos = value.find(from, pos)) != std::string::npos) {
        value.replace(pos, from.size(), to);
        pos += to.size();
    }
}

std::string spice_name_for_component(std::string_view component_type, const std::string& id) {
    static const auto prefix_by_type = [] {
        std::unordered_map<std::string, char> map;
        try {
            const auto catalog = default_catalog_provider().load_catalog();
            if (catalog) {
                for (const auto& conv : catalog->value("spice_converters", nlohmann::json::array())) {
                    const std::string prefix_str = conv.value("spice_prefix", std::string{});
                    const char prefix = prefix_str.empty() ? '\0' : std::toupper(static_cast<unsigned char>(prefix_str.front()));
                    if (!prefix) continue;
                    for (const auto& type : conv.value("component_types", std::vector<std::string>{})) {
                        if (!type.empty()) map[type] = prefix;
                    }
                }
            }
        } catch (const std::exception& e) {
            spdlog::warn("[SpiceCurrentProbeRegistry] failed to load SPICE prefix catalog: {}", e.what());
        }
        return map;
    }();

    const auto prefix_it = prefix_by_type.find(std::string(component_type));
    if (prefix_it != prefix_by_type.end()) {
        return prefixed_spice_name(prefix_it->second, id);
    }
    return id;
}

std::string branch_name_for_component(std::string_view, const std::string& id) {
    return to_lower(prefixed_spice_name('V', id)) + "#branch";
}

void set_pin_current(CircuitComponent* comp, const std::string& pin_name, double current) {
    for (auto* pin : comp->get_pins()) {
        if (pin && pin->id == pin_name) {
            pin->current = static_cast<float>(current);
            return;
        }
    }
}

std::vector<SpiceCurrentProbeRegistry::Definition> load_catalog_definitions() {
    const auto catalog = default_catalog_provider().load_catalog();
    if (!catalog) {
        return {};
    }

    try {
        std::vector<SpiceCurrentProbeRegistry::Definition> definitions;
        for (const auto& entry : catalog->value("current_probes", nlohmann::json::array())) {
            SpiceCurrentProbeRegistry::Definition def;
            def.id = entry.value("id", "");
            def.component_types = entry.value("component_types", std::vector<std::string>{});
            if (entry.contains("save_expressions")) {
                def.extra_save_expression_templates =
                    entry.value("save_expressions", std::vector<std::string>{});
            }
            for (const auto& probe : entry.value("probes", nlohmann::json::array())) {
                SpiceCurrentProbeRegistry::Probe parsed;
                parsed.expression_template = probe.value("expression_template", "");
                parsed.pin = probe.value("pin", "");
                parsed.sign = probe.value("sign", 1.0);
                if (!parsed.expression_template.empty() && !parsed.pin.empty()) {
                    def.probes.push_back(std::move(parsed));
                }
            }
            if (!def.id.empty() && !def.component_types.empty() && !def.probes.empty()) {
                definitions.push_back(std::move(def));
            }
        }
        if (!definitions.empty()) {
            return definitions;
        }
    } catch (const std::exception& e) {
        spdlog::warn("[SpiceCurrentProbeRegistry] catalog current_probes unavailable: {}", e.what());
    }

    return {};
}

const std::unordered_map<std::string, SpiceCurrentProbeRegistry::Definition>& definitions_by_type() {
    static const auto definitions = [] {
        std::unordered_map<std::string, SpiceCurrentProbeRegistry::Definition> by_type;
        for (const auto& def : load_catalog_definitions()) {
            for (const auto& type : def.component_types) {
                by_type[type] = def;
            }
        }
        return by_type;
    }();
    return definitions;
}

} // namespace

const SpiceCurrentProbeRegistry::Definition*
SpiceCurrentProbeRegistry::find(std::string_view component_type) {
    const auto& definitions = definitions_by_type();
    auto it = definitions.find(std::string(component_type));
    return it == definitions.end() ? nullptr : &it->second;
}

std::string SpiceCurrentProbeRegistry::render_expression(std::string_view expression_template,
                                                         std::string_view component_type,
                                                         const std::string& component_id) {
    std::string expression(expression_template);
    replace_all(expression, "{spice_name}", spice_name_for_component(component_type, component_id));
    replace_all(expression, "{branch_name}", branch_name_for_component(component_type, component_id));
    replace_all(expression, "{id}", component_id);
    return expression;
}

std::string SpiceCurrentProbeRegistry::normalize_expression(std::string expression) {
    return to_lower(std::move(expression));
}

std::vector<std::string> SpiceCurrentProbeRegistry::save_expressions(std::string_view component_type,
                                                                     const std::string& component_id) {
    const auto* definition = find(component_type);
    if (!definition) {
        return {};
    }

    std::set<std::string> unique;
    std::vector<std::string> expressions;
    auto append = [&](const std::string& expression_template) {
        auto expression = render_expression(expression_template, component_type, component_id);
        if (unique.insert(normalize_expression(expression)).second) {
            expressions.push_back(std::move(expression));
        }
    };

    for (const auto& expression_template : definition->extra_save_expression_templates) {
        append(expression_template);
    }
    for (const auto& probe : definition->probes) {
        append(probe.expression_template);
    }
    return expressions;
}

bool SpiceCurrentProbeRegistry::apply_currents(CircuitComponent* component,
                                               const CurrentGetter& get_current) {
    if (!component) {
        return false;
    }
    const std::string component_type(component->type());
    const auto* definition = find(component_type);
    if (!definition) {
        return false;
    }

    const std::string component_id = component->id();
    bool applied = false;
    for (const auto& probe : definition->probes) {
        const auto expression = render_expression(probe.expression_template, component_type, component_id);
        double raw_current = 0.0;
        if (!get_current(expression, raw_current)) {
            continue;
        }
        const double pin_current = raw_current * probe.sign;
        set_pin_current(component, probe.pin, pin_current);
        applied = true;
    }
    return applied;
}

} // namespace mechatron
