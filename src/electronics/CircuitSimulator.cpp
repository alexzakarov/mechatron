#include "CircuitSimulator.hpp"
#include <spdlog/spdlog.h>

namespace mechatron {

CircuitSimulator::CircuitSimulator() {
    spdlog::info("CircuitSimulator initialized");
}

bool CircuitSimulator::remove_component(std::string_view id) {
    std::string id_str(id);

    // Check if it's an external component
    auto ext_it = m_components_external.find(id_str);
    if (ext_it != m_components_external.end()) {
        // Remove connected wires
        std::vector<std::string> wires_to_remove;
        for (auto& [wire_id, wire] : m_wires) {
            if (wire->source && wire->source->component_id == id_str ||
                wire->target && wire->target->component_id == id_str) {
                wires_to_remove.push_back(wire_id);
            }
        }
        for (const auto& wire_id : wires_to_remove) {
            m_wires.erase(wire_id);
        }

        // Just remove the reference - don't delete the external component
        m_components_external.erase(ext_it);
        spdlog::debug("Removed external circuit component reference: {}", id);
        return true;
    }

    // Check if it's an owned component
    auto owned_it = m_components_owned.find(id_str);
    if (owned_it != m_components_owned.end()) {
        // Remove connected wires
        std::vector<std::string> wires_to_remove;
        for (auto& [wire_id, wire] : m_wires) {
            if (wire->source && wire->source->component_id == id_str ||
                wire->target && wire->target->component_id == id_str) {
                wires_to_remove.push_back(wire_id);
            }
        }
        for (const auto& wire_id : wires_to_remove) {
            m_wires.erase(wire_id);
        }

        // unique_ptr will automatically delete the component
        m_components_owned.erase(owned_it);
        spdlog::debug("Removed owned circuit component: {}", id);
        return true;
    }

    return false;
}

CircuitComponent* CircuitSimulator::get_component(std::string_view id) {
    std::string id_str(id);

    // Check external components first
    auto ext_it = m_components_external.find(id_str);
    if (ext_it != m_components_external.end()) {
        return ext_it->second;
    }

    // Check owned components
    auto owned_it = m_components_owned.find(id_str);
    if (owned_it != m_components_owned.end()) {
        return owned_it->second.get();
    }

    return nullptr;
}

bool CircuitSimulator::connect(const std::string& wire_id,
                                const std::string& source_comp,
                                const std::string& source_pin,
                                const std::string& target_comp,
                                const std::string& target_pin) {
    // Find source pin
    auto* src_comp = get_component(source_comp);
    if (!src_comp) {
        spdlog::error("Source component not found: {}", source_comp);
        return false;
    }

    auto src_pins = src_comp->get_pins();
    CircuitPin* src_pin_ptr = nullptr;
    for (auto* pin : src_pins) {
        if (pin->id == source_pin) {
            src_pin_ptr = pin;
            break;
        }
    }

    if (!src_pin_ptr) {
        spdlog::error("Source pin not found: {}.{}", source_comp, source_pin);
        return false;
    }

    // Find target pin
    auto* tgt_comp = get_component(target_comp);
    if (!tgt_comp) {
        spdlog::error("Target component not found: {}", target_comp);
        return false;
    }

    auto tgt_pins = tgt_comp->get_pins();
    CircuitPin* tgt_pin_ptr = nullptr;
    for (auto* pin : tgt_pins) {
        if (pin->id == target_pin) {
            tgt_pin_ptr = pin;
            break;
        }
    }

    if (!tgt_pin_ptr) {
        spdlog::error("Target pin not found: {}.{}", target_comp, target_pin);
        return false;
    }

    // Create wire
    auto wire = std::make_unique<Wire>();
    wire->id = wire_id;
    wire->source = src_pin_ptr;
    wire->target = tgt_pin_ptr;
    m_wires[wire_id] = std::move(wire);

    spdlog::debug("Connected {}:{} -> {}:{}", source_comp, source_pin, target_comp, target_pin);
    return true;
}

bool CircuitSimulator::disconnect(const std::string& wire_id) {
    auto it = m_wires.find(wire_id);
    if (it == m_wires.end()) return false;

    m_wires.erase(it);
    spdlog::debug("Disconnected wire: {}", wire_id);
    return true;
}

void CircuitSimulator::step(double dt) {
    if (m_components_owned.empty() && m_components_external.empty()) return;

    // Helper lambda to iterate over all components (both owned and external)
    auto for_each_component = [this](auto&& fn) {
        for (auto& [id, comp] : m_components_owned) {
            fn(comp.get());
        }
        for (auto& [id, comp] : m_components_external) {
            fn(comp);
        }
    };

    // Phase A: Let voltage sources and active components set initial pin states
    for_each_component([dt](CircuitComponent* comp) {
        comp->update(dt);
    });

    // Phase B: Build nets (identify electrical nodes via Union-Find)
    build_nets();

    // Phase C: Count voltage sources for MNA auxiliary variables
    m_num_voltage_sources = 0;
    for_each_component([this](CircuitComponent* comp) {
        if (dynamic_cast<DCVoltageSource*>(comp)) {
            m_num_voltage_sources++;
        }
    });

    spdlog::info("[CIRCUIT] step: nodes={}, voltage_sources={}, owned_components={}, external_components={}",
                 m_num_nodes, m_num_voltage_sources, m_components_owned.size(), m_components_external.size());

    if (m_num_nodes <= 1 && m_num_voltage_sources == 0) {
        // No circuit to solve (just ground or empty)
        return;
    }

    // Phase D: Build and solve MNA system with Newton-Raphson iteration
    MNASolver final_solver;  // Store final solver for current extraction
    constexpr int MAX_ITERATIONS = 10;
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        MNASolver solver;
        solver.begin_build(m_num_nodes, m_num_voltage_sources);

        // Stamp each component into the MNA matrix
        for_each_component([this, &solver, dt](CircuitComponent* comp) {
            comp->stamp(m_pin_to_node, solver, dt);
        });

        // Stamp wire conductances (wires are near-zero resistance conductors)
        for (auto& [id, wire] : m_wires) {
            if (!wire->source || !wire->target) continue;
            int n1 = m_pin_to_node[wire->source];
            int n2 = m_pin_to_node[wire->target];
            if (n1 == n2) continue;  // Same net, skip
            double G_wire = 1.0 / static_cast<double>(wire->resistance);
            solver.add_conductance(n1, n2, G_wire);
        }

        if (!solver.solve()) {
            spdlog::warn("MNA solver failed at iteration {}", iter);
            break;
        }

        // Store this solver for later current extraction
        final_solver = solver;

        // Write solved voltages back to pins and check convergence
        double max_delta = 0.0;
        for_each_component([this, &final_solver, &max_delta](CircuitComponent* comp) {
            for (CircuitPin* pin : comp->get_pins()) {
                if (!pin) continue;
                auto it = m_pin_to_node.find(pin);
                if (it == m_pin_to_node.end()) continue;
                double new_v = final_solver.get_node_voltage(it->second);
                double delta = std::abs(new_v - static_cast<double>(pin->voltage));
                max_delta = (std::max)(max_delta, delta);
                pin->voltage = static_cast<float>(new_v);
            }
        });

        if (max_delta < 1e-6) break;  // Converged
    }

    // Phase E: Compute branch currents using solved voltages
    // For voltage sources, extract the branch current from MNA solver
    int vs_index = 0;
    for_each_component([this, &final_solver, dt, &vs_index](CircuitComponent* comp) {
        // If this is a voltage source, get its branch current from the solver
        if (auto* vs = dynamic_cast<DCVoltageSource*>(comp)) {
            double current = final_solver.get_branch_current(vs_index++);
            auto pins = vs->get_pins();
            if (pins.size() >= 2) {
                // Current flows OUT of V+ (negative in MNA convention)
                pins[0]->current = -static_cast<float>(current);
                pins[1]->current = static_cast<float>(current);
            }
        }
        // Also call update for other components
        comp->update(dt);
    });
}

void CircuitSimulator::reset() {
    for (auto& [id, comp] : m_components_owned) {
        comp->reset();
    }
    for (auto& [id, comp] : m_components_external) {
        comp->reset();
    }
    spdlog::info("Circuit simulator reset");
}

void CircuitSimulator::build_nets() {
    m_pin_to_node.clear();

    // Collect all pins from both owned and external components
    std::vector<CircuitPin*> all_pins;
    for (auto& [id, comp] : m_components_owned) {
        for (CircuitPin* pin : comp->get_pins()) {
            if (pin) all_pins.push_back(pin);
        }
    }
    for (auto& [id, comp] : m_components_external) {
        for (CircuitPin* pin : comp->get_pins()) {
            if (pin) all_pins.push_back(pin);
        }
    }

    if (all_pins.empty()) {
        m_num_nodes = 1;  // Ground only
        return;
    }

    // Union-Find data structures
    int n = static_cast<int>(all_pins.size());
    std::vector<int> parent(n);
    for (int i = 0; i < n; i++) parent[i] = i;

    auto find_root = [&parent](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];  // Path compression
            x = parent[x];
        }
        return x;
    };

    auto union_pins = [&parent, &find_root](int a, int b) {
        int ra = find_root(a), rb = find_root(b);
        if (ra != rb) parent[ra] = rb;
    };

    // Map pin pointer to index
    std::unordered_map<CircuitPin*, int> pin_to_idx;
    for (int i = 0; i < n; i++) {
        pin_to_idx[all_pins[i]] = i;
    }

    // Union all wire-connected pins (bidirectional)
    for (auto& [id, wire] : m_wires) {
        if (!wire->source || !wire->target) continue;
        auto it_s = pin_to_idx.find(wire->source);
        auto it_t = pin_to_idx.find(wire->target);
        if (it_s != pin_to_idx.end() && it_t != pin_to_idx.end()) {
            union_pins(it_s->second, it_t->second);
        }
    }

    // Identify ground nets (pins connected to Ground or PinType::Ground)
    std::unordered_map<int, bool> root_is_ground;
    for (int i = 0; i < n; i++) {
        CircuitPin* pin = all_pins[i];
        if (pin->type == PinType::Ground) {
            int root = find_root(i);
            root_is_ground[root] = true;
        }
    }

    // Assign node indices: ground = 0, others = 1, 2, 3, ...
    std::unordered_map<int, int> root_to_node;
    int next_node = 1;  // 0 reserved for ground

    // First pass: assign ground nodes
    for (int i = 0; i < n; i++) {
        int root = find_root(i);
        if (root_is_ground.count(root) && root_is_ground[root]) {
            root_to_node[root] = 0;  // Ground node
        }
    }

    // Second pass: assign non-ground nodes
    for (int i = 0; i < n; i++) {
        int root = find_root(i);
        if (root_to_node.find(root) == root_to_node.end()) {
            root_to_node[root] = next_node++;
        }
    }

    // Map each pin to its node index
    for (int i = 0; i < n; i++) {
        int root = find_root(i);
        m_pin_to_node[all_pins[i]] = root_to_node[root];
    }

    m_num_nodes = next_node;  // Total number of nodes (including ground=0)
}

std::vector<CircuitComponent*> CircuitSimulator::get_all_components() {
    std::vector<CircuitComponent*> result;
    result.reserve(m_components_owned.size() + m_components_external.size());
    for (auto& [id, comp] : m_components_owned) {
        result.push_back(comp.get());
    }
    for (auto& [id, comp] : m_components_external) {
        result.push_back(comp);
    }
    return result;
}

// ============================================================================
// H-Bridge Implementation
// ============================================================================

HBridge::HBridge(float supply_voltage)
    : m_supply_voltage(supply_voltage)
{
    // Initialize pins
    m_vcc.voltage = supply_voltage;
    m_gnd.voltage = 0.0f;

    spdlog::debug("H-Bridge initialized with {:.1f}V supply", supply_voltage);
}

void HBridge::set_pwm_duty(float duty) {
    m_pwm_duty = std::max(0.0f, std::min(1.0f, duty));
}

void HBridge::update(double dt) {
    // Update PWM phase
    m_pwm_phase += static_cast<float>(dt) * m_pwm_frequency;
    if (m_pwm_phase >= 1.0f) {
        m_pwm_phase -= 1.0f;
    }

    // Use VCC - GND as actual supply voltage
    float supply = m_vcc.voltage - m_gnd.voltage;
    if (supply <= 0.0f) supply = m_supply_voltage;  // Fallback

    // Determine direction from IN1 and IN2
    bool in1 = m_in1.digital_state;
    bool in2 = m_in2.digital_state;
    bool enabled = m_en.digital_state;

    if (!enabled) {
        // Disabled = coast/brake mode
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = 0.0f;
        m_out2.voltage = 0.0f;
        return;
    }

    // H-Bridge truth table:
    // IN1=0, IN2=0: Brake (both outputs low)
    // IN1=0, IN2=1: Reverse (OUT1=0, OUT2=VCC)
    // IN1=1, IN2=0: Forward (OUT1=VCC, OUT2=0)
    // IN1=1, IN2=1: Brake (both outputs high)

    if (!in1 && !in2) {
        // Brake to ground
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = 0.0f;
        m_out2.voltage = 0.0f;
    }
    else if (!in1 && in2) {
        // Reverse
        m_direction = -1;
        float effective_voltage = supply * m_pwm_duty;
        m_output_voltage = -effective_voltage;
        m_out1.voltage = 0.0f;
        m_out2.voltage = effective_voltage;
    }
    else if (in1 && !in2) {
        // Forward
        m_direction = 1;
        float effective_voltage = supply * m_pwm_duty;
        m_output_voltage = effective_voltage;
        m_out1.voltage = effective_voltage;
        m_out2.voltage = 0.0f;
    }
    else { // in1 && in2
        // Brake to VCC
        m_direction = 0;
        m_output_voltage = 0.0f;
        m_out1.voltage = supply;
        m_out2.voltage = supply;
    }
}

void HBridge::reset() {
    m_pwm_phase = 0.0f;
    m_pwm_duty = 0.0f;
    m_direction = 0;
    m_output_voltage = 0.0f;

    m_in1.digital_state = false;
    m_in2.digital_state = false;
    m_en.digital_state = false;

    m_out1.voltage = 0.0f;
    m_out2.voltage = 0.0f;
}

} // namespace mechatron
