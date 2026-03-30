#pragma once

#include <string>

namespace mechatron {

class PropertiesPanel {
public:
    void render(class SimulationOrchestrator& orchestrator);

    void set_selected(const std::string& id) { m_selected_id = id; }
    const std::string& selected() const { return m_selected_id; }

private:
    std::string m_selected_id;
};

} // namespace mechatron
