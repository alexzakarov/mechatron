#pragma once

#include "io/SerialPort.hpp"
#include <chrono>
#include <string>
#include <vector>

namespace mechatron {

class PropertiesPanel {
public:
    void render(class SimulationOrchestrator& orchestrator);

    void set_selected(const std::string& id) { m_selected_id = id; }
    const std::string& selected() const { return m_selected_id; }

private:
    void refresh_physical_ports(bool force = false);

    std::string m_selected_id;
    std::vector<SerialPortInfo> m_physical_ports;
    std::chrono::steady_clock::time_point m_last_port_refresh{};
};

} // namespace mechatron
