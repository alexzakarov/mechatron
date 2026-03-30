#pragma once

namespace mechatron {

class TimelinePanel {
public:
    void render(class SimulationOrchestrator& orchestrator);

private:
    float m_timeline_scale = 1.0f;
};

} // namespace mechatron
