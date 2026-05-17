#pragma once

#include <cstdint>
#include <functional>
#include <chrono>

namespace mechatron {

enum class SimulationState {
    Stopped,
    Running,
    Paused,
    Stepping
};

class TimeManager {
public:
    explicit TimeManager(double physics_step_ms = 1.0);

    void start();
    void pause();
    void resume();
    void stop();
    void step();

    void update();
    bool consume_step_request();

    double physics_step_size() const { return m_physics_step_us * 1e-6; }
    double physics_step_size_us() const { return m_physics_step_us; }

    double simulation_time() const { return m_sim_time_us * 1e-6; }
    double simulation_time_us() const { return m_sim_time_us; }

    uint64_t current_tick() const { return m_tick; }
    SimulationState state() const { return m_state; }

    void set_realtime_factor(double factor) { m_realtime_factor = factor; }
    double realtime_factor() const { return m_realtime_factor; }

    void set_max_steps_per_frame(uint32_t max) { m_max_steps_per_frame = max; }

    bool is_deterministic() const { return m_deterministic; }
    void set_deterministic(bool det) { m_deterministic = det; }

private:
    double m_physics_step_us;
    double m_sim_time_us = 0.0;
    uint64_t m_tick = 0;
    SimulationState m_state = SimulationState::Stopped;
    double m_realtime_factor = 1.0;
    uint32_t m_max_steps_per_frame = 16;
    bool m_deterministic = true;
    bool m_step_requested = false;

    std::chrono::steady_clock::time_point m_last_real_time;
};

} // namespace mechatron
