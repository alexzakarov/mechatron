#include "TimeManager.hpp"
#include <algorithm>

namespace mechatron {

TimeManager::TimeManager(double physics_step_ms)
    : m_physics_step_us(physics_step_ms * 1000.0) {}

void TimeManager::start() {
    m_state = SimulationState::Running;
    m_sim_time_us = 0.0;
    m_tick = 0;
    m_last_real_time = std::chrono::steady_clock::now();
}

void TimeManager::pause() {
    if (m_state == SimulationState::Running) {
        m_state = SimulationState::Paused;
    }
}

void TimeManager::resume() {
    if (m_state == SimulationState::Paused) {
        m_state = SimulationState::Running;
        m_last_real_time = std::chrono::steady_clock::now();
    }
}

void TimeManager::stop() {
    m_state = SimulationState::Stopped;
    m_sim_time_us = 0.0;
    m_tick = 0;
}

void TimeManager::step() {
    // Advance logical time now and let the orchestrator consume exactly one
    // simulation update on the next frame while the public state remains
    // paused for the UI.
    m_sim_time_us += m_physics_step_us;
    m_tick++;
    m_step_requested = true;
    m_state = SimulationState::Paused;
}

void TimeManager::update() {
    if (m_state == SimulationState::Stopped || m_state == SimulationState::Paused) return;

    if (m_state == SimulationState::Stepping) {
        m_sim_time_us += m_physics_step_us;
        m_tick++;
        m_state = SimulationState::Paused;
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed_us = std::chrono::duration<double, std::micro>(now - m_last_real_time).count();
    m_last_real_time = now;

    double scaled_elapsed = elapsed_us * m_realtime_factor;
    double steps_needed = scaled_elapsed / m_physics_step_us;
    uint32_t steps = static_cast<uint32_t>(std::min(steps_needed, static_cast<double>(m_max_steps_per_frame)));

    for (uint32_t i = 0; i < steps; ++i) {
        m_sim_time_us += m_physics_step_us;
        m_tick++;
    }
}

bool TimeManager::consume_step_request() {
    bool requested = m_step_requested;
    m_step_requested = false;
    return requested;
}

} // namespace mechatron
