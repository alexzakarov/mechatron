#include "SoftControlPlugin.hpp"
#include "control/ControlAlgorithms.hpp"
#include <unordered_map>
#include <functional>

namespace mechatron {

// ============================================================================
// Control Algorithm Component Wrappers
// ============================================================================

template<typename Algo>
class ControlAlgorithmAdapter : public Component {
public:
    explicit ControlAlgorithmAdapter(std::unique_ptr<Algo> algo)
        : m_algo(std::move(algo)) {}

    std::string_view plugin_type() const override { return "soft_control"; }
    std::string_view component_type() const override { return m_algo->component_type(); }
    std::string_view category() const override { return m_algo->category(); }

    void update(double dt) override {
        // Default: compute with zero setpoint/measurement
        // Real usage would wire setpoint/measurement via ports
    }

    void serialize(nlohmann::json& out) const override {
        out["algorithm_type"] = std::string(m_algo->component_type());
    }

    void deserialize(const nlohmann::json& in) override {
    }

    Algo* algorithm() { return m_algo.get(); }
    const Algo* algorithm() const { return m_algo.get(); }

protected:
    std::unique_ptr<Algo> m_algo;
};

// PID Controller Component
class PIDControllerComponent : public ControlAlgorithmAdapter<PIDController> {
public:
    PIDControllerComponent()
        : ControlAlgorithmAdapter(std::make_unique<PIDController>()) {}

    void update(double dt) override {
        m_algo->set_sample_time(static_cast<float>(dt));
        m_output = m_algo->compute(m_setpoint, m_measurement);
    }

    float output() const { return m_output; }
    void set_setpoint(float sp) { m_setpoint = sp; }
    void set_measurement(float meas) { m_measurement = meas; }

    void serialize(nlohmann::json& out) const override {
        ControlAlgorithmAdapter::serialize(out);
        auto& p = m_algo->get_params();
        out["kp"] = p.kp;
        out["ki"] = p.ki;
        out["kd"] = p.kd;
    }

    void deserialize(const nlohmann::json& in) override {
        if (in.contains("kp")) m_algo->set_kp(in["kp"]);
        if (in.contains("ki")) m_algo->set_ki(in["ki"]);
        if (in.contains("kd")) m_algo->set_kd(in["kd"]);
    }

private:
    float m_setpoint = 0.0f;
    float m_measurement = 0.0f;
    float m_output = 0.0f;
};

// PI Controller Component
class PIControllerComponent : public ControlAlgorithmAdapter<PIController> {
public:
    PIControllerComponent()
        : ControlAlgorithmAdapter(std::make_unique<PIController>()) {}

    void update(double dt) override {
        auto p = m_algo->get_params();
        p.dt = static_cast<float>(dt);
        m_algo->set_params(p);
        m_output = m_algo->compute(m_setpoint, m_measurement);
    }

    void serialize(nlohmann::json& out) const override {
        ControlAlgorithmAdapter::serialize(out);
        auto& p = m_algo->get_params();
        out["kp"] = p.kp;
        out["ki"] = p.ki;
    }

    void deserialize(const nlohmann::json& in) override {
        auto p = m_algo->get_params();
        if (in.contains("kp")) { p.kp = in["kp"]; }
        if (in.contains("ki")) { p.ki = in["ki"]; }
        m_algo->set_params(p);
    }

private:
    float m_setpoint = 0.0f;
    float m_measurement = 0.0f;
    float m_output = 0.0f;
};

// Lead-Lag Compensator Component
class LeadLagComponent : public ControlAlgorithmAdapter<LeadLagCompensator> {
public:
    LeadLagComponent()
        : ControlAlgorithmAdapter(std::make_unique<LeadLagCompensator>()) {}

    void update(double dt) override {
        auto p = m_algo->get_params();
        p.dt = static_cast<float>(dt);
        m_algo->set_params(p);
        m_output = m_algo->compute(m_setpoint, m_measurement);
    }

    void serialize(nlohmann::json& out) const override {
        ControlAlgorithmAdapter::serialize(out);
        auto& p = m_algo->get_params();
        out["gain"] = p.gain;
        out["zero"] = p.zero;
        out["pole"] = p.pole;
    }

    void deserialize(const nlohmann::json& in) override {
        auto p = m_algo->get_params();
        if (in.contains("gain")) p.gain = in["gain"];
        if (in.contains("zero")) p.zero = in["zero"];
        if (in.contains("pole")) p.pole = in["pole"];
        m_algo->set_params(p);
    }

private:
    float m_setpoint = 0.0f;
    float m_measurement = 0.0f;
    float m_output = 0.0f;
};

// Feedforward Controller Component
class FeedforwardComponent : public ControlAlgorithmAdapter<FeedforwardController> {
public:
    FeedforwardComponent()
        : ControlAlgorithmAdapter(std::make_unique<FeedforwardController>()) {}

    void update(double dt) override {
        m_output = m_algo->compute(m_desired_accel, m_desired_vel, m_desired_pos);
    }

    void serialize(nlohmann::json& out) const override {
        ControlAlgorithmAdapter::serialize(out);
        auto& p = m_algo->get_params();
        out["mass"] = p.mass;
        out["damping"] = p.damping;
    }

    void deserialize(const nlohmann::json& in) override {
        auto p = m_algo->get_params();
        if (in.contains("mass")) p.mass = in["mass"];
        if (in.contains("damping")) p.damping = in["damping"];
        m_algo->set_params(p);
    }

private:
    float m_desired_accel = 0.0f;
    float m_desired_vel = 0.0f;
    float m_desired_pos = 0.0f;
    float m_output = 0.0f;
};

// Trajectory Generator Component
class TrajectoryComponent : public ControlAlgorithmAdapter<TrajectoryGenerator> {
public:
    TrajectoryComponent()
        : ControlAlgorithmAdapter(std::make_unique<TrajectoryGenerator>()) {}

    void update(double dt) override {
        m_time += dt;
        m_position = m_algo->get_position(m_time);
        m_velocity = m_algo->get_velocity(m_time);
        m_complete = m_algo->is_complete(m_time);
    }

    void plan(float start, float target) {
        m_algo->plan(start, target);
        m_time = 0.0;
    }

    float position() const { return m_position; }
    float velocity() const { return m_velocity; }
    bool is_complete() const { return m_complete; }

    void serialize(nlohmann::json& out) const override {
        ControlAlgorithmAdapter::serialize(out);
        out["position"] = m_position;
    }

    void deserialize(const nlohmann::json& in) override {
    }

private:
    double m_time = 0.0;
    float m_position = 0.0f;
    float m_velocity = 0.0f;
    bool m_complete = false;
};

// ============================================================================
// Plugin Implementation
// ============================================================================

std::vector<ComponentDescriptor> SoftControlPlugin::components() const {
    return {
        {"pid_controller", "PID Controller", "control", "PID feedback controller"},
        {"pi_controller", "PI Controller", "control", "PI feedback controller"},
        {"lead_lag", "Lead-Lag Compensator", "control", "Frequency domain compensator"},
        {"state_space", "State Space Controller", "control", "LQR state feedback"},
        {"feedforward", "Feedforward Controller", "control", "Model-based feedforward"},
        {"cascade", "Cascade Controller", "control", "Nested control loops"},
        {"deadbeat", "Deadbeat Controller", "control", "Minimum-time response"},
        {"kalman_filter", "Kalman Filter", "estimator", "Optimal state estimator"},
        {"mpc", "MPC Controller", "control", "Model predictive control"},
        {"trajectory", "Trajectory Generator", "control", "Motion trajectory planning"}
    };
}

std::unique_ptr<Component> SoftControlPlugin::create(std::string_view type) {
    using namespace std;
    static const unordered_map<string, function<unique_ptr<Component>()>> factory = {
        {"pid_controller", []() { return make_unique<PIDControllerComponent>(); }},
        {"pi_controller", []() { return make_unique<PIControllerComponent>(); }},
        {"lead_lag", []() { return make_unique<LeadLagComponent>(); }},
        {"feedforward", []() { return make_unique<FeedforwardComponent>(); }},
        {"trajectory", []() { return make_unique<TrajectoryComponent>(); }},
        // These use the generic adapter (limited functionality but functional)
        {"state_space", []() { return make_unique<ControlAlgorithmAdapter<StateSpaceController>>(make_unique<StateSpaceController>()); }},
        {"cascade", []() {
            auto cc = make_unique<CascadeController>();
            cc->add_loop({PIDController::Params{}, 1e6f});
            cc->add_loop({PIDController::Params{}, 1e6f});
            return make_unique<ControlAlgorithmAdapter<CascadeController>>(std::move(cc));
        }},
        {"deadbeat", []() { return make_unique<ControlAlgorithmAdapter<DeadbeatController>>(make_unique<DeadbeatController>()); }},
        {"kalman_filter", []() { return make_unique<ControlAlgorithmAdapter<KalmanFilter>>(make_unique<KalmanFilter>()); }},
        {"mpc", []() { return make_unique<ControlAlgorithmAdapter<MPCController>>(make_unique<MPCController>()); }}
    };

    auto it = factory.find(string(type));
    if (it != factory.end()) {
        return it->second();
    }
    return nullptr;
}

void SoftControlPlugin::on_register(PluginHost& host) {
}

void SoftControlPlugin::on_unregister() {
}

} // namespace mechatron
