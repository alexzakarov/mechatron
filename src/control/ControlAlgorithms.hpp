#pragma once

#include "core/Types.hpp"
#include <vector>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mechatron {

// ============================================================================
// PID Controller
// ============================================================================

/**
 * @brief PID (Proportional-Integral-Derivative) Controller
 *
 * Classic PID controller implementation with:
 * - Anti-windup clamping
 * - Derivative on measurement (to avoid derivative kick on setpoint changes)
 * - Output limiting
 * - Feedforward support
 *
 * Transfer function: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*d/dt(e(t))
 */
class PIDController {
public:
    struct Params {
        float kp = 1.0f;           // Proportional gain
        float ki = 0.0f;           // Integral gain
        float kd = 0.0f;           // Derivative gain
        float output_min = -1e6f;  // Minimum output
        float output_max = 1e6f;   // Maximum output
        float integral_min = -1e6f;// Integral clamping (anti-windup)
        float integral_max = 1e6f; // Integral clamping (anti-windup)
        float dt = 0.001f;         // Sample time (seconds)
        bool derivative_on_measurement = true; // Use derivative on measurement instead of error
    };

    PIDController();
    explicit PIDController(const Params& params);

    // Reset controller state
    void reset();

    // Calculate control output
    float compute(float setpoint, float measurement);

    // Set parameters
    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    // Individual gain setters
    void set_kp(float kp) { m_params.kp = kp; }
    void set_ki(float ki) { m_params.ki = ki; }
    void set_kd(float kd) { m_params.kd = kd; }

    // Set output limits
    void set_output_limits(float min, float max);

    // Set sample time
    void set_sample_time(float dt) { m_params.dt = dt; }

    // Get current state
    float get_integral() const { return m_integral; }
    float get_derivative() const { return m_derivative; }
    float get_error() const { return m_error; }

    // Enable/disable controller
    void set_enabled(bool enabled) { m_enabled = enabled; }
    bool is_enabled() const { return m_enabled; }

    // Get component info
    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "pid_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    float m_integral = 0.0f;
    float m_derivative = 0.0f;
    float m_error = 0.0f;
    float m_prev_measurement = 0.0f;
    bool m_enabled = true;
    bool m_first_run = true;
};

// ============================================================================
// PI Controller (common in industrial applications)
// ============================================================================

/**
 * @brief PI Controller (Proportional-Integral)
 *
 * Simpler than PID, often sufficient for most applications.
 * Better stability characteristics than full PID.
 */
class PIController {
public:
    struct Params {
        float kp = 1.0f;
        float ki = 0.1f;
        float output_min = -1e6f;
        float output_max = 1e6f;
        float integral_max = 1e6f;  // Anti-windup
        float dt = 0.001f;
    };

    PIController();
    explicit PIController(const Params& params);

    void reset();
    float compute(float setpoint, float measurement);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    void set_kp(float kp) { m_params.kp = kp; }
    void set_ki(float ki) { m_params.ki = ki; }
    void set_output_limits(float min, float max);

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "pi_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    float m_integral = 0.0f;
    float m_prev_error = 0.0f;
    bool m_first_run = true;
};

// ============================================================================
// Lead-Lag Compensator
// ============================================================================

/**
 * @brief Lead-Lag Compensator
 *
 * Transfer function: G(s) = K * (s + z) / (s + p)
 * - Lead (z < p): Improves transient response, increases phase margin
 * - Lag (z > p): Improves steady-state accuracy, reduces steady-state error
 *
 * Commonly used for frequency-domain shaping of control loops.
 */
class LeadLagCompensator {
public:
    struct Params {
        float gain = 1.0f;         // Overall gain K
        float zero = 1.0f;         // Zero location (rad/s)
        float pole = 10.0f;        // Pole location (rad/s)
        float output_min = -1e6f;
        float output_max = 1e6f;
        float dt = 0.001f;
    };

    LeadLagCompensator();
    explicit LeadLagCompensator(const Params& params);

    void reset();
    float compute(float setpoint, float measurement);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    void set_gain(float gain) { m_params.gain = gain; }
    void set_zero(float zero) { m_params.zero = zero; }
    void set_pole(float pole) { m_params.pole = pole; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "lead_lag_compensator"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    float m_input_prev = 0.0f;
    float m_output_prev = 0.0f;
    bool m_first_run = true;
};

// ============================================================================
// State Space Controller
// ============================================================================

/**
 * @brief State Space Controller (LQR-like)
 *
 * Implements state feedback: u = -K * x
 * Can be extended to include integral action and reference tracking.
 *
 * System model: dx/dt = A*x + B*u
 * Control law: u = -K*x + N*r
 */
class StateSpaceController {
public:
    struct Params {
        std::vector<std::vector<float>> A;  // State matrix
        std::vector<std::vector<float>> B;  // Input matrix
        std::vector<std::vector<float>> K;  // Feedback gain matrix
        std::vector<float> N;               // Feedforward gain
        int n_states = 2;
        int n_inputs = 1;
        float dt = 0.001f;
    };

    StateSpaceController();
    explicit StateSpaceController(const Params& params);

    void reset();
    float compute(const std::vector<float>& reference, const std::vector<float>& state);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "state_space_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    std::vector<float> m_state_estimate;
};

// ============================================================================
// Feedforward Controller
// ============================================================================

/**
 * @brief Feedforward Controller
 *
 * Uses model knowledge to anticipate control action.
 * Combined with feedback for best results: u = u_ff + u_fb
 *
 * For motion systems: u_ff = model_inverse(desired_acceleration, velocity, position)
 */
class FeedforwardController {
public:
    struct Params {
        float mass = 1.0f;          // System mass (kg)
        float damping = 0.1f;       // Damping coefficient
        float stiffness = 0.0f;     // Spring stiffness
        float gravity_comp = 0.0f;  // Gravity compensation
        float friction_static = 0.0f;
        float friction_dynamic = 0.0f;
    };

    FeedforwardController();
    explicit FeedforwardController(const Params& params);

    // Compute feedforward for motion control
    float compute(float desired_accel, float desired_vel = 0.0f, float desired_pos = 0.0f);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "feedforward_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
};

// ============================================================================
// Cascade Controller (Position/Velocity loops)
// ============================================================================

/**
 * @brief Cascade Controller
 *
 * Nested control loops where outer loop output becomes inner loop reference.
 * Common structure: Position → Velocity → Current/Torque
 *
 * Faster inner loops reject disturbances before they affect outer loops.
 */
class CascadeController {
public:
    struct LoopConfig {
        PIDController::Params pid_params;
        float output_rate_limit = 1e6f;  // Rate limiter
    };

    CascadeController();

    // Configure loops (typically 2-3 loops)
    void add_loop(const LoopConfig& config);
    void clear_loops();

    // Reset all loops
    void reset();

    // Compute through cascade (outermost reference, innermost output)
    float compute(float setpoint, const std::vector<float>& measurements);

    // Access individual loops
    PIDController& get_loop(size_t index) { return m_loops[index]; }
    const PIDController& get_loop(size_t index) const { return m_loops[index]; }
    size_t loop_count() const { return m_loops.size(); }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "cascade_controller"; }
    std::string_view category() const { return "control"; }

private:
    std::vector<PIDController> m_loops;
    std::vector<float> m_loop_outputs;
};

// ============================================================================
// Deadbeat Controller
// ============================================================================

/**
 * @brief Deadbeat Controller (Discrete-time minimum-time response)
 *
 * Achieves zero error in minimum number of sampling periods.
 * Requires accurate plant model. Sensitive to model mismatches.
 *
 * For a first-order system: G(z) = (b*z^-1) / (1 - a*z^-1)
 * Deadbeat controller: D(z) = (1-a) / (b*(1-z^-1))
 */
class DeadbeatController {
public:
    struct Params {
        float a = 0.9f;  // System pole (discrete)
        float b = 0.1f;  // System zero
        float dt = 0.001f;
    };

    DeadbeatController();
    explicit DeadbeatController(const Params& params);

    void reset();
    float compute(float setpoint, float measurement);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "deadbeat_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    float m_prev_output = 0.0f;
    bool m_first_run = true;
};

// ============================================================================
// Kalman Filter (State Estimator)
// ============================================================================

/**
 * @brief Kalman Filter
 *
 * Optimal state estimator for linear systems with Gaussian noise.
 * Predicts state using model, then corrects using measurements.
 *
 * Often used with state feedback control (LQG).
 */
class KalmanFilter {
public:
    struct Params {
        int n_states = 2;
        int n_inputs = 1;
        int n_outputs = 1;
        std::vector<std::vector<float>> A;  // State transition
        std::vector<std::vector<float>> B;  // Input matrix
        std::vector<std::vector<float>> C;  // Output matrix
        std::vector<std::vector<float>> Q;  // Process noise covariance
        std::vector<std::vector<float>> R;  // Measurement noise covariance
        std::vector<std::vector<float>> P;  // Estimate error covariance
        float dt = 0.001f;
    };

    KalmanFilter();
    explicit KalmanFilter(const Params& params);

    void reset();
    void predict(const std::vector<float>& input = {});
    void update(const std::vector<float>& measurement);

    const std::vector<float>& get_state() const { return m_state; }
    void set_initial_state(const std::vector<float>& x0);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "kalman_filter"; }
    std::string_view category() const { return "estimator"; }

private:
    Params m_params;
    std::vector<float> m_state;
    std::vector<std::vector<float>> m_P;  // Current covariance
};

// ============================================================================
// Model Predictive Control (MPC) - Basic Implementation
// ============================================================================

/**
 * @brief Model Predictive Controller (simplified)
 *
 * Solves an optimization problem at each time step to minimize a cost function
 * over a prediction horizon, subject to constraints.
 *
 * This is a basic implementation for linear systems with quadratic cost.
 */
class MPCController {
public:
    struct Params {
        int horizon = 10;           // Prediction horizon
        int n_states = 2;
        int n_inputs = 1;
        float dt = 0.001f;
        std::vector<std::vector<float>> A;
        std::vector<std::vector<float>> B;
        std::vector<std::vector<float>> Q;  // State cost
        std::vector<std::vector<float>> R;  // Input cost
        std::vector<float> input_min;
        std::vector<float> input_max;
        std::vector<float> state_min;
        std::vector<float> state_max;
    };

    MPCController();
    explicit MPCController(const Params& params);

    void reset();
    float compute(const std::vector<float>& reference, const std::vector<float>& state);

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "mpc_controller"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    std::vector<float> m_predicted_state;
};

// ============================================================================
// Trajectory Generator
// ============================================================================

/**
 * @brief Trajectory Generator
 *
 * Generates smooth reference trajectories for motion control.
 * Supports: step, ramp, polynomial (minimum-time), S-curve
 */
class TrajectoryGenerator {
public:
    enum class Type {
        Step,
        Ramp,
        Polynomial,  // 3rd or 5th order polynomial
        SCurve       // S-curve (limited jerk)
    };

    struct Params {
        Type type = Type::Polynomial;
        float max_velocity = 1.0f;
        float max_acceleration = 5.0f;
        float max_jerk = 50.0f;
        float dt = 0.001f;
        int polynomial_order = 5;  // 3 or 5
    };

    TrajectoryGenerator();
    explicit TrajectoryGenerator(const Params& params);

    // Start a new trajectory from current to target position
    void plan(float start_pos, float target_pos, float start_vel = 0.0f, float target_vel = 0.0f);

    // Get reference at current time
    float get_position(double t) const;
    float get_velocity(double t) const;
    float get_acceleration(double t) const;

    // Check if trajectory is complete
    bool is_complete(double t) const;
    double get_duration() const { return m_duration; }

    void set_params(const Params& params) { m_params = params; }
    const Params& get_params() const { return m_params; }

    std::string_view plugin_type() const { return "soft_control"; }
    std::string_view component_type() const { return "trajectory_generator"; }
    std::string_view category() const { return "control"; }

private:
    Params m_params;
    double m_duration = 0.0;
    float m_start_pos = 0.0f;
    float m_target_pos = 0.0f;
    float m_start_vel = 0.0f;
    float m_target_vel = 0.0f;

    // Polynomial coefficients
    float m_a0 = 0, m_a1 = 0, m_a2 = 0, m_a3 = 0, m_a4 = 0, m_a5 = 0;

    // S-curve phases
    struct SCurvePhase {
        double t1, t2, t3;  // Acceleration times
        double d1, d2, d3;  // Distances
    } m_scurve;
};

// ============================================================================
// Control System Utilities
// ============================================================================

namespace ControlUtils {

// Bode plot analysis (for frequency response)
struct BodeData {
    std::vector<float> frequencies;    // Hz
    std::vector<float> magnitude;      // dB
    std::vector<float> phase;          // degrees
};

// Step response analysis
struct StepResponseMetrics {
    float rise_time;        // 10-90% rise time
    float settling_time;    // 2% settling time
    float overshoot;        // Percent
    float peak_time;        // Time to peak
    float steady_state_error;
    float peak_value;
};

StepResponseMetrics analyze_step_response(const std::vector<float>& response,
                                          float dt, float final_value);

// Frequency response computation
BodeData compute_frequency_response(const std::vector<float>& num,
                                    const std::vector<float>& den,
                                    const std::vector<float>& frequencies);

// PID tuning methods
struct PIDTuning {
    float kp, ki, kd;
};

// Ziegler-Nichols tuning (based on ultimate gain and period)
PIDTuning ziegler_nichols(float ku, float tu, const std::string& type = "pid");

// Cohen-Coon tuning (for first-order plus dead-time processes)
PIDTuning cohen_coon(float K, float T, float L, const std::string& type = "pid");

// Pole placement
std::vector<float> place_poles(const std::vector<std::vector<float>>& A,
                                const std::vector<std::vector<float>>& B,
                                const std::vector<float>& desired_poles);

} // namespace ControlUtils

} // namespace mechatron
