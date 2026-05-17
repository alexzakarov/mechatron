#include "ControlAlgorithms.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace mechatron {

// ============================================================================
// PID Controller Implementation
// ============================================================================

PIDController::PIDController(const Params& params)
    : m_params(params)
{
}

PIDController::PIDController()
    : PIDController(Params())
{
}

void PIDController::reset()
{
    m_integral = 0.0f;
    m_derivative = 0.0f;
    m_error = 0.0f;
    m_prev_measurement = 0.0f;
    m_first_run = true;
}

void PIDController::set_output_limits(float min, float max)
{
    m_params.output_min = min;
    m_params.output_max = max;
}

float PIDController::compute(float setpoint, float measurement)
{
    if (!m_enabled) {
        return 0.0f;
    }

    // Calculate error
    m_error = setpoint - measurement;

    // === Error clamping for large step changes to prevent integral windup ===
    // When error is very large (e.g., stepping from 176° to 0°), clamp it to
    // prevent the integral from accumulating too rapidly. This is a form of
    // "error band limiting" or "setpoint ramping" approach.
    // The max_error allows full proportional action but limits integral buildup.
    float error_for_integral = m_error;
    if (error_for_integral > m_params.max_error) {
        error_for_integral = m_params.max_error;
    } else if (error_for_integral < -m_params.max_error) {
        error_for_integral = -m_params.max_error;
    }

    // Proportional term (use full error for immediate response)
    float p_term = m_params.kp * m_error;

    // Integral term with anti-windup clamping (use clamped error)
    m_integral += error_for_integral * m_params.dt;

    // Clamp integral to prevent windup
    if (m_integral > m_params.integral_max) {
        m_integral = m_params.integral_max;
    } else if (m_integral < m_params.integral_min) {
        m_integral = m_params.integral_min;
    }

    float i_term = m_params.ki * m_integral;

    // Derivative term
    if (m_params.derivative_on_measurement) {
        // Derivative on measurement to avoid derivative kick on setpoint changes
        if (m_first_run) {
            m_derivative = 0.0f;
            m_first_run = false;
        } else {
            m_derivative = (measurement - m_prev_measurement) / m_params.dt;
        }
        m_prev_measurement = measurement;
    } else {
        // Derivative on error
        if (m_first_run) {
            m_derivative = 0.0f;
            m_first_run = false;
        } else {
            m_derivative = (m_error - m_prev_measurement) / m_params.dt;
        }
        m_prev_measurement = m_error;
    }

    float d_term = m_params.kd * m_derivative;

    // Calculate output
    float output = p_term + i_term - d_term;  // Note: -d_term for proper sign

    // Clamp output with improved back-calculation
    if (output > m_params.output_max) {
        output = m_params.output_max;
        // Only apply back-calculation if integral is contributing significantly
        // This prevents the integral from being incorrectly reduced due to large p_term
        float output_without_i = p_term - d_term;
        if (m_params.ki != 0.0f && i_term > 0) {
            // Calculate what the integral should be to achieve output_max
            m_integral = (m_params.output_max - output_without_i) / m_params.ki;
            // Re-clamp to ensure we don't exceed limits
            m_integral = std::max(m_params.integral_min, std::min(m_params.integral_max, m_integral));
        }
    } else if (output < m_params.output_min) {
        output = m_params.output_min;
        float output_without_i = p_term - d_term;
        if (m_params.ki != 0.0f && i_term < 0) {
            m_integral = (m_params.output_min - output_without_i) / m_params.ki;
            m_integral = std::max(m_params.integral_min, std::min(m_params.integral_max, m_integral));
        }
    }

    return output;
}

// ============================================================================
// PI Controller Implementation
// ============================================================================

PIController::PIController(const Params& params)
    : m_params(params)
{
}

PIController::PIController()
    : PIController(Params())
{
}

void PIController::reset()
{
    m_integral = 0.0f;
    m_prev_error = 0.0f;
    m_first_run = true;
}

void PIController::set_output_limits(float min, float max)
{
    m_params.output_min = min;
    m_params.output_max = max;
}

float PIController::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // Proportional term
    float p_term = m_params.kp * error;

    // Integral term with trapezoidal integration
    if (m_first_run) {
        m_integral = error * m_params.dt;
        m_first_run = false;
    } else {
        m_integral += (error + m_prev_error) * 0.5f * m_params.dt;
    }
    m_prev_error = error;

    // Anti-windup
    if (m_integral > m_params.integral_max) {
        m_integral = m_params.integral_max;
    } else if (m_integral < -m_params.integral_max) {
        m_integral = -m_params.integral_max;
    }

    float i_term = m_params.ki * m_integral;

    float output = p_term + i_term;

    // Clamp output
    if (output > m_params.output_max) {
        output = m_params.output_max;
    } else if (output < m_params.output_min) {
        output = m_params.output_min;
    }

    return output;
}

// ============================================================================
// Lead-Lag Compensator Implementation
// ============================================================================

LeadLagCompensator::LeadLagCompensator(const Params& params)
    : m_params(params)
{
}

LeadLagCompensator::LeadLagCompensator()
    : LeadLagCompensator(Params())
{
}

void LeadLagCompensator::reset()
{
    m_input_prev = 0.0f;
    m_output_prev = 0.0f;
    m_first_run = true;
}

float LeadLagCompensator::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    if (m_first_run) {
        m_first_run = false;
        m_input_prev = error;
        return m_params.gain * error;
    }

    // Discretize using bilinear transform (Tustin approximation)
    // For H(s) = K*(s+z)/(s+p), discretized:
    // H(z) = K * (2+dt*z)/(2+dt*p) * (1 - (2-dt*z)/(2+dt*z)*z^-1) / (1 - (2-dt*p)/(2+dt*p)*z^-1)

    float dt = m_params.dt;
    float alpha_z = (2.0f - dt * m_params.zero) / (2.0f + dt * m_params.zero);
    float alpha_p = (2.0f - dt * m_params.pole) / (2.0f + dt * m_params.pole);
    float K = m_params.gain * (2.0f + dt * m_params.zero) / (2.0f + dt * m_params.pole);

    // Output difference equation: y[n] = K*(x[n] - alpha_z*x[n-1]) + alpha_p*y[n-1]
    float output = K * (error - alpha_z * m_input_prev) + alpha_p * m_output_prev;

    m_input_prev = error;
    m_output_prev = output;

    // Clamp output
    if (output > m_params.output_max) {
        output = m_params.output_max;
    } else if (output < m_params.output_min) {
        output = m_params.output_min;
    }

    return output;
}

// ============================================================================
// State Space Controller Implementation
// ============================================================================

StateSpaceController::StateSpaceController(const Params& params)
    : m_params(params)
{
    m_state_estimate.resize(params.n_states, 0.0f);
}

StateSpaceController::StateSpaceController()
    : StateSpaceController(Params())
{
}

void StateSpaceController::reset()
{
    std::fill(m_state_estimate.begin(), m_state_estimate.end(), 0.0f);
}

float StateSpaceController::compute(const std::vector<float>& reference, const std::vector<float>& state)
{
    // State feedback control: u = -K*x + N*r

    if (state.size() != static_cast<size_t>(m_params.n_states)) {
        spdlog::warn("State dimension mismatch in StateSpaceController");
        return 0.0f;
    }

    // Compute u = -K * x
    float u = 0.0f;
    for (int i = 0; i < m_params.n_inputs; ++i) {
        float state_feedback = 0.0f;
        for (int j = 0; j < m_params.n_states; ++j) {
            state_feedback += m_params.K[i][j] * state[j];
        }
        u -= state_feedback;
    }

    // Add feedforward: u += N * r
    if (!m_params.N.empty()) {
        float feedforward = 0.0f;
        for (size_t i = 0; i < std::min(m_params.N.size(), reference.size()); ++i) {
            feedforward += m_params.N[i] * reference[i];
        }
        u += feedforward;
    }

    return u;
}

// ============================================================================
// Feedforward Controller Implementation
// ============================================================================

FeedforwardController::FeedforwardController(const Params& params)
    : m_params(params)
{
}

FeedforwardController::FeedforwardController()
    : FeedforwardController(Params())
{
}

float FeedforwardController::compute(float desired_accel, float desired_vel, float desired_pos)
{
    // Force = mass*accel + damping*velocity + stiffness*position + gravity + friction

    float force = m_params.mass * desired_accel
                + m_params.damping * desired_vel
                + m_params.stiffness * desired_pos
                + m_params.gravity_comp;

    // Add friction
    float friction = 0.0f;
    if (std::abs(desired_vel) > 1e-6f) {
        friction = m_params.friction_dynamic * std::copysign(1.0f, desired_vel);
    } else if (std::abs(force) > m_params.friction_static) {
        friction = m_params.friction_static * std::copysign(1.0f, force);
    }
    force += friction;

    return force;
}

// ============================================================================
// Cascade Controller Implementation
// ============================================================================

CascadeController::CascadeController()
{
    m_loops.reserve(3);  // Typical: position, velocity, current
}

void CascadeController::add_loop(const LoopConfig& config)
{
    m_loops.emplace_back(config.pid_params);
    m_loop_outputs.push_back(0.0f);
}

void CascadeController::clear_loops()
{
    m_loops.clear();
    m_loop_outputs.clear();
}

void CascadeController::reset()
{
    for (auto& loop : m_loops) {
        loop.reset();
    }
    std::fill(m_loop_outputs.begin(), m_loop_outputs.end(), 0.0f);
}

float CascadeController::compute(float setpoint, const std::vector<float>& measurements)
{
    if (m_loops.empty()) {
        return 0.0f;
    }

    if (measurements.size() != m_loops.size()) {
        spdlog::warn("Measurement count mismatch in CascadeController: {} vs {}",
                     measurements.size(), m_loops.size());
        return 0.0f;
    }

    // Outermost loop receives the setpoint
    float reference = setpoint;

    // Compute each loop in cascade
    for (size_t i = 0; i < m_loops.size(); ++i) {
        m_loop_outputs[i] = m_loops[i].compute(reference, measurements[i]);

        // Inner loop's reference becomes outer loop's output
        reference = m_loop_outputs[i];
    }

    // Return innermost loop output
    return m_loop_outputs.back();
}

// ============================================================================
// Deadbeat Controller Implementation
// ============================================================================

DeadbeatController::DeadbeatController(const Params& params)
    : m_params(params)
{
}

DeadbeatController::DeadbeatController()
    : DeadbeatController(Params())
{
}

void DeadbeatController::reset()
{
    m_prev_output = 0.0f;
    m_first_run = true;
}

float DeadbeatController::compute(float setpoint, float measurement)
{
    float error = setpoint - measurement;

    if (m_first_run) {
        m_first_run = false;
        m_prev_output = error;
        return m_prev_output;
    }

    // Deadbeat control law for first-order system: y[n] = a*y[n-1] + b*u[n-1]
    // Controller: u[n] = (1-a)/b * (r[n] - y[n]) + a*u[n-1]

    float output = ((1.0f - m_params.a) / m_params.b) * error + m_params.a * m_prev_output;

    m_prev_output = output;
    return output;
}

// ============================================================================
// Kalman Filter Implementation
// ============================================================================

KalmanFilter::KalmanFilter(const Params& params)
    : m_params(params)
{
    m_state.resize(params.n_states, 0.0f);
    m_P = params.P;
}

KalmanFilter::KalmanFilter()
    : KalmanFilter(Params())
{
}

void KalmanFilter::reset()
{
    std::fill(m_state.begin(), m_state.end(), 0.0f);
    m_P = m_params.P;
}

void KalmanFilter::set_initial_state(const std::vector<float>& x0)
{
    if (x0.size() == m_state.size()) {
        m_state = x0;
    }
}

void KalmanFilter::predict(const std::vector<float>& input)
{
    // State prediction: x_hat[k|k-1] = A*x_hat[k-1|k-1] + B*u[k-1]
    std::vector<float> state_pred(m_params.n_states, 0.0f);

    for (int i = 0; i < m_params.n_states; ++i) {
        for (int j = 0; j < m_params.n_states; ++j) {
            state_pred[i] += m_params.A[i][j] * m_state[j];
        }
        if (!input.empty() && i < static_cast<int>(input.size())) {
            for (int j = 0; j < m_params.n_inputs; ++j) {
                state_pred[i] += m_params.B[i][j] * input[j];
            }
        }
    }

    // Covariance prediction: P[k|k-1] = A*P[k-1|k-1]*A' + Q
    std::vector<std::vector<float>> P_pred(m_params.n_states, std::vector<float>(m_params.n_states, 0.0f));

    for (int i = 0; i < m_params.n_states; ++i) {
        for (int j = 0; j < m_params.n_states; ++j) {
            for (int k = 0; k < m_params.n_states; ++k) {
                for (int l = 0; l < m_params.n_states; ++l) {
                    P_pred[i][j] += m_params.A[i][k] * m_P[k][l] * m_params.A[j][l];
                }
            }
            P_pred[i][j] += m_params.Q[i][j];
        }
    }

    m_state = state_pred;
    m_P = P_pred;
}

void KalmanFilter::update(const std::vector<float>& measurement)
{
    // Measurement prediction: y_hat = C*x_hat
    std::vector<float> y_pred(m_params.n_outputs, 0.0f);
    for (int i = 0; i < m_params.n_outputs; ++i) {
        for (int j = 0; j < m_params.n_states; ++j) {
            y_pred[i] += m_params.C[i][j] * m_state[j];
        }
    }

    // Innovation: e = y - y_hat
    std::vector<float> innovation(m_params.n_outputs);
    for (int i = 0; i < m_params.n_outputs; ++i) {
        innovation[i] = measurement[i] - y_pred[i];
    }

    // Innovation covariance: S = C*P*C' + R
    std::vector<std::vector<float>> S(m_params.n_outputs, std::vector<float>(m_params.n_outputs, 0.0f));
    for (int i = 0; i < m_params.n_outputs; ++i) {
        for (int j = 0; j < m_params.n_outputs; ++j) {
            for (int k = 0; k < m_params.n_states; ++k) {
                for (int l = 0; l < m_params.n_states; ++l) {
                    S[i][j] += m_params.C[i][k] * m_P[k][l] * m_params.C[j][l];
                }
            }
            S[i][j] += m_params.R[i][j];
        }
    }

    // Kalman gain: K = P*C'*S^-1 (simplified - assume S is diagonal for small n_outputs)
    std::vector<std::vector<float>> K(m_params.n_states, std::vector<float>(m_params.n_outputs, 0.0f));
    for (int i = 0; i < m_params.n_states; ++i) {
        for (int j = 0; j < m_params.n_outputs; ++j) {
            for (int k = 0; k < m_params.n_states; ++k) {
                K[i][j] += m_P[i][k] * m_params.C[j][k];
            }
            K[i][j] /= S[j][j];  // Simplified for diagonal S
        }
    }

    // State update: x = x + K*e
    for (int i = 0; i < m_params.n_states; ++i) {
        for (int j = 0; j < m_params.n_outputs; ++j) {
            m_state[i] += K[i][j] * innovation[j];
        }
    }

    // Covariance update: P = (I - K*C)*P
    std::vector<std::vector<float>> P_new(m_params.n_states, std::vector<float>(m_params.n_states, 0.0f));
    for (int i = 0; i < m_params.n_states; ++i) {
        for (int j = 0; j < m_params.n_states; ++j) {
            float val = m_P[i][j];
            for (int k = 0; k < m_params.n_outputs; ++k) {
                val -= K[i][k] * m_params.C[k][j] * m_P[i][j];
            }
            P_new[i][j] = val;
        }
    }
    m_P = P_new;
}

// ============================================================================
// MPC Controller Implementation
// ============================================================================

MPCController::MPCController(const Params& params)
    : m_params(params)
{
    m_predicted_state.resize(params.n_states, 0.0f);
}

MPCController::MPCController()
    : MPCController(Params())
{
}

void MPCController::reset()
{
    std::fill(m_predicted_state.begin(), m_predicted_state.end(), 0.0f);
}

float MPCController::compute(const std::vector<float>& reference, const std::vector<float>& state)
{
    // Model Predictive Control implementation
    // Solves: min sum(x[k]'*Q*x[k] + u[k]'*R*u[k]) for k=0 to horizon
    // Subject to: x[k+1] = A*x[k] + B*u[k]
    //
    // This implementation uses an analytical solution for the unconstrained case.
    // For constrained MPC, a QP solver would be required.

    if (state.empty() || m_params.n_states == 0 || m_params.n_inputs == 0) {
        return 0.0f;
    }

    // Default to identity matrices if not provided
    auto& A = m_params.A;
    auto& B = m_params.B;
    auto& Q = m_params.Q;
    auto& R = m_params.R;

    // If matrices are not defined, use simple proportional control as fallback
    if (A.empty() || B.empty()) {
        float primary_ref = reference.empty() ? 0.0f : reference[0];
        float error = primary_ref - state[0];
        return error * 5.0f; // Default gain
    }

    int n_states = m_params.n_states;
    int n_inputs = m_params.n_inputs;
    int horizon = m_params.horizon;

    // Ensure Q and R matrices exist
    if (Q.empty()) {
        // Default: identity for Q
        m_params.Q.assign(n_states, std::vector<float>(n_states, 0.0f));
        for (int i = 0; i < n_states; ++i) m_params.Q[i][i] = 1.0f;
    }
    if (R.empty()) {
        // Default: small value for R (penalize input less than state)
        m_params.R.assign(n_inputs, std::vector<float>(n_inputs, 0.0f));
        for (int i = 0; i < n_inputs; ++i) m_params.R[i][i] = 0.1f;
    }

    // Predict states over horizon and compute cost gradient
    // Using simplified approach: compute gradient of cost function
    // J = sum((x_ref - x_k)'*Q*(x_ref - x_k) + u_k'*R*u_k)
    //
    // For unconstrained case: dJ/du = 0 gives optimal u
    //
    // This is a simplified version that computes:
    // 1. Predicted error trajectory
    // 2. Control action that minimizes predicted error

    // Initialize predicted state
    m_predicted_state = state;

    std::vector<float> gradient(n_inputs, 0.0f);
    std::vector<float> hessian_diag(n_inputs, 0.0f);

    // Predict over horizon and accumulate gradient
    for (int k = 0; k < horizon; ++k) {
        // Get reference for this step (repeat last reference if needed)
        float ref_k = reference.empty() ? 0.0f :
                      (k < static_cast<int>(reference.size()) ? reference[k] : reference.back());

        // Compute error
        std::vector<float> error(n_states);
        for (int i = 0; i < n_states; ++i) {
            error[i] = ref_k - m_predicted_state[i];
        }

        // Compute gradient contribution: dJ/du = -2*B'*Q*error
        // For single input: gradient[0] += -2 * sum(B[i] * Q[i][i] * error[i])
        for (int i = 0; i < n_states; ++i) {
            for (int j = 0; j < n_inputs; ++j) {
                // B[i][j] * Q[i][i] * error[i]
                float b_q_e = B[i][j] * Q[i][i] * error[i];
                gradient[j] += 2.0f * b_q_e;
                // Hessian diagonal: d2J/du2 = 2 * (B'*Q*B + R)
                hessian_diag[j] += 2.0f * (B[i][j] * Q[i][i] * B[i][j] + R[j][j]);
            }
        }

        // Predict next state: x[k+1] = A*x[k] + B*u
        // For gradient computation, we use current predicted state
        std::vector<float> next_state(n_states, 0.0f);
        for (int i = 0; i < n_states; ++i) {
            for (int j = 0; j < n_states; ++j) {
                next_state[i] += A[i][j] * m_predicted_state[j];
            }
        }
        m_predicted_state = next_state;
    }

    // Compute optimal control: u* = -H^(-1) * g
    // For diagonal case: u[i] = -gradient[i] / hessian_diag[i]
    std::vector<float> u_opt(n_inputs);
    for (int i = 0; i < n_inputs; ++i) {
        if (std::abs(hessian_diag[i]) > 1e-6f) {
            u_opt[i] = -gradient[i] / hessian_diag[i];
        } else {
            u_opt[i] = 0.0f;
        }

        // Clamp to input limits
        if (!m_params.input_max.empty() && i < static_cast<int>(m_params.input_max.size())) {
            u_opt[i] = std::min(u_opt[i], m_params.input_max[i]);
        }
        if (!m_params.input_min.empty() && i < static_cast<int>(m_params.input_min.size())) {
            u_opt[i] = std::max(u_opt[i], m_params.input_min[i]);
        }
    }

    // Return first control input
    return u_opt.empty() ? 0.0f : u_opt[0];
}

// ============================================================================
// Trajectory Generator Implementation
// ============================================================================

TrajectoryGenerator::TrajectoryGenerator(const Params& params)
    : m_params(params)
{
}

TrajectoryGenerator::TrajectoryGenerator()
    : TrajectoryGenerator(Params())
{
}

void TrajectoryGenerator::plan(float start_pos, float target_pos, float start_vel, float target_vel)
{
    m_start_pos = start_pos;
    m_target_pos = target_pos;
    m_start_vel = start_vel;
    m_target_vel = target_vel;

    float distance = target_pos - start_pos;

    switch (m_params.type) {
        case Type::Step:
            m_duration = 0.0;
            break;

        case Type::Ramp:
            // t = d / v_max
            m_duration = std::abs(distance) / m_params.max_velocity;
            break;

        case Type::Polynomial: {
            // Minimum-time polynomial (3rd or 5th order)
            // For 5th order: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            // Boundary conditions: q(0), q'(0), q''(0), q(T), q'(T), q''(T) = 0

            // Estimate duration based on velocity and acceleration limits
            float t_min = std::sqrt(std::abs(distance) * 6.0f / m_params.max_acceleration);
            m_duration = t_min;

            if (m_params.polynomial_order == 5) {
                // Solve for 5th order polynomial coefficients
                float T = static_cast<float>(m_duration);
                float T2 = T * T;
                float T3 = T2 * T;
                float T4 = T3 * T;
                float T5 = T4 * T;

                // Boundary conditions with zero start/end velocity and acceleration
                m_a0 = start_pos;
                m_a1 = start_vel;
                m_a2 = 0.0f;

                // Solve remaining coefficients for target position and zero end velocity/accel
                // This is a simplified solution
                m_a3 = (10 * distance - 4 * T * target_vel) / T3;
                m_a4 = (-15 * distance + 7 * T * target_vel) / T4;
                m_a5 = (6 * distance - 3 * T * target_vel) / T5;
            } else {
                // 3rd order polynomial
                float T = static_cast<float>(m_duration);
                float T2 = T * T;
                float T3 = T2 * T;

                m_a0 = start_pos;
                m_a1 = start_vel;
                m_a2 = -3 * start_pos / T2 - 2 * start_vel / T + 3 * target_pos / T2 - target_vel / T;
                m_a3 = 2 * start_pos / T3 + start_vel / T2 - 2 * target_pos / T3 + target_vel / T2;
                m_a4 = 0.0f;
                m_a5 = 0.0f;
            }
            break;
        }

        case Type::SCurve: {
            // S-curve trajectory with limited jerk
            float d = std::abs(distance);
            float v_max = m_params.max_velocity;
            float a_max = m_params.max_acceleration;
            float j_max = m_params.max_jerk;

            // Phase times
            m_scurve.t1 = a_max / j_max;  // Acceleration ramp-up
            m_scurve.t2 = v_max / a_max - m_scurve.t1;  // Constant acceleration
            m_scurve.t3 = m_scurve.t1;  // Acceleration ramp-down

            // Handle case where we don't reach max velocity
            if (m_scurve.t2 < 0) {
                m_scurve.t1 = std::sqrt(v_max / j_max);
                m_scurve.t2 = 0.0f;
                m_scurve.t3 = m_scurve.t1;
            }

            float t_accel = m_scurve.t1 + m_scurve.t2 + m_scurve.t3;
            float d_accel = 0.5f * v_max * t_accel;

            if (2 * d_accel > d) {
                // Triangle velocity profile (don't reach max velocity)
                m_scurve.t1 = std::pow(d / (2 * j_max), 1.0f / 3);
                m_scurve.t2 = 0.0f;
                m_scurve.t3 = m_scurve.t1;
                m_duration = 2 * m_scurve.t1;
            } else {
                // Trapezoidal velocity profile
                float t_const = (d - 2 * d_accel) / v_max;
                m_duration = 2 * t_accel + t_const;
            }
            break;
        }
    }
}

float TrajectoryGenerator::get_position(double t) const
{
    t = std::min(t, m_duration);

    switch (m_params.type) {
        case Type::Step:
            return (t >= 0.5 * m_duration) ? m_target_pos : m_start_pos;

        case Type::Ramp:
            if (t <= 0) return m_start_pos;
            return m_start_pos + (m_target_pos - m_start_pos) * (t / m_duration);

        case Type::Polynomial:
        case Type::SCurve: {
            float T = static_cast<float>(t);
            float T2 = T * T;
            float T3 = T2 * T;
            float T4 = T3 * T;
            float T5 = T4 * T;
            return m_a0 + m_a1 * T + m_a2 * T2 + m_a3 * T3 + m_a4 * T4 + m_a5 * T5;
        }
    }

    return m_start_pos;
}

float TrajectoryGenerator::get_velocity(double t) const
{
    t = std::min(t, m_duration);

    switch (m_params.type) {
        case Type::Step:
            return 0.0f;

        case Type::Ramp:
            return (m_target_pos - m_start_pos) / m_duration;

        case Type::Polynomial:
        case Type::SCurve: {
            float T = static_cast<float>(t);
            float T2 = T * T;
            float T3 = T2 * T;
            float T4 = T3 * T;
            return m_a1 + 2 * m_a2 * T + 3 * m_a3 * T2 + 4 * m_a4 * T3 + 5 * m_a5 * T4;
        }
    }

    return 0.0f;
}

float TrajectoryGenerator::get_acceleration(double t) const
{
    t = std::min(t, m_duration);

    switch (m_params.type) {
        case Type::Step:
        case Type::Ramp:
            return 0.0f;

        case Type::Polynomial:
        case Type::SCurve: {
            float T = static_cast<float>(t);
            float T2 = T * T;
            float T3 = T2 * T;
            return 2 * m_a2 + 6 * m_a3 * T + 12 * m_a4 * T2 + 20 * m_a5 * T3;
        }
    }

    return 0.0f;
}

bool TrajectoryGenerator::is_complete(double t) const
{
    return t >= m_duration;
}

// ============================================================================
// Control System Utilities
// ============================================================================

namespace ControlUtils {

StepResponseMetrics analyze_step_response(const std::vector<float>& response,
                                          float dt, float final_value)
{
    StepResponseMetrics metrics{};
    if (response.empty()) {
        return metrics;
    }

    float peak_value = response[0];
    float peak_time = 0.0f;

    // Find peak
    for (size_t i = 0; i < response.size(); ++i) {
        if (response[i] > peak_value) {
            peak_value = response[i];
            peak_time = i * dt;
        }
    }
    metrics.peak_value = peak_value;
    metrics.peak_time = peak_time;

    // Calculate overshoot
    metrics.overshoot = ((peak_value - final_value) / final_value) * 100.0f;

    // Rise time (10-90%)
    float val_10 = 0.1f * final_value;
    float val_90 = 0.9f * final_value;
    float t_10 = 0.0f, t_90 = 0.0f;

    for (size_t i = 0; i < response.size(); ++i) {
        if (t_10 == 0.0f && response[i] >= val_10) {
            t_10 = i * dt;
        }
        if (t_90 == 0.0f && response[i] >= val_90) {
            t_90 = i * dt;
        }
    }
    metrics.rise_time = t_90 - t_10;

    // Settling time (2% band)
    float band = 0.02f * final_value;
    float settle_lower = final_value - band;
    float settle_upper = final_value + band;

    for (size_t i = response.size() - 1; i > 0; --i) {
        if (response[i] < settle_lower || response[i] > settle_upper) {
            metrics.settling_time = i * dt;
            break;
        }
    }

    // Steady state error
    metrics.steady_state_error = final_value - response.back();

    return metrics;
}

PIDTuning ziegler_nichols(float ku, float tu, const std::string& type)
{
    PIDTuning tuning{};

    if (type == "p") {
        tuning.kp = 0.5f * ku;
        tuning.ki = 0.0f;
        tuning.kd = 0.0f;
    } else if (type == "pi") {
        tuning.kp = 0.45f * ku;
        tuning.ki = 0.54f * ku / tu;
        tuning.kd = 0.0f;
    } else {  // PID
        tuning.kp = 0.6f * ku;
        tuning.ki = 1.2f * ku / tu;
        tuning.kd = 0.075f * ku * tu;
    }

    return tuning;
}

PIDTuning cohen_coon(float K, float T, float L, const std::string& type)
{
    PIDTuning tuning{};

    float ratio = L / T;

    if (type == "p") {
        tuning.kp = (1.0f / K) * (1.0f + (0.35f * ratio) / (1.0f - 0.85f * ratio));
        tuning.ki = 0.0f;
        tuning.kd = 0.0f;
    } else if (type == "pi") {
        tuning.kp = (0.9f / K) * (1.0f + (0.35f * ratio) / (1.0f - 0.85f * ratio));
        tuning.ki = tuning.kp / (3.33f * L * (ratio - 1.0f) / (ratio + 1.0f));
        tuning.kd = 0.0f;
    } else {  // PID
        tuning.kp = (1.35f / K) * (1.0f + (0.35f * ratio) / (1.0f - 0.85f * ratio));
        tuning.ki = tuning.kp / (2.5f * L * (ratio - 1.0f) / (ratio + 1.0f));
        tuning.kd = 0.37f * L * (1.0f - ratio) / (1.0f + 0.85f * ratio);
    }

    return tuning;
}

} // namespace ControlUtils

} // namespace mechatron
