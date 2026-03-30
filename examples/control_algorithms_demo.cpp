// Control Algorithms Demo - PID, State-Space, MPC, and Trajectory Generation
// Demonstrates advanced control algorithms for mechatronics systems

#include "control/ControlAlgorithms.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace mechatron;

// Simple first-order plant for simulation: G(s) = K/(tau*s + 1)
class FirstOrderPlant {
public:
    FirstOrderPlant(float K = 1.0f, float tau = 0.5f)
        : m_K(K), m_tau(tau), m_output(0.0f)
    {
    }

    void update(float input, float dt)
    {
        // Euler discretization: y[n+1] = y[n] + (dt/tau) * (K*u - y[n])
        m_output += (dt / m_tau) * (m_K * input - m_output);
    }

    float get_output() const { return m_output; }
    void reset() { m_output = 0.0f; }

private:
    float m_K, m_tau, m_output;
};

// Second-order plant for simulation: G(s) = omega_n^2/(s^2 + 2*zeta*omega_n*s + omega_n^2)
class SecondOrderPlant {
public:
    SecondOrderPlant(float omega_n = 10.0f, float zeta = 0.5f)
        : m_omega_n(omega_n), m_zeta(zeta), m_pos(0.0f), m_vel(0.0f)
    {
    }

    void update(float force, float dt)
    {
        // Mass-spring-damper: m*a + c*v + k*x = F
        // Normalized: a + 2*zeta*omega_n*v + omega_n^2*x = omega_n^2*u
        float accel = std::pow(m_omega_n, 2) * force - 2 * m_zeta * m_omega_n * m_vel - std::pow(m_omega_n, 2) * m_pos;

        m_vel += accel * dt;
        m_pos += m_vel * dt;
    }

    float get_position() const { return m_pos; }
    float get_velocity() const { return m_vel; }
    void reset() { m_pos = m_vel = 0.0f; }

private:
    float m_omega_n, m_zeta, m_pos, m_vel;
};

void print_separator()
{
    std::cout << std::string(70, '-') << std::endl;
}

void test_pid_controller()
{
    std::cout << "\n=== Test 1: PID Controller ===" << std::endl;
    print_separator();

    // Create plant and controller
    FirstOrderPlant plant(1.0f, 0.5f);  // K=1, tau=0.5s
    PIDController::Params params;
    params.kp = 2.0f;
    params.ki = 1.0f;
    params.kd = 0.1f;
    params.dt = 0.01f;
    params.output_min = -10.0f;
    params.output_max = 10.0f;

    PIDController pid(params);

    // Simulation parameters
    float setpoint = 1.0f;
    float dt = 0.01f;
    float duration = 3.0f;
    int steps = static_cast<int>(duration / dt);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time\tSetpoint\tOutput\tError\t\tControl" << std::endl;

    // Run simulation
    std::vector<float> response;
    for (int i = 0; i <= steps; ++i) {
        float t = i * dt;
        float measurement = plant.get_output();
        float control = pid.compute(setpoint, measurement);
        plant.update(control, dt);
        response.push_back(measurement);

        if (i % 50 == 0) {
            std::cout << t << "\t" << setpoint << "\t\t" << measurement
                      << "\t" << (setpoint - measurement) << "\t" << control << std::endl;
        }

        // Change setpoint midway
        if (i == steps / 2) {
            setpoint = 0.5f;
        }
    }

    // Analyze response
    auto metrics = ControlUtils::analyze_step_response(response, dt, setpoint);
    std::cout << "\nStep Response Metrics:" << std::endl;
    std::cout << "  Rise time: " << metrics.rise_time << " s" << std::endl;
    std::cout << "  Settling time: " << metrics.settling_time << " s" << std::endl;
    std::cout << "  Overshoot: " << metrics.overshoot << "%" << std::endl;
    std::cout << "  Steady state error: " << metrics.steady_state_error << std::endl;
}

void test_pi_controller()
{
    std::cout << "\n=== Test 2: PI Controller ===" << std::endl;
    print_separator();

    FirstOrderPlant plant(1.0f, 0.8f);
    PIController::Params params;
    params.kp = 1.5f;
    params.ki = 0.5f;
    params.dt = 0.01f;

    PIController pi(params);

    float setpoint = 1.0f;
    float dt = 0.01f;

    std::cout << "PI Control Response:" << std::endl;
    std::cout << "Time\tOutput\t\tIntegral" << std::endl;

    for (int i = 0; i <= 200; ++i) {
        float t = i * dt;
        float measurement = plant.get_output();
        float control = pi.compute(setpoint, measurement);
        plant.update(control, dt);

        if (i % 40 == 0) {
            std::cout << t << "\t" << measurement << "\t\t" << pi.get_params().ki << std::endl;
        }
    }

    std::cout << "Final output: " << plant.get_output() << " (target: " << setpoint << ")" << std::endl;
}

void test_cascade_controller()
{
    std::cout << "\n=== Test 3: Cascade Controller (Position/Velocity) ===" << std::endl;
    print_separator();

    CascadeController cascade;

    // Position loop (outer)
    CascadeController::LoopConfig pos_loop;
    pos_loop.pid_params.kp = 5.0f;
    pos_loop.pid_params.ki = 0.0f;
    pos_loop.pid_params.kd = 0.5f;
    pos_loop.pid_params.dt = 0.01f;
    cascade.add_loop(pos_loop);

    // Velocity loop (inner)
    CascadeController::LoopConfig vel_loop;
    vel_loop.pid_params.kp = 2.0f;
    vel_loop.pid_params.ki = 1.0f;
    vel_loop.pid_params.kd = 0.0f;
    vel_loop.pid_params.dt = 0.01f;
    cascade.add_loop(vel_loop);

    std::cout << "Cascade Control Structure:" << std::endl;
    std::cout << "  Outer Loop: Position (Kp=5.0, Kd=0.5)" << std::endl;
    std::cout << "  Inner Loop: Velocity (Kp=2.0, Ki=1.0)" << std::endl;
    std::cout << "\nTime\tPosition\tVelocity\tControl" << std::endl;

    float position = 0.0f;
    float velocity = 0.0f;
    float setpoint = 1.0f;
    float dt = 0.01f;

    for (int i = 0; i <= 150; ++i) {
        float t = i * dt;

        std::vector<float> measurements = {position, velocity};
        float control = cascade.compute(setpoint, measurements);

        // Simple plant model
        velocity += control * dt * 0.5f;
        position += velocity * dt;

        if (i % 25 == 0) {
            std::cout << t << "\t" << position << "\t\t" << velocity << "\t\t" << control << std::endl;
        }
    }

    std::cout << "Final position: " << position << " (target: " << setpoint << ")" << std::endl;
}

void test_trajectory_generator()
{
    std::cout << "\n=== Test 4: Trajectory Generation ===" << std::endl;
    print_separator();

    TrajectoryGenerator::Params params;
    params.type = TrajectoryGenerator::Type::Polynomial;
    params.max_velocity = 2.0f;
    params.max_acceleration = 5.0f;
    params.polynomial_order = 5;
    params.dt = 0.01f;

    TrajectoryGenerator traj_gen(params);

    float start_pos = 0.0f;
    float target_pos = 1.0f;

    traj_gen.plan(start_pos, target_pos);

    std::cout << "5th-Order Polynomial Trajectory:" << std::endl;
    std::cout << "From " << start_pos << " to " << target_pos << std::endl;
    std::cout << "Duration: " << traj_gen.get_duration() << " s" << std::endl;
    std::cout << "\nTime\tPosition\tVelocity\tAcceleration" << std::endl;

    for (int i = 0; i <= static_cast<int>(traj_gen.get_duration() / 0.1); ++i) {
        float t = i * 0.1f;
        float pos = traj_gen.get_position(t);
        float vel = traj_gen.get_velocity(t);
        float acc = traj_gen.get_acceleration(t);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << t << "\t" << pos << "\t\t" << vel << "\t\t" << acc << std::endl;
    }

    // Test S-curve
    std::cout << "\nS-Curve Trajectory:" << std::endl;
    params.type = TrajectoryGenerator::Type::SCurve;
    params.max_velocity = 1.0f;
    params.max_acceleration = 2.0f;
    params.max_jerk = 10.0f;

    TrajectoryGenerator scurve(params);
    scurve.plan(0.0f, 1.0f);

    std::cout << "Duration: " << scurve.get_duration() << " s" << std::endl;
    std::cout << "Time\tPosition\tVelocity\tAcceleration" << std::endl;

    for (int i = 0; i <= static_cast<int>(scurve.get_duration() / 0.1); ++i) {
        float t = i * 0.1f;
        std::cout << t << "\t" << scurve.get_position(t)
                  << "\t\t" << scurve.get_velocity(t)
                  << "\t\t" << scurve.get_acceleration(t) << std::endl;
    }
}

void test_feedforward_controller()
{
    std::cout << "\n=== Test 5: Feedforward Control ===" << std::endl;
    print_separator();

    // Plant: mass-spring system
    float mass = 2.0f;
    float damping = 0.5f;
    float stiffness = 10.0f;

    FeedforwardController::Params ff_params;
    ff_params.mass = mass;
    ff_params.damping = damping;
    ff_params.stiffness = stiffness;

    FeedforwardController ff(ff_params);

    TrajectoryGenerator traj_gen;
    TrajectoryGenerator::Params traj_params;
    traj_params.type = TrajectoryGenerator::Type::Polynomial;
    traj_params.max_velocity = 1.0f;
    traj_params.max_acceleration = 5.0f;
    traj_gen.set_params(traj_params);

    traj_gen.plan(0.0f, 1.0f);

    std::cout << "Feedforward Control for Mass-Spring System:" << std::endl;
    std::cout << "Mass: " << mass << " kg, Damping: " << damping << " Ns/m, Stiffness: " << stiffness << " N/m" << std::endl;
    std::cout << "\nTime\tDes.Pos\tDes.Vel\tDes.Acc\tFF Force" << std::endl;

    float dt = 0.01f;
    for (int i = 0; i <= static_cast<int>(traj_gen.get_duration() / dt); i += 10) {
        float t = i * dt;
        float des_pos = traj_gen.get_position(t);
        float des_vel = traj_gen.get_velocity(t);
        float des_acc = traj_gen.get_acceleration(t);
        float ff_force = ff.compute(des_acc, des_vel, des_pos);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << t << "\t" << des_pos << "\t" << des_vel << "\t" << des_acc << "\t" << ff_force << std::endl;
    }
}

void test_pid_tuning_methods()
{
    std::cout << "\n=== Test 6: PID Tuning Methods ===" << std::endl;
    print_separator();

    // Ziegler-Nichols tuning
    float Ku = 4.0f;  // Ultimate gain
    float Tu = 1.0f;  // Ultimate period (seconds)

    std::cout << "Ziegler-Nichols Tuning (Ku=" << Ku << ", Tu=" << Tu << "s):" << std::endl;
    std::cout << std::left << std::setw(15) << "Controller"
              << std::setw(10) << "Kp" << "Ki" << "Kd" << std::endl;
    std::cout << std::string(50, '-') << std::endl;

    auto p_tuning = ControlUtils::ziegler_nichols(Ku, Tu, "p");
    std::cout << std::left << std::setw(15) << "P"
              << std::setw(10) << p_tuning.kp << p_tuning.ki << p_tuning.kd << std::endl;

    auto pi_tuning = ControlUtils::ziegler_nichols(Ku, Tu, "pi");
    std::cout << std::left << std::setw(15) << "PI"
              << std::setw(10) << pi_tuning.kp << pi_tuning.ki << pi_tuning.kd << std::endl;

    auto pid_tuning = ControlUtils::ziegler_nichols(Ku, Tu, "pid");
    std::cout << std::left << std::setw(15) << "PID"
              << std::setw(10) << pid_tuning.kp << pid_tuning.ki << pid_tuning.kd << std::endl;

    // Cohen-Coon tuning
    std::cout << "\nCohen-Coon Tuning (K=1, T=1, L=0.5):" << std::endl;
    std::cout << std::string(50, '-') << std::endl;

    float K = 1.0f, T = 1.0f, L = 0.5f;

    auto cc_p = ControlUtils::cohen_coon(K, T, L, "p");
    std::cout << std::left << std::setw(15) << "P"
              << std::setw(10) << cc_p.kp << cc_p.ki << cc_p.kd << std::endl;

    auto cc_pi = ControlUtils::cohen_coon(K, T, L, "pi");
    std::cout << std::left << std::setw(15) << "PI"
              << std::setw(10) << cc_pi.kp << cc_pi.ki << cc_pi.kd << std::endl;

    auto cc_pid = ControlUtils::cohen_coon(K, T, L, "pid");
    std::cout << std::left << std::setw(15) << "PID"
              << std::setw(10) << cc_pid.kp << cc_pid.ki << cc_pid.kd << std::endl;
}

void test_lead_lag_compensator()
{
    std::cout << "\n=== Test 7: Lead-Lag Compensator ===" << std::endl;
    print_separator();

    LeadLagCompensator::Params params;
    params.gain = 2.0f;
    params.zero = 1.0f;    // Zero at 1 rad/s
    params.pole = 10.0f;   // Pole at 10 rad/s (lead compensator)
    params.dt = 0.01f;

    LeadLagCompensator compensator(params);

    FirstOrderPlant plant(1.0f, 0.3f);

    std::cout << "Lead Compensator (zero=1 rad/s, pole=10 rad/s):" << std::endl;
    std::cout << "Improves transient response and phase margin" << std::endl;
    std::cout << "\nTime\tSetpoint\tOutput\t\tControl" << std::endl;

    float setpoint = 1.0f;
    float dt = 0.01f;

    for (int i = 0; i <= 200; ++i) {
        float t = i * dt;
        float measurement = plant.get_output();
        float control = compensator.compute(setpoint, measurement);
        plant.update(control, dt);

        if (i % 40 == 0) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << t << "\t" << setpoint << "\t" << measurement << "\t\t" << control << std::endl;
        }
    }

    std::cout << "Final output: " << plant.get_output() << " (target: " << setpoint << ")" << std::endl;
}

void test_control_algorithm_comparison()
{
    std::cout << "\n=== Test 8: Control Algorithm Comparison ===" << std::endl;
    print_separator();

    std::cout << "\nControl Algorithms Available in MECHATRON:" << std::endl;
    std::cout << std::left << std::setw(30) << "Algorithm"
              << "Use Case" << std::endl;
    std::cout << std::string(70, '-') << std::endl;

    std::cout << std::left << std::setw(30) << "PID Controller"
              << "General purpose, most common" << std::endl;
    std::cout << std::left << std::setw(30) << "PI Controller"
              << "Industrial applications, better stability" << std::endl;
    std::cout << std::left << std::setw(30) << "Lead-Lag Compensator"
              << "Frequency domain shaping" << std::endl;
    std::cout << std::left << std::setw(30) << "State-Space Controller"
              << "MIMO systems, optimal control" << std::endl;
    std::cout << std::left << std::setw(30) << "Feedforward Controller"
              << "Motion control, model-based" << std::endl;
    std::cout << std::left << std::setw(30) << "Cascade Controller"
              << "Nested loops (pos/vel/current)" << std::endl;
    std::cout << std::left << std::setw(30) << "Deadbeat Controller"
              << "Fast discrete-time response" << std::endl;
    std::cout << std::left << std::setw(30) << "Kalman Filter"
              << "State estimation, sensor fusion" << std::endl;
    std::cout << std::left << std::setw(30) << "MPC Controller"
              << "Constrained optimization, predictive" << std::endl;
    std::cout << std::left << std::setw(30) << "Trajectory Generator"
              << "Smooth reference profiles" << std::endl;

    std::cout << "\nTuning Methods:" << std::endl;
    std::cout << "  - Ziegler-Nichols: Ultimate gain/period method" << std::endl;
    std::cout << "  - Cohen-Coon: First-order + dead-time processes" << std::endl;
    std::cout << "  - Pole Placement: Assign closed-loop poles" << std::endl;
    std::cout << "  - LQR: Optimal state feedback (requires Q, R matrices)" << std::endl;
}

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "=== Control Algorithms Demo ===" << std::endl;
    std::cout << "=== Advanced Mechatronics Control ===" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_pid_controller();
        test_pi_controller();
        test_cascade_controller();
        test_trajectory_generator();
        test_feedforward_controller();
        test_pid_tuning_methods();
        test_lead_lag_compensator();
        test_control_algorithm_comparison();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=== CONTROL ALGORITHMS DEMO COMPLETE ===" << std::endl;
        std::cout << "========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n✗ DEMO FAILED: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
