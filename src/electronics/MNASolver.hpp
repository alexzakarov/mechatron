#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace mechatron {

// Modified Nodal Analysis (MNA) Solver
// Solves the system: [G B; C D] * [v; j] = [i; e]
// Where:
//   G = conductance matrix (n x n)
//   B, C = voltage source connections
//   v = node voltages (unknown)
//   j = voltage source branch currents (unknown)
//   i = current source injections (known)
//   e = voltage source values (known)
class MNASolver {
public:
    // Reset and prepare for a new system
    void begin_build(int num_nodes, int num_voltage_sources) {
        m_num_nodes = num_nodes;
        m_num_vs = num_voltage_sources;
        // Ground node (index 0) is eliminated from the matrix
        // Matrix size = (num_nodes - 1) + num_voltage_sources
        int size = (num_nodes - 1) + num_voltage_sources;
        m_size = size;
        m_A.assign(size, std::vector<double>(size + 1, 0.0));  // Augmented matrix
        m_x.assign(size, 0.0);
        m_solved = false;
        m_vs_counter = 0;

        // Add minimum conductance to ground for numerical stability
        constexpr double G_MIN = 1e-9;
        for (int i = 0; i < num_nodes - 1; i++) {
            m_A[i][i] += G_MIN;
        }
    }

    // Add a conductance stamp between two nodes
    // G[n1][n1] += G, G[n2][n2] += G, G[n1][n2] -= G, G[n2][n1] -= G
    void add_conductance(int n1, int n2, double G) {
        int i1 = n1 - 1;  // Convert to matrix index (ground = -1, skip)
        int i2 = n2 - 1;

        if (i1 >= 0 && i1 < m_size) {
            m_A[i1][i1] += G;
        }
        if (i2 >= 0 && i2 < m_size) {
            m_A[i2][i2] += G;
        }
        if (i1 >= 0 && i2 >= 0 && i1 < m_size && i2 < m_size) {
            m_A[i1][i2] -= G;
            m_A[i2][i1] -= G;
        }
    }

    // Add a voltage source: V_source = voltage, from n_neg to n_pos
    // Returns the auxiliary variable index for this voltage source
    int add_voltage_source(int n_pos, int n_neg, double voltage) {
        int vs_idx = m_vs_counter++;
        int aux_row = (m_num_nodes - 1) + vs_idx;  // Row in augmented matrix for this VS
        int i_pos = n_pos - 1;
        int i_neg = n_neg - 1;

        // B matrix: links nodes to VS auxiliary current
        if (i_pos >= 0 && i_pos < m_size) {
            m_A[i_pos][aux_row] += 1.0;
            m_A[aux_row][i_pos] += 1.0;
        }
        if (i_neg >= 0 && i_neg < m_size) {
            m_A[i_neg][aux_row] -= 1.0;
            m_A[aux_row][i_neg] -= 1.0;
        }

        // RHS: voltage source value
        m_A[aux_row][m_size] = voltage;

        return vs_idx;
    }

    // Add a current source: current flows from n_neg to n_pos
    void add_current_source(int n_pos, int n_neg, double current) {
        int i_pos = n_pos - 1;
        int i_neg = n_neg - 1;

        if (i_pos >= 0 && i_pos < m_size) {
            m_A[i_pos][m_size] += current;
        }
        if (i_neg >= 0 && i_neg < m_size) {
            m_A[i_neg][m_size] -= current;
        }
    }

    // Solve the system using Gaussian elimination with partial pivoting
    bool solve() {
        if (m_size == 0) {
            m_solved = true;
            return true;
        }

        // Forward elimination with partial pivoting
        for (int col = 0; col < m_size; col++) {
            // Find pivot
            int max_row = col;
            double max_val = std::abs(m_A[col][col]);
            for (int row = col + 1; row < m_size; row++) {
                double val = std::abs(m_A[row][col]);
                if (val > max_val) {
                    max_val = val;
                    max_row = row;
                }
            }

            // Swap rows
            if (max_row != col) {
                std::swap(m_A[col], m_A[max_row]);
            }

            // Check for singular matrix
            if (std::abs(m_A[col][col]) < 1e-15) {
                return false;
            }

            // Eliminate below
            for (int row = col + 1; row < m_size; row++) {
                double factor = m_A[row][col] / m_A[col][col];
                for (int j = col; j <= m_size; j++) {
                    m_A[row][j] -= factor * m_A[col][j];
                }
            }
        }

        // Back substitution
        for (int row = m_size - 1; row >= 0; row--) {
            m_x[row] = m_A[row][m_size];
            for (int col = row + 1; col < m_size; col++) {
                m_x[row] -= m_A[row][col] * m_x[col];
            }
            m_x[row] /= m_A[row][row];
        }

        m_solved = true;
        return true;
    }

    // Get solved voltage for a node (0 = ground = 0.0V)
    double get_node_voltage(int node_index) const {
        if (!m_solved) return 0.0;
        if (node_index == 0) return 0.0;  // Ground
        int idx = node_index - 1;
        if (idx < 0 || idx >= m_size) return 0.0;
        return m_x[idx];
    }

    // Get solved branch current for voltage source at given index
    double get_branch_current(int vs_index) const {
        if (!m_solved) return 0.0;
        int idx = (m_num_nodes - 1) + vs_index;
        if (idx < 0 || idx >= m_size) return 0.0;
        return m_x[idx];
    }

    int size() const { return m_size; }
    bool is_solved() const { return m_solved; }

    // Copy assignment for solver state preservation
    MNASolver& operator=(const MNASolver& other) {
        if (this != &other) {
            m_A = other.m_A;
            m_x = other.m_x;
            m_size = other.m_size;
            m_num_nodes = other.m_num_nodes;
            m_num_vs = other.m_num_vs;
            m_vs_counter = other.m_vs_counter;
            m_solved = other.m_solved;
        }
        return *this;
    }

private:
    std::vector<std::vector<double>> m_A;  // Augmented matrix [A|b]
    std::vector<double> m_x;               // Solution vector
    int m_size = 0;
    int m_num_nodes = 0;
    int m_num_vs = 0;
    int m_vs_counter = 0;
    bool m_solved = false;
};

} // namespace mechatron
