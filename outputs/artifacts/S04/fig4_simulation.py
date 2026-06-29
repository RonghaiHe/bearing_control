"""
Simulation of Fig. 4: Square Formation with Constant Leader Velocity

This script reproduces the simulation example from Section III-A demonstrating
control law (7) for a 2D square formation with 2 leaders and 2 followers.

The formation translates and scales while maintaining the desired pattern.

References:
- Section III-A, Fig. 4, pp. 6-7
- k_p = 0.5, k_v = 2
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple
import sys
import os

# Add parent directories to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_bearing
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian
from S03.control_constant_velocity import ControlLaw7


def setup_square_formation() -> Tuple[FormationGraph, Dict[Tuple[int, int], np.ndarray], np.ndarray, np.ndarray]:
    n_agents = 4
    leader_indices = [1, 2]
    
    edges = [
        (0, 1), (1, 0),
        (0, 2), (2, 0),
        (2, 3), (3, 2),
        (0, 3), (3, 0),
        (1, 2), (2, 1),
    ]
    
    graph = FormationGraph(n_agents, edges, leader_indices)
    
    cube_vertices = np.array([
        [-0.5, 0.5],
        [0.5, 0.5],
        [0.5, -0.5],
        [-0.5, -0.5],
    ])
    
    desired_bearings = {}
    for (i, j) in edges:
        desired_bearings[(i, j)] = compute_bearing(cube_vertices[i], cube_vertices[j])
    
    initial_positions = np.array([
        [0.0, 0.0],
        [10.0, 10.0],
        [10.0, -10.0],
        [19.0, 5.0],
    ])
    
    initial_velocities = np.zeros((n_agents, 2))
    
    return graph, desired_bearings, initial_positions, initial_velocities


def leader_motion(t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    v_c = np.array([0.5, 0.0])

    scale = 1.0 + 0.1 * np.sin(0.5 * t)
    scale_dot = 0.05 * np.cos(0.5 * t)
    scale_ddot = -0.025 * np.sin(0.5 * t)

    c = v_c * t

    p0_local = np.array([-0.5, 0.5])
    p1_local = np.array([0.5, 0.5])

    leader_positions = np.array([
        c + scale * p0_local,
        c + scale * p1_local,
    ])

    leader_velocities = np.array([
        v_c + scale_dot * p0_local,
        v_c + scale_dot * p1_local,
    ])

    leader_accelerations = np.array([
        scale_ddot * p0_local,
        scale_ddot * p1_local,
    ])

    return leader_positions, leader_velocities, leader_accelerations


def run_simulation():
    graph, desired_bearings, initial_positions, initial_velocities = setup_square_formation()

    k_p = 0.5
    k_v = 2.0
    dt = 0.01
    T = 30.0
    n_steps = int(T / dt) + 1

    B = compute_bearing_laplacian(graph, desired_bearings)
    B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, graph)
    B_ff_inv = np.linalg.inv(B_ff)

    n_followers = len(graph.follower_indices)

    time_array = np.linspace(0, T, n_steps)
    positions_history = np.zeros((n_steps, 4, 2))
    velocities_history = np.zeros((n_steps, 4, 2))
    bearing_errors = np.zeros((n_steps,))

    positions = initial_positions.copy()
    velocities = initial_velocities.copy()

    for step in range(n_steps):
        t = time_array[step]

        leader_pos, leader_vel, leader_accel = leader_motion(t)

        for idx, i in enumerate(graph.leader_indices):
            positions[i] = leader_pos[idx]
            velocities[i] = leader_vel[idx]

        positions_history[step] = positions.copy()
        velocities_history[step] = velocities.copy()

        total_error = 0.0
        for (i, j), g_desired in desired_bearings.items():
            g_actual = compute_bearing(positions[i], positions[j])
            total_error += np.linalg.norm(g_actual - g_desired)
        bearing_errors[step] = total_error

        p_f = positions[graph.follower_indices].flatten()
        v_f = velocities[graph.follower_indices].flatten()
        p_l = positions[graph.leader_indices].flatten()
        v_l = velocities[graph.leader_indices].flatten()

        delta_p = p_f + B_ff_inv @ B_fl @ p_l
        delta_v = v_f + B_ff_inv @ B_fl @ v_l

        v_l_dot = leader_accel.flatten()
        u_f = -k_p * delta_p - k_v * delta_v

        controls = u_f.reshape(n_followers, 2)

        for idx, i in enumerate(graph.follower_indices):
            positions[i] += velocities[i] * dt
            velocities[i] += controls[idx] * dt

    return time_array, positions_history, velocities_history, bearing_errors


def plot_results(time_array, positions_history, velocities_history, bearing_errors):
    """Generate figures matching Fig. 4 in the paper."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Fig. 4(a): Trajectory
    ax1 = axes[0, 0]
    colors = ['blue', 'red', 'green', 'orange']
    labels = ['Leader 0', 'Leader 1', 'Follower 2', 'Follower 3']
    
    for i in range(4):
        ax1.plot(positions_history[:, i, 0], positions_history[:, i, 1], 
                color=colors[i], label=labels[i], linewidth=2)
    
    # Mark start and end points
    for i in range(4):
        ax1.scatter(positions_history[0, i, 0], positions_history[0, i, 1], 
                   color=colors[i], marker='o', s=100, zorder=5)
        ax1.scatter(positions_history[-1, i, 0], positions_history[-1, i, 1], 
                   color=colors[i], marker='x', s=100, zorder=5)
    
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_title('(a) Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Fig. 4(b): Total bearing error
    ax2 = axes[0, 1]
    ax2.plot(time_array, bearing_errors, 'b-', linewidth=2)
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Total Bearing Error')
    ax2.set_title(r'(b) Total Bearing Error $\sum_{(i,j) \in \mathcal{E}} \|g_{ij}(t) - g_{ij}^*\|$')
    ax2.grid(True, alpha=0.3)
    
    # Fig. 4(c): Velocity (x-component)
    ax3 = axes[1, 0]
    for i in range(4):
        ax3.plot(time_array, velocities_history[:, i, 0], 
                color=colors[i], label=labels[i], linewidth=2)
    
    ax3.set_xlabel('Time')
    ax3.set_ylabel('x-velocity')
    ax3.set_title('(c) Velocity')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Additional: Scale over time
    ax4 = axes[1, 1]
    scales = []
    for step in range(len(time_array)):
        positions = positions_history[step]
        centroid = np.mean(positions, axis=0)
        diffs = positions - centroid
        scale = np.sqrt(np.mean(np.sum(diffs**2, axis=1)))
        scales.append(scale)
    
    ax4.plot(time_array, scales, 'g-', linewidth=2)
    ax4.set_xlabel('Time')
    ax4.set_ylabel('Formation Scale')
    ax4.set_title('Formation Scale Over Time')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/ronghai/bearing_control/outputs/artifacts/S04/fig4_simulation.png', 
                dpi=150, bbox_inches='tight')
    plt.close()
    
    print("Figure 4 saved to outputs/artifacts/S04/fig4_simulation.png")


if __name__ == "__main__":
    print("Running Fig. 4 simulation...")
    time_array, positions_history, velocities_history, bearing_errors = run_simulation()
    plot_results(time_array, positions_history, velocities_history, bearing_errors)
    print("Simulation complete!")
