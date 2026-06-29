"""
Simulation of Fig. 5: 3D Cube Formation with Time-Varying Leader Velocity

This script reproduces the simulation example from Section III-B demonstrating
control law (10) for a 3D cube formation with 2 leaders and 6 followers.

The formation converges from an initial configuration to the desired pattern
while translating through an obstacle, contracting and expanding as needed.

References:
- Section III-B, Fig. 5, pp. 8-9
- 3D cube formation (Fig. 2(c))
"""

import matplotlib
matplotlib.use('Agg')

import numpy as np
import warnings
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import Dict, Tuple
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_bearing
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian
from S05.control_time_varying import ControlLaw10


def setup_cube_formation() -> Tuple[FormationGraph, Dict[Tuple[int, int], np.ndarray], np.ndarray, np.ndarray]:
    n_agents = 8

    edges = []
    for i in range(8):
        for j in range(8):
            if i < j:
                diff = i ^ j
                if diff in [1, 2, 4, 3, 5, 6]:
                    edges.append((i, j))
                    edges.append((j, i))

    leader_indices = [0, 1]
    graph = FormationGraph(n_agents, edges, leader_indices)

    cube_vertices = np.array([
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ])

    desired_bearings = {}
    for (i, j) in edges:
        desired_bearings[(i, j)] = compute_bearing(cube_vertices[i], cube_vertices[j])

    np.random.seed(42)
    initial_positions = cube_vertices + np.array([
        [-10.0, 9.0, -1.0],
        [-1.0, 9.0, -1.0],
        [-13.0, 9.0, -8.0],
        [1.0, 9.0, -9.0],
        [-10.0, 0.0, -3.0],
        [-4.0, 0.0, -3.0],
        [-10.0, 1.5, -8.5],
        [-1.0, 0.0, -9.0],
    ])
    print("Initial positions:\n", initial_positions)
    initial_velocities = np.zeros((n_agents, 3))

    return graph, desired_bearings, initial_positions, initial_velocities


def scale_function(t, v_c_y=1.0, obstacle_y=45.0, contraction_zone=5.0, min_scale=0.3):
    """Scale that contracts near obstacle - sharp Gaussian transition."""
    y_center = v_c_y * t
    dist = y_center - obstacle_y
    sigma = contraction_zone / 2.0
    return min_scale + (1.0 - min_scale) * np.exp(-dist**2 / (2.0 * sigma**2))


def scale_dot_function(t, v_c_y=1.0, obstacle_y=45.0, contraction_zone=5.0, min_scale=0.3):
    """Derivative of Gaussian scale function."""
    y_center = v_c_y * t
    dist = y_center - obstacle_y
    sigma = contraction_zone / 2.0
    gaussian = np.exp(-dist**2 / (2.0 * sigma**2))
    return (1.0 - min_scale) * gaussian * (-dist / sigma**2) * v_c_y


def leader_motion_3d(t):
    """Compute leader positions, velocities, and accelerations.

    Leaders travel along Y-axis. Formation contracts near obstacle at y=45.
    """
    v_c = np.array([0.0, 1.0, 0.0])
    v_c_y = 2.0

    s = scale_function(t, v_c_y, contraction_zone=5.0)
    s_dot = scale_dot_function(t, v_c_y, contraction_zone=5.0)

    c = v_c * t

    p0_local = np.array([-10.0, 10.0, 0.0])
    p1_local = np.array([0.0, 10.0, 0.0])

    leader_positions = np.array([
        c + s * p0_local,
        c + s * p1_local,
    ])

    leader_velocities = np.array([
        v_c + s_dot * p0_local,
        v_c + s_dot * p1_local,
    ])

    leader_accelerations = np.zeros((2, 3))

    return leader_positions, leader_velocities, leader_accelerations


def run_simulation_3d():
    """Run the 3D simulation and generate figures."""
    graph, desired_bearings, initial_positions, initial_velocities = setup_cube_formation()

    k_p = 0.5
    k_v = 2.0
    dt = 0.02
    T = 80.0
    n_steps = int(T / dt) + 1

    B = compute_bearing_laplacian(graph, desired_bearings)
    B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, graph)

    B_ff_inv = np.linalg.inv(B_ff)
    B_ff_inv_B_fl = B_ff_inv @ B_fl

    n_followers = len(graph.follower_indices)
    n_f = n_followers * 3

    time_array = np.linspace(0, T, n_steps)
    positions_history = np.zeros((n_steps, 8, 3))
    velocities_history = np.zeros((n_steps, 8, 3))
    bearing_errors = np.zeros((n_steps,))

    positions = initial_positions.copy()
    velocities = initial_velocities.copy()

    for step in range(n_steps):
        t = time_array[step]

        leader_pos, leader_vel, leader_accel = leader_motion_3d(t)

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
        u_f = -k_p * delta_p - k_v * delta_v + B_ff_inv_B_fl @ v_l_dot

        controls = u_f.reshape(n_followers, 3)

        for idx, i in enumerate(graph.follower_indices):
            positions[i] += velocities[i] * dt
            velocities[i] += controls[idx] * dt

    return time_array, positions_history, velocities_history, bearing_errors


def draw_obstacle(ax, obstacle_y=45.0, wall_size=3.0, opening_size=1.5):
    """Draw a gray wall with rectangular opening (doorway)."""
    panels = []

    # Left panel: x from -wall_size to -opening_size
    panels.append([[-wall_size, obstacle_y, -wall_size],
                   [-opening_size, obstacle_y, -wall_size],
                   [-opening_size, obstacle_y, wall_size],
                   [-wall_size, obstacle_y, wall_size]])

    # Right panel: x from opening_size to wall_size
    panels.append([[opening_size, obstacle_y, -wall_size],
                   [wall_size, obstacle_y, -wall_size],
                   [wall_size, obstacle_y, wall_size],
                   [opening_size, obstacle_y, wall_size]])

    # Top panel: z from opening_size to wall_size
    panels.append([[-wall_size, obstacle_y, opening_size],
                   [wall_size, obstacle_y, opening_size],
                   [wall_size, obstacle_y, wall_size],
                   [-wall_size, obstacle_y, wall_size]])

    # Bottom panel: z from -wall_size to -opening_size
    panels.append([[-wall_size, obstacle_y, -wall_size],
                   [wall_size, obstacle_y, -wall_size],
                   [wall_size, obstacle_y, -opening_size],
                   [-wall_size, obstacle_y, -opening_size]])

    poly = Poly3DCollection(panels, alpha=0.45, facecolor='gray', edgecolor='dimgray', linewidth=0.8)
    ax.add_collection3d(poly)


def plot_results_3d(time_array, positions_history, velocities_history, bearing_errors):
    """Generate a single 3D visualization matching figure_68."""
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')

    leader_idx = [0, 1]
    follower_idx = list(range(2, 8))

    # All cube + face diagonal edges (bidirectional, deduplicated)
    snapshot_edges = [
        (0, 1), (0, 2), (0, 4), (0, 7),
        (1, 3), (1, 5),
        (2, 3), (2, 6),
        (3, 7),
        (4, 5), (4, 6),
        (5, 7),
        (6, 7)
    ]

    # Draw dashed trajectories
    for i in range(8):
        if i in leader_idx:
            color = 'red'
        else:
            color = 'blue'
        ax.plot(positions_history[:, i, 0], positions_history[:, i, 1], positions_history[:, i, 2],
                color=color, linestyle='--', linewidth=0.8, alpha=0.6)

    # Draw ~6 snapshots
    n_snapshots = 6
    snapshot_indices = np.linspace(0, len(time_array) - 1, n_snapshots, dtype=int)

    for si in snapshot_indices:
        pos = positions_history[si]
        # Draw edges
        for (i, j) in snapshot_edges:
            ax.plot([pos[i, 0], pos[j, 0]], [pos[i, 1], pos[j, 1]], [pos[i, 2], pos[j, 2]],
                    'k-', linewidth=0.6, alpha=0.5)
        # Draw leaders (red triangles)
        for i in leader_idx:
            ax.scatter(pos[i, 0], pos[i, 1], pos[i, 2], c='red', marker='>', s=80, zorder=5,
                       edgecolors='darkred', linewidths=0.5)
        # Draw followers (blue circles)
        for i in follower_idx:
            ax.scatter(pos[i, 0], pos[i, 1], pos[i, 2], c='blue', marker='o', s=50, zorder=5,
                       edgecolors='darkblue', linewidths=0.5)

    # Draw obstacle
    draw_obstacle(ax, obstacle_y=45.0, wall_size=3.0, opening_size=1.5)

    ax.set_xlabel('x (m)', fontsize=11)
    ax.set_ylabel('y (m)', fontsize=11)
    ax.set_zlabel('z (m)', fontsize=11)
    ax.set_xlim(-13, 13)
    ax.set_ylim(-5, 80)
    ax.set_zlim(-13, 13)
    ax.set_aspect('equal')
    ax.view_init(elev=15, azim=-20)

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='>', color='red', markerfacecolor='red', markersize=10,
               label='Leader', linestyle='None', markeredgecolor='darkred'),
        Line2D([0], [0], marker='o', color='blue', markerfacecolor='blue', markersize=8,
               label='Follower', linestyle='None', markeredgecolor='darkblue'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=11, framealpha=0.9)

    plt.tight_layout()
    plt.savefig('/home/ronghai/bearing_control/outputs/artifacts/S06/fig5_simulation.png',
                dpi=150, bbox_inches='tight')
    plt.close()


def plot_curves(time_array, positions_history, velocities_history, bearing_errors):
    """Generate velocity and bearing error plots matching fig4 style."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    leader_idx = [0, 1]
    follower_idx = list(range(2, 8))
    
    # (a) Bearing error
    ax1 = axes[0]
    ax1.plot(time_array, bearing_errors, 'b-', linewidth=2)
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Total Bearing Error')
    ax1.set_title(r'(a) Total Bearing Error $\sum_{(i,j) \in \mathcal{E}} \|g_{ij}(t) - g_{ij}^*\|$')
    ax1.grid(True, alpha=0.3)
    
    # (b) x-velocity
    ax2 = axes[1]
    for i in leader_idx:
        ax2.plot(time_array, velocities_history[:, i, 0],
                 color='red', linestyle='-', linewidth=2)
    for i in follower_idx:
        ax2.plot(time_array, velocities_history[:, i, 0],
                 color='blue', linestyle='--', linewidth=2)
    ax2.plot([], [], color='red', linestyle='-', linewidth=2, label='Leader')
    ax2.plot([], [], color='blue', linestyle='--', linewidth=2, label='Follower')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('x-velocity (m/s)')
    ax2.set_title('(b) x-velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # (c) y-velocity
    ax3 = axes[2]
    for i in leader_idx:
        ax3.plot(time_array, velocities_history[:, i, 1],
                 color='red', linestyle='-', linewidth=2)
    for i in follower_idx:
        ax3.plot(time_array, velocities_history[:, i, 1],
                 color='blue', linestyle='--', linewidth=2)
    ax3.plot([], [], color='red', linestyle='-', linewidth=2, label='Leader')
    ax3.plot([], [], color='blue', linestyle='--', linewidth=2, label='Follower')
    ax3.set_xlabel('Time (sec)')
    ax3.set_ylabel('y-velocity (m/s)')
    ax3.set_title('(c) y-velocity')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/ronghai/bearing_control/outputs/artifacts/S06/fig5_simulation_curves.png',
                dpi=150, bbox_inches='tight')
    plt.close()
    print("Figure 5 curves saved to outputs/artifacts/S06/fig5_simulation_curves.png")


if __name__ == "__main__":
    print("Running Fig. 5 simulation (3D cube formation through obstacle)...")
    time_array, positions_history, velocities_history, bearing_errors = run_simulation_3d()
    plot_results_3d(time_array, positions_history, velocities_history, bearing_errors)
    plot_curves(time_array, positions_history, velocities_history, bearing_errors)
    print("Simulation complete!")
