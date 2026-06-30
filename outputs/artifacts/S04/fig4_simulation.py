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
from S02.bearing_laplacian import compute_bearing_laplacian
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
    T = 180.0
    pi = np.pi

    s = 20.0 + 20.0 * (t / T) - 20.0 * np.sin(pi * t / T)
    s_dot = 20.0 / T - 20.0 * pi / T * np.cos(pi * t / T)
    s_ddot = 20.0 * pi**2 / T**2 * np.sin(pi * t / T)

    c_x = 0.5 * t
    c_x_dot = 0.5

    c_y = 20.0 * (t / T) - 15.0 * np.sin(pi * t / T)
    c_y_dot = 20.0 / T - 15.0 * pi / T * np.cos(pi * t / T)
    c_y_ddot = 15.0 * pi**2 / T**2 * np.sin(pi * t / T)

    c = np.array([c_x, c_y])
    c_dot = np.array([c_x_dot, c_y_dot])
    c_ddot = np.array([0.0, c_y_ddot])

    p1_local = np.array([0.5, 0.5])
    p2_local = np.array([0.5, -0.5])

    leader_positions = np.array([
        c + s * p1_local,
        c + s * p2_local,
    ])

    leader_velocities = np.array([
        c_dot + s_dot * p1_local,
        c_dot + s_dot * p2_local,
    ])

    leader_accelerations = np.array([
        c_ddot + s_ddot * p1_local,
        c_ddot + s_ddot * p2_local,
    ])

    return leader_positions, leader_velocities, leader_accelerations


def run_simulation():
    graph, desired_bearings, initial_positions, initial_velocities = setup_square_formation()

    k_p = 0.5
    k_v = 2.0
    dt = 0.01
    T = 180.0
    n_steps = int(T / dt) + 1

    B = compute_bearing_laplacian(graph, desired_bearings)
    d = 2
    leader_dofs = []
    for i in graph.leader_indices:
        leader_dofs.extend([d*i, d*i+1])
    follower_dofs = []
    for i in graph.follower_indices:
        follower_dofs.extend([d*i, d*i+1])
    B_ll = B[np.ix_(leader_dofs, leader_dofs)]
    B_lf = B[np.ix_(leader_dofs, follower_dofs)]
    B_fl = B[np.ix_(follower_dofs, leader_dofs)]
    B_ff = B[np.ix_(follower_dofs, follower_dofs)]
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
        u_f = -k_p * (B_ff @ delta_p) - k_v * (B_ff @ delta_v) - B_ff_inv @ (B_fl @ v_l_dot)

        controls = u_f.reshape(n_followers, 2)

        for idx, i in enumerate(graph.follower_indices):
            positions[i] += velocities[i] * dt
            velocities[i] += controls[idx] * dt

    return time_array, positions_history, velocities_history, bearing_errors


def plot_results(time_array, positions_history, velocities_history, bearing_errors):
    """Generate figures matching Fig. 4 in the paper."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    leader_idx = [1, 2]
    follower_idx = [0, 3]
    snapshot_edges = [(0, 1), (0, 2), (0, 3), (1, 2), (2, 3)]
    n_snapshots = 5

    # Fig. 4(a): Trajectory with snapshot formations
    ax1 = axes[0, 0]

    for i in range(4):
        style = '--'
        color = 'red' if i in leader_idx else 'blue'
        ax1.plot(positions_history[:, i, 0], positions_history[:, i, 1],
                 color=color, linestyle=style, linewidth=1.5)

    snapshot_indices = np.linspace(0, len(time_array) - 1, n_snapshots, dtype=int)
    for si in snapshot_indices:
        pos = positions_history[si]
        for (i, j) in snapshot_edges:
            ax1.plot([pos[i, 0], pos[j, 0]], [pos[i, 1], pos[j, 1]],
                     'k-', linewidth=1)
        for i in leader_idx:
            ax1.plot(pos[i, 0], pos[i, 1], marker='>', color='red',
                     markersize=8, markerfacecolor='red', zorder=5)
        for i in follower_idx:
            ax1.plot(pos[i, 0], pos[i, 1], marker='o', color='blue',
                     markersize=8, markerfacecolor='blue', zorder=5)

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='>', color='red', markerfacecolor='red',
               markersize=8, label='Leader', linestyle='None'),
        Line2D([0], [0], marker='o', color='blue', markerfacecolor='blue',
               markersize=8, label='Follower', linestyle='None'),
    ]
    ax1.legend(handles=legend_elements, loc='upper left')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_title('(a) Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Fig. 4(b): Total bearing error
    ax2 = axes[0, 1]
    ax2.plot(time_array, bearing_errors, 'b-', linewidth=2)
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Total Bearing Error')
    ax2.set_title(r'(b) Total Bearing Error $\sum_{(i,j) \in \mathcal{E}} \|g_{ij}(t) - g_{ij}^*\|$')
    ax2.grid(True, alpha=0.3)

    # Fig. 4(c): x-velocity
    ax3 = axes[1, 0]
    for i in leader_idx:
        ax3.plot(time_array, velocities_history[:, i, 0],
                 color='red', linestyle='-', linewidth=2)
    for i in follower_idx:
        ax3.plot(time_array, velocities_history[:, i, 0],
                 color='blue', linestyle='--', linewidth=2)
    ax3.plot([], [], color='red', linestyle='-', linewidth=2, label='Leader')
    ax3.plot([], [], color='blue', linestyle='--', linewidth=2, label='Follower')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('x-velocity (m/s)')
    ax3.set_title('(c) x-velocity')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Fig. 4(d): y-velocity
    ax4 = axes[1, 1]
    for i in leader_idx:
        ax4.plot(time_array, velocities_history[:, i, 1],
                 color='red', linestyle='-', linewidth=2)
    for i in follower_idx:
        ax4.plot(time_array, velocities_history[:, i, 1],
                 color='blue', linestyle='--', linewidth=2)
    ax4.plot([], [], color='red', linestyle='-', linewidth=2, label='Leader')
    ax4.plot([], [], color='blue', linestyle='--', linewidth=2, label='Follower')
    ax4.set_xlabel('Time (sec)')
    ax4.set_ylabel('y-velocity (m/s)')
    ax4.set_title('(d) y-velocity')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('/home/ronghai/bearing_control/outputs/artifacts/S04/fig4_simulation.png',
                dpi=150, bbox_inches='tight')
    plt.close()

    print("Figure 4 saved to outputs/artifacts/S04/fig4_simulation.png")


def generate_gif():
    """Generate animated GIF showing formation moving with trajectory trails."""
    import io
    from PIL import Image

    time_array, positions_history, velocities_history, bearing_errors = run_simulation()

    n_frames = 150
    frame_indices = np.linspace(0, len(time_array) - 1, n_frames, dtype=int)
    trail_length = 50

    leader_idx = [1, 2]
    follower_idx = [0, 3]
    snapshot_edges = [(0, 1), (0, 2), (0, 3), (1, 2), (2, 3)]

    all_x = positions_history[:, :, 0]
    all_y = positions_history[:, :, 1]
    x_margin = (all_x.max() - all_x.min()) * 0.1
    y_margin = (all_y.max() - all_y.min()) * 0.1
    x_lim = (all_x.min() - x_margin, all_x.max() + x_margin)
    y_lim = (all_y.min() - y_margin, all_y.max() + y_margin)

    fig, ax = plt.subplots(figsize=(10, 8))
    frames = []

    for fi, frame_i in enumerate(frame_indices):
        ax.clear()

        trail_start = max(0, fi - trail_length)
        for agent in range(4):
            color = 'red' if agent in leader_idx else 'blue'
            for t_idx in range(trail_start, fi):
                alpha = 0.8 * (t_idx - trail_start) / max(1, fi - trail_start)
                pos = positions_history[frame_indices[t_idx], agent]
                ax.plot(pos[0], pos[1], 'o', color=color, markersize=3, alpha=alpha)

        pos = positions_history[frame_i]
        for (i, j) in snapshot_edges:
            ax.plot([pos[i, 0], pos[j, 0]], [pos[i, 1], pos[j, 1]], 'k-', linewidth=1)

        for i in leader_idx:
            ax.plot(pos[i, 0], pos[i, 1], marker='>', color='red',
                    markersize=8, markerfacecolor='red', zorder=5)
        for i in follower_idx:
            ax.plot(pos[i, 0], pos[i, 1], marker='o', color='blue',
                    markersize=8, markerfacecolor='blue', zorder=5)

        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')

        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        frames.append(Image.open(buf).copy())
        buf.close()

    plt.close(fig)

    output_path = '/home/ronghai/bearing_control/outputs/artifacts/S04/fig4_simulation.gif'
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=67,
        loop=0
    )
    print(f"GIF saved to {output_path}")


if __name__ == "__main__":
    print("Running Fig. 4 simulation...")
    time_array, positions_history, velocities_history, bearing_errors = run_simulation()
    plot_results(time_array, positions_history, velocities_history, bearing_errors)
    generate_gif()
    print("Simulation complete!")
