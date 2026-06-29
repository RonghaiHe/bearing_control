"""
Simulation of Fig. 5: 3D Cube Formation with Time-Varying Leader Velocity

This script reproduces the simulation example from Section III-B demonstrating
control law (10) for a 3D cube formation with 2 leaders and 6 followers.

The formation converges from an initial configuration to the desired pattern
while translating and scaling.

References:
- Section III-B, Fig. 5, pp. 8-9
- 3D cube formation (Fig. 2(c))
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
from S05.control_time_varying import ControlLaw10


def setup_cube_formation() -> Tuple[FormationGraph, Dict[Tuple[int, int], np.ndarray], np.ndarray, np.ndarray]:
    """
    Setup the 3D cube formation from Fig. 2(c).
    
    The cube formation has:
    - 8 agents in 3D
    - Agents 0, 1 are leaders (solid dots)
    - Agents 2-7 are followers (hollow dots)
    - Desired bearings form a cube pattern
    
    Returns:
        Tuple of (graph, desired_bearings, initial_positions, initial_velocities)
    """
    # Formation graph: cube with 8 agents
    # Leaders: 0, 1; Followers: 2-7
    n_agents = 8
    
    # Edges: Each agent connects to its neighbors in the cube
    # Cube vertices: 0=(0,0,0), 1=(1,0,0), 2=(1,1,0), 3=(0,1,0),
    #                4=(0,0,1), 5=(1,0,1), 6=(1,1,1), 7=(0,1,1)
    edges = []
    
    # Connect each vertex to its 3 neighbors (along cube edges)
    for i in range(8):
        for j in range(8):
            if i < j:
                # Check if i and j differ in exactly one coordinate
                diff = i ^ j  # XOR
                if diff in [1, 2, 4]:  # Powers of 2
                    edges.append((i, j))
                    edges.append((j, i))  # Bidirectional
    
    leader_indices = [0, 1]
    
    graph = FormationGraph(n_agents, edges, leader_indices)
    
    # Desired bearings for a unit cube
    # Cube vertices (desired pattern):
    cube_vertices = np.array([
        [0.0, 0.0, 0.0],  # 0
        [1.0, 0.0, 0.0],  # 1
        [1.0, 1.0, 0.0],  # 2
        [0.0, 1.0, 0.0],  # 3
        [0.0, 0.0, 1.0],  # 4
        [1.0, 0.0, 1.0],  # 5
        [1.0, 1.0, 1.0],  # 6
        [0.0, 1.0, 1.0],  # 7
    ])
    
    # Compute desired bearings
    desired_bearings = {}
    for (i, j) in edges:
        desired_bearings[(i, j)] = compute_bearing(cube_vertices[i], cube_vertices[j])
    
    # Initial positions (random, not at target)
    np.random.seed(42)
    initial_positions = cube_vertices + 2.0 * np.random.randn(n_agents, 3)
    
    # Initial velocities (zero)
    initial_velocities = np.zeros((n_agents, 3))
    
    return graph, desired_bearings, initial_positions, initial_velocities


def leader_motion_3d(t: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Define leader motion for the 3D simulation.
    
    The leaders move such that the formation:
    1. Translates in all three dimensions
    2. Scales (expands/contracts)
    
    Args:
        t: Time
    
    Returns:
        Tuple of (leader_positions, leader_velocities)
    """
    # Time-varying translational velocity
    v_c = np.array([
        0.3 * np.sin(0.2 * t),
        0.2 * np.cos(0.3 * t),
        0.1 * np.sin(0.1 * t)
    ])
    
    # Time-varying scaling rate
    alpha = 0.05 * np.sin(0.5 * t)
    
    # Centroid motion (integral of velocity)
    c = np.array([
        -1.5 * np.cos(0.2 * t) + 1.5,
        0.67 * np.sin(0.3 * t),
        -1.0 * np.cos(0.1 * t) + 1.0
    ])
    
    # Leader positions (following desired pattern with time-varying scale)
    scale = 1.0 + 0.2 * np.sin(0.5 * t)  # Oscillating scale
    
    # Leader 0 at (0,0,0), Leader 1 at (1,0,0) in cube coordinates
    leader_positions = np.array([
        c + scale * np.array([0.0, 0.0, 0.0]),  # Leader 0
        c + scale * np.array([1.0, 0.0, 0.0]),  # Leader 1
    ])
    
    # Leader velocities (from Theorem 2, Eq. 6)
    # v_i = v_c + alpha * (p_i - c)
    leader_velocities = np.array([
        v_c + alpha * (leader_positions[0] - c),
        v_c + alpha * (leader_positions[1] - c),
    ])
    
    return leader_positions, leader_velocities


def run_simulation_3d():
    """Run the 3D simulation and generate figures."""
    # Setup
    graph, desired_bearings, initial_positions, initial_velocities = setup_cube_formation()
    
    # Simulation parameters
    k_p = 0.5
    k_v = 2.0
    dt = 0.01
    T = 15.0
    n_steps = int(T / dt) + 1
    
    # Initialize history arrays
    time_array = np.linspace(0, T, n_steps)
    positions_history = np.zeros((n_steps, 8, 3))
    velocities_history = np.zeros((n_steps, 8, 3))
    bearing_errors = np.zeros((n_steps,))
    
    # Set initial conditions
    positions = initial_positions.copy()
    velocities = initial_velocities.copy()
    
    # Create control law
    control_law = ControlLaw10(graph, desired_bearings, k_p, k_v)
    
    # Simulation loop
    for step in range(n_steps):
        t = time_array[step]
        
        # Get leader motion
        leader_pos, leader_vel = leader_motion_3d(t)
        
        # Update leader positions/velocities
        for idx, i in enumerate(graph.leader_indices):
            positions[i] = leader_pos[idx]
            velocities[i] = leader_vel[idx]
        
        # Store current state
        positions_history[step] = positions.copy()
        velocities_history[step] = velocities.copy()
        
        # Compute bearing error
        total_error = 0.0
        for (i, j), g_desired in desired_bearings.items():
            g_actual = compute_bearing(positions[i], positions[j])
            total_error += np.linalg.norm(g_actual - g_desired)
        bearing_errors[step] = total_error
        
        # Create formation state
        state = FormationState(graph, positions, velocities)
        
        # Compute controls for followers
        controls = control_law.compute_all_controls(state)
        
        # Update follower dynamics (double integrator)
        for i in graph.follower_indices:
            positions[i] += velocities[i] * dt
            velocities[i] += controls[i] * dt
    
    return time_array, positions_history, velocities_history, bearing_errors


def plot_results_3d(time_array, positions_history, velocities_history, bearing_errors):
    """Generate figures matching Fig. 5 in the paper."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    colors = ['blue', 'red'] + ['green', 'orange', 'purple', 'cyan', 'magenta', 'brown']
    labels = ['Leader 0', 'Leader 1'] + [f'Follower {i}' for i in range(2, 8)]
    
    # Fig. 5(a): XY projection of trajectory
    ax1 = axes[0, 0]
    for i in range(8):
        ax1.plot(positions_history[:, i, 0], positions_history[:, i, 1],
                color=colors[i], label=labels[i], linewidth=1.5)
    
    # Mark start and end points
    for i in range(8):
        ax1.scatter(positions_history[0, i, 0], positions_history[0, i, 1],
                   color=colors[i], marker='o', s=50, zorder=5)
        ax1.scatter(positions_history[-1, i, 0], positions_history[-1, i, 1],
                   color=colors[i], marker='x', s=50, zorder=5)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('(a) Trajectory (XY projection)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    ax2 = axes[0, 1]
    ax2.plot(time_array, bearing_errors, 'b-', linewidth=2)
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Total Bearing Error')
    ax2.set_title(r'(b) Total Bearing Error')
    ax2.grid(True, alpha=0.3)
    
    ax3 = axes[1, 0]
    for i in range(8):
        ax3.plot(time_array, velocities_history[:, i, 0], 
                color=colors[i], label=labels[i], linewidth=1.5)
    
    ax3.set_xlabel('Time')
    ax3.set_ylabel('x-velocity')
    ax3.set_title('(c) Velocity (x-component)')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    
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
    plt.savefig('/home/ronghai/bearing_control/outputs/artifacts/S06/fig5_simulation.png', 
                dpi=150, bbox_inches='tight')
    plt.close()


if __name__ == "__main__":
    print("Running Fig. 5 simulation (3D cube formation)...")
    time_array, positions_history, velocities_history, bearing_errors = run_simulation_3d()
    plot_results_3d(time_array, positions_history, velocities_history, bearing_errors)
    print("Simulation complete!")
