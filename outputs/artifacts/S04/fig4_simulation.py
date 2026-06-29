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
    """
    Setup the square formation from Fig. 2(b).
    
    The square formation has:
    - 4 agents in 2D
    - Agents 0, 1 are leaders (solid dots)
    - Agents 2, 3 are followers (hollow dots)
    - Desired bearings form a square pattern
    
    Returns:
        Tuple of (graph, desired_bearings, initial_positions, initial_velocities)
    """
    # Formation graph: square with 4 agents
    # Edges: (0,1), (1,2), (2,3), (3,0), (0,2), (1,3)
    # Leaders: 0, 1; Followers: 2, 3
    n_agents = 4
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]
    leader_indices = [0, 1]
    
    graph = FormationGraph(n_agents, edges, leader_indices)
    
    # Desired bearings for a unit square
    # The square is centered at origin with side length 1
    # Agent positions (desired pattern):
    # 0: (-0.5, 0.5), 1: (0.5, 0.5), 2: (0.5, -0.5), 3: (-0.5, -0.5)
    
    # Compute desired bearings
    desired_bearings = {}
    
    # Edge (0,1): from top-left to top-right
    desired_bearings[(0, 1)] = np.array([1.0, 0.0])
    
    # Edge (1,2): from top-right to bottom-right
    desired_bearings[(1, 2)] = np.array([0.0, -1.0])
    
    # Edge (2,3): from bottom-right to bottom-left
    desired_bearings[(2, 3)] = np.array([-1.0, 0.0])
    
    # Edge (3,0): from bottom-left to top-left
    desired_bearings[(3, 0)] = np.array([0.0, 1.0])
    
    # Edge (0,2): diagonal from top-left to bottom-right
    desired_bearings[(0, 2)] = np.array([1.0/np.sqrt(2), -1.0/np.sqrt(2)])
    
    # Edge (1,3): diagonal from top-right to bottom-left
    desired_bearings[(1, 3)] = np.array([-1.0/np.sqrt(2), -1.0/np.sqrt(2)])
    
    # Initial positions (arbitrary, not at target)
    initial_positions = np.array([
        [0.0, 2.0],   # Leader 0
        [2.0, 2.0],   # Leader 1
        [2.5, 0.0],   # Follower 2
        [0.0, 0.0],   # Follower 3
    ])
    
    # Initial velocities (zero)
    initial_velocities = np.zeros((n_agents, 2))
    
    return graph, desired_bearings, initial_positions, initial_velocities


def leader_motion(t: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Define leader motion for the simulation.
    
    The leaders move such that the formation:
    1. Translates in x-direction
    2. Scales (expands/contracts)
    
    From the paper description:
    - Translation velocity: v_c = [0.5, 0] (constant)
    - Scaling rate: alpha = -0.1 (contraction)
    
    Args:
        t: Time
    
    Returns:
        Tuple of (leader_positions, leader_velocities)
    """
    # Constant translational velocity
    v_c = np.array([0.5, 0.0])
    
    # Time-varying scaling rate (for demonstration)
    # The paper mentions scaling velocity in x-direction is negative
    alpha = -0.1
    
    # Centroid motion
    c = v_c * t
    
    # Leader positions (following desired pattern with time-varying scale)
    scale = 1.0 + 0.1 * np.sin(0.5 * t)  # Oscillating scale for demonstration
    
    leader_positions = np.array([
        c + scale * np.array([-0.5, 0.5]),  # Leader 0
        c + scale * np.array([0.5, 0.5]),   # Leader 1
    ])
    
    # Leader velocities (from Theorem 2, Eq. 6)
    # v_i = v_c + alpha * (p_i - c)
    leader_velocities = np.array([
        v_c + alpha * (leader_positions[0] - c),
        v_c + alpha * (leader_positions[1] - c),
    ])
    
    return leader_positions, leader_velocities


def run_simulation():
    """Run the simulation and generate figures."""
    # Setup
    graph, desired_bearings, initial_positions, initial_velocities = setup_square_formation()
    
    # Simulation parameters
    k_p = 0.5
    k_v = 2.0
    dt = 0.01
    T = 10.0
    n_steps = int(T / dt) + 1
    
    # Initialize history arrays
    time_array = np.linspace(0, T, n_steps)
    positions_history = np.zeros((n_steps, 4, 2))
    velocities_history = np.zeros((n_steps, 4, 2))
    bearing_errors = np.zeros((n_steps,))
    
    # Set initial conditions
    positions = initial_positions.copy()
    velocities = initial_velocities.copy()
    
    # Create control law
    control_law = ControlLaw7(graph, desired_bearings, k_p, k_v)
    
    # Simulation loop
    for step in range(n_steps):
        t = time_array[step]
        
        # Get leader motion
        leader_pos, leader_vel = leader_motion(t)
        
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
