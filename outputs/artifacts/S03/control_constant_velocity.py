"""
Control Law (7) for Constant Leader Velocity

The bearing-based formation control law for double-integrator dynamics
when the leader velocity is constant.

u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]

References:
- Section III-A, pp. 5-6
- Theorem 3: Global exponential stability for constant v_l
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_bearing, compute_projection_matrix
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian


class ControlLaw7:
    """
    Bearing-based formation control law (7) for constant leader velocity.
    
    u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]
    
    This control law requires:
    - Relative position feedback
    - Relative velocity feedback
    - Desired bearings g*_ij (constant)
    
    Suitable for: Constant leader velocity (v_l dot = 0)
    """
    
    def __init__(self, graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                 k_p: float = 1.0, k_v: float = 1.0):
        """
        Args:
            graph: Formation graph topology
            desired_bearings: Desired bearing vectors for each edge
            k_p: Position control gain (positive)
            k_v: Velocity control gain (positive)
        """
        self.graph = graph
        self.desired_bearings = desired_bearings
        self.k_p = k_p
        self.k_v = k_v
        
        # Pre-compute projection matrices
        self.projections = {}
        for (i, j), g in desired_bearings.items():
            self.projections[(i, j)] = compute_projection_matrix(g)
    
    def compute_control(self, state: FormationState, agent_idx: int) -> np.ndarray:
        """
        Compute control input for a follower agent.
        
        u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]
        
        Args:
            state: Current formation state
            agent_idx: Index of the follower agent
        
        Returns:
            Control input u_i (d-dimensional vector)
        """
        if agent_idx in self.graph.leader_indices:
            # Leaders have prescribed motion, no control input
            return np.zeros(state.d)
        
        p_i = state.positions[agent_idx]
        v_i = state.velocities[agent_idx]
        
        control = np.zeros(state.d)
        
        for j in self.graph.neighbors[agent_idx]:
            if (agent_idx, j) in self.projections:
                P_g = self.projections[(agent_idx, j)]
                p_j = state.positions[j]
                v_j = state.velocities[j]
                
                # Position and velocity errors
                dp = p_i - p_j
                dv = v_i - v_j
                
                # Apply projection and sum
                control += P_g @ (self.k_p * dp + self.k_v * dv)
        
        return -control
    
    def compute_all_controls(self, state: FormationState) -> np.ndarray:
        """
        Compute control inputs for all followers.
        
        Args:
            state: Current formation state
        
        Returns:
            Array of control inputs for all agents (n x d)
        """
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i)
        
        return controls
    
    def matrix_form_control(self, state: FormationState) -> np.ndarray:
        """
        Compute control in matrix form (for stability analysis).
        
        u_f = -k_p * B_ff * delta_p - k_v * B_ff * delta_v
        
        where delta_p = p_f + B_ff^{-1} * B_fl * p_l
              delta_v = v_f + B_ff^{-1} * B_fl * v_l
        
        Args:
            state: Current formation state
        
        Returns:
            Control input vector (flattened)
        """
        # Get follower states
        p_f = state.get_follower_positions().flatten()
        v_f = state.get_follower_velocities().flatten()
        p_l = state.get_leader_positions().flatten()
        v_l = state.get_leader_velocities().flatten()
        
        # Compute bearing Laplacian
        B = compute_bearing_laplacian(self.graph, self.desired_bearings)
        B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, self.graph)
        
        # Compute tracking errors
        delta_p = p_f + np.linalg.solve(B_ff, B_fl @ p_l)
        delta_v = v_f + np.linalg.solve(B_ff, B_fl @ v_l)
        
        # Control in matrix form
        u_f = -self.k_p * B_ff @ delta_p - self.k_v * B_ff @ delta_v
        
        return u_f
    
    def error_dynamics(self, state: FormationState) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute error dynamics (for stability analysis).
        
        d/dt [delta_p; delta_v] = [0, I; -k_p*B_ff, -k_v*B_ff] * [delta_p; delta_v]
        
        Args:
            state: Current formation state
        
        Returns:
            (delta_p_dot, delta_v_dot) - time derivatives of tracking errors
        """
        # Get follower states
        p_f = state.get_follower_positions().flatten()
        v_f = state.get_follower_velocities().flatten()
        p_l = state.get_leader_positions().flatten()
        v_l = state.get_leader_velocities().flatten()
        
        # Compute bearing Laplacian
        B = compute_bearing_laplacian(self.graph, self.desired_bearings)
        B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, self.graph)
        
        # Compute tracking errors
        delta_p = p_f + np.linalg.solve(B_ff, B_fl @ p_l)
        delta_v = v_f + np.linalg.solve(B_ff, B_fl @ v_l)
        
        # Error dynamics (for constant v_l, dot_v_l = 0)
        delta_p_dot = delta_v
        delta_v_dot = -self.k_p * B_ff @ delta_p - self.k_v * B_ff @ delta_v
        
        return delta_p_dot, delta_v_dot


def simulate_formation_dynamics(graph: FormationGraph, 
                               desired_bearings: Dict[Tuple[int, int], np.ndarray],
                               initial_positions: np.ndarray,
                               initial_velocities: np.ndarray,
                               leader_positions_fn,
                               leader_velocities_fn,
                               k_p: float = 1.0,
                               k_v: float = 1.0,
                               dt: float = 0.01,
                               T: float = 10.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate formation dynamics with control law (7).
    
    Args:
        graph: Formation graph topology
        desired_bearings: Desired bearing vectors
        initial_positions: n x d initial positions
        initial_velocities: n x d initial velocities
        leader_positions_fn: Function returning leader positions at time t
        leader_velocities_fn: Function returning leader velocities at time t
        k_p: Position gain
        k_v: Velocity gain
        dt: Time step
        T: Simulation duration
    
    Returns:
        Tuple of (time_array, positions_history, velocities_history)
    """
    n = graph.n
    d = initial_positions.shape[1]
    
    # Initialize history arrays
    n_steps = int(T / dt) + 1
    time_array = np.linspace(0, T, n_steps)
    positions_history = np.zeros((n_steps, n, d))
    velocities_history = np.zeros((n_steps, n, d))
    
    # Set initial conditions
    positions = initial_positions.copy()
    velocities = initial_velocities.copy()
    
    # Create control law
    control_law = ControlLaw7(graph, desired_bearings, k_p, k_v)
    
    for step in range(n_steps):
        t = time_array[step]
        
        # Store current state
        positions_history[step] = positions
        velocities_history[step] = velocities
        
        # Get leader motion
        leader_pos = leader_positions_fn(t)
        leader_vel = leader_velocities_fn(t)
        
        # Update leader positions/velocities
        for idx, i in enumerate(graph.leader_indices):
            positions[i] = leader_pos[idx]
            velocities[i] = leader_vel[idx]
        
        # Create formation state
        state = FormationState(graph, positions, velocities)
        
        # Compute controls for followers
        controls = control_law.compute_all_controls(state)
        
        # Update follower dynamics (double integrator)
        for i in graph.follower_indices:
            # d/dt p_i = v_i
            # d/dt v_i = u_i
            positions[i] += velocities[i] * dt
            velocities[i] += controls[i] * dt
        
        # Store final state
        if step == n_steps - 1:
            positions_history[step] = positions
            velocities_history[step] = velocities
    
    return time_array, positions_history, velocities_history
