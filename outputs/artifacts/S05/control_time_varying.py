"""
Control Law (10) for Time-Varying Leader Velocity

The bearing-based formation control law with acceleration feedback for
double-integrator dynamics when the leader velocity is time-varying.

u_i = -K_i^{-1} * sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) - v_j_dot]

where K_i = sum_{j∈N_i} P_g*_ij

References:
- Section III-B, pp. 7-8
- Lemma 3: K_i is nonsingular for unique target formation
- Theorem 4: Global exponential time-varying leader velocity
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_bearing, compute_projection_matrix
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian


class ControlLaw10:
    """
    Bearing-based formation control law (10) for time-varying leader velocity.
    
    u_i = -K_i^{-1} * sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) - v_j_dot]
    
    This control law requires:
    - Relative position feedback
    - Relative velocity feedback
    - Acceleration feedback from neighbors
    - Desired bearings g*_ij (constant)
    
    Suitable for: Time-varying leader velocity (v_l dot ≠ 0)
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
        
        # Pre-compute K_i matrices for each follower
        self.K_matrices = {}
        for i in graph.follower_indices:
            K_i = np.zeros((len(desired_bearings[list(desired_bearings.keys())[0]]),
                           len(desired_bearings[list(desired_bearings.keys())[0]])))
            for j in graph.neighbors[i]:
                if (i, j) in self.projections:
                    K_i += self.projections[(i, j)]
            self.K_matrices[i] = K_i
    
    def compute_control(self, state: FormationState, agent_idx: int, 
                       neighbor_accelerations: Dict[int, np.ndarray] = None) -> np.ndarray:
        """
        Compute control input for a follower agent.
        
        u_i = -K_i^{-1} * sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) - v_j_dot]
        
        Args:
            state: Current formation state
            agent_idx: Index of the follower agent
            neighbor_accelerations: Dictionary mapping neighbor index to acceleration
        
        Returns:
            Control input u_i (d-dimensional vector)
        """
        if agent_idx in self.graph.leader_indices:
            # Leaders have prescribed motion, no control input
            return np.zeros(state.d)
        
        p_i = state.positions[agent_idx]
        v_i = state.velocities[agent_idx]
        
        # Sum term
        sum_term = np.zeros(state.d)
        
        for j in self.graph.neighbors[agent_idx]:
            if (agent_idx, j) in self.projections:
                P_g = self.projections[(agent_idx, j)]
                p_j = state.positions[j]
                v_j = state.velocities[j]
                
                # Position and velocity errors
                dp = p_i - p_j
                dv = v_i - v_j
                
                # Acceleration of neighbor (if available)
                v_j_dot = np.zeros(state.d)
                if neighbor_accelerations and j in neighbor_accelerations:
                    v_j_dot = neighbor_accelerations[j]
                
                # Apply projection and sum
                sum_term += P_g @ (self.k_p * dp + self.k_v * dv - v_j_dot)
        
        # Apply K_i^{-1}
        K_i = self.K_matrices[agent_idx]
        control = -np.linalg.solve(K_i, sum_term)
        
        return control
    
    def compute_all_controls(self, state: FormationState, 
                            leader_accelerations: np.ndarray = None) -> np.ndarray:
        """
        Compute control inputs for all followers.
        
        Args:
            state: Current formation state
            leader_accelerations: n_l x d array of leader accelerations
        
        Returns:
            Array of control inputs for all agents (n x d)
        """
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        # Prepare neighbor accelerations
        neighbor_accelerations = {}
        if leader_accelerations is not None:
            for idx, i in enumerate(self.graph.leader_indices):
                neighbor_accelerations[i] = leader_accelerations[idx]
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i, neighbor_accelerations)
        
        return controls
    
    def matrix_form_control(self, state: FormationState, 
                           leader_accelerations: np.ndarray = None) -> np.ndarray:
        """
        Compute control in matrix form (for stability analysis).
        
        B_ff * v_f_dot + B_fl * v_l_dot = -k_p * B_ff * delta_p - k_v * B_ff * delta_v
        
        Args:
            state: Current formation state
            leader_accelerations: Leader accelerations
        
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
        
        # Right-hand side
        rhs = -self.k_p * B_ff @ delta_p - self.k_v * B_ff @ delta_v
        
        # If leader accelerations are provided, account for them
        if leader_accelerations is not None:
            v_l_dot = leader_accelerations.flatten()
            rhs -= B_fl @ v_l_dot
        
        return rhs
    
    def error_dynamics(self, state: FormationState, 
                      leader_accelerations: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute error dynamics (for stability analysis).
        
        d/dt [delta_p; delta_v] = [0, I; -k_p*I, -k_v*I] * [delta_p; delta_v]
        
        Args:
            state: Current formation state
            leader_accelerations: Leader accelerations
        
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
        
        # Error dynamics (Theorem 4)
        # The acceleration feedback eliminates the v_l_dot term
        delta_p_dot = delta_v
        delta_v_dot = -self.k_p * delta_p - self.k_v * delta_v
        
        return delta_p_dot, delta_v_dot
