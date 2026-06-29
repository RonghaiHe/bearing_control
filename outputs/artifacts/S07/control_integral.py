"""
Control Law (12) with Integral Control for Input Disturbances

The bearing-based formation control law with integral term to handle
constant input disturbances and constant leader acceleration.

u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) + k_I * integral(p_i - p_j)]

References:
- Section IV-A, pp. 9-10
- Theorem 5: Stability condition 0 < k_I < k_p * k_v * lambda_min(B_ff)
- Theorem 6: Extension to time-varying v_l
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_projection_matrix
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian


class ControlLaw12:
    """
    Bearing-based formation control law (12) with integral control.
    
    u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) + k_I * integral(p_i - p_j)]
    
    This control law handles:
    - Constant input disturbances (sensor/actuator biases)
    - Constant leader acceleration
    
    Stability condition: 0 < k_I < k_p * k_v * lambda_min(B_ff)
    """
    
    def __init__(self, graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                 k_p: float = 1.0, k_v: float = 1.0, k_I: float = 0.1):
        """
        Args:
            graph: Formation graph topology
            desired_bearings: Desired bearing vectors for each edge
            k_p: Position control gain (positive)
            k_v: Velocity control gain (positive)
            k_I: Integral control gain (positive, must satisfy stability condition)
        """
        self.graph = graph
        self.desired_bearings = desired_bearings
        self.k_p = k_p
        self.k_v = k_v
        self.k_I = k_I
        
        # Pre-compute projection matrices
        self.projections = {}
        for (i, j), g in desired_bearings.items():
            self.projections[(i, j)] = compute_projection_matrix(g)
        
        # Integral state for each follower
        self.integral_states = {i: np.zeros(len(list(desired_bearings.values())[0])) 
                               for i in graph.follower_indices}
    
    def reset_integrators(self):
        """Reset all integral states to zero."""
        for i in self.graph.follower_indices:
            self.integral_states[i] = np.zeros_like(self.integral_states[i])
    
    def compute_control(self, state: FormationState, agent_idx: int, 
                       dt: float = 0.01) -> np.ndarray:
        """
        Compute control input for a follower agent.
        
        u_i = -sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) + k_I * integral(p_i - p_j)]
        
        Args:
            state: Current formation state
            agent_idx: Index of the follower agent
            dt: Time step for integral update
        
        Returns:
            Control input u_i (d-dimensional vector)
        """
        if agent_idx in self.graph.leader_indices:
            return np.zeros(state.d)
        
        p_i = state.positions[agent_idx]
        v_i = state.velocities[agent_idx]
        
        control = np.zeros(state.d)
        
        for j in self.graph.neighbors[agent_idx]:
            if (agent_idx, j) in self.projections:
                P_g = self.projections[(agent_idx, j)]
                p_j = state.positions[j]
                v_j = state.velocities[j]
                
                dp = p_i - p_j
                dv = v_i - v_j
                
                # Update integral state
                self.integral_states[agent_idx] += dp * dt
                
                # Apply projection and sum
                control += P_g @ (self.k_p * dp + self.k_v * dv + 
                                 self.k_I * self.integral_states[agent_idx])
        
        return -control
    
    def compute_all_controls(self, state: FormationState, dt: float = 0.01) -> np.ndarray:
        """Compute control inputs for all followers."""
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i, dt)
        
        return controls


class ControlLaw14:
    """
    Extension of control law (12) for time-varying leader velocity (Eq. 14).
    
    u_i = -K_i^{-1} * sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j) - v_j_dot 
           + k_I * integral(p_i - p_j)]
    
    Handles both constant input disturbances and time-varying v_l.
    Stability condition: 0 < k_I < k_p * k_v
    """
    
    def __init__(self, graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                 k_p: float = 1.0, k_v: float = 1.0, k_I: float = 0.1):
        self.graph = graph
        self.desired_bearings = desired_bearings
        self.k_p = k_p
        self.k_v = k_v
        self.k_I = k_I
        
        self.projections = {}
        for (i, j), g in desired_bearings.items():
            self.projections[(i, j)] = compute_projection_matrix(g)
        
        # Pre-compute K_i matrices
        self.K_matrices = {}
        d = len(list(desired_bearings.values())[0])
        for i in graph.follower_indices:
            K_i = np.zeros((d, d))
            for j in graph.neighbors[i]:
                if (i, j) in self.projections:
                    K_i += self.projections[(i, j)]
            self.K_matrices[i] = K_i
        
        self.integral_states = {i: np.zeros(d) for i in graph.follower_indices}
    
    def reset_integrators(self):
        for i in self.graph.follower_indices:
            self.integral_states[i] = np.zeros_like(self.integral_states[i])
    
    def compute_control(self, state: FormationState, agent_idx: int,
                       neighbor_accelerations: Dict[int, np.ndarray] = None,
                       dt: float = 0.01) -> np.ndarray:
        if agent_idx in self.graph.leader_indices:
            return np.zeros(state.d)
        
        p_i = state.positions[agent_idx]
        v_i = state.velocities[agent_idx]
        
        sum_term = np.zeros(state.d)
        
        for j in self.graph.neighbors[agent_idx]:
            if (agent_idx, j) in self.projections:
                P_g = self.projections[(agent_idx, j)]
                p_j = state.positions[j]
                v_j = state.velocities[j]
                
                dp = p_i - p_j
                dv = v_i - v_j
                
                self.integral_states[agent_idx] += dp * dt
                
                v_j_dot = np.zeros(state.d)
                if neighbor_accelerations and j in neighbor_accelerations:
                    v_j_dot = neighbor_accelerations[j]
                
                sum_term += P_g @ (self.k_p * dp + self.k_v * dv - v_j_dot +
                                  self.k_I * self.integral_states[agent_idx])
        
        K_i = self.K_matrices[agent_idx]
        return -np.linalg.solve(K_i, sum_term)
    
    def compute_all_controls(self, state: FormationState,
                            leader_accelerations: np.ndarray = None,
                            dt: float = 0.01) -> np.ndarray:
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        neighbor_accelerations = {}
        if leader_accelerations is not None:
            for idx, i in enumerate(self.graph.leader_indices):
                neighbor_accelerations[i] = leader_accelerations[idx]
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i, neighbor_accelerations, dt)
        
        return controls


def check_stability_condition(k_p: float, k_v: float, k_I: float, 
                             B_ff_eigenvalues: np.ndarray, is_time_varying: bool = False) -> bool:
    """
    Check the stability condition for integral control.
    
    For constant v_l (Theorem 5): 0 < k_I < k_p * k_v * lambda_min(B_ff)
    For time-varying v_l (Theorem 6): 0 < k_I < k_p * k_v
    
    Args:
        k_p: Position gain
        k_v: Velocity gain
        k_I: Integral gain
        B_ff_eigenvalues: Eigenvalues of B_ff
        is_time_varying: Whether leader velocity is time-varying
    
    Returns:
        True if stability condition is satisfied
    """
    lambda_min = np.min(B_ff_eigenvalues)
    
    if is_time_varying:
        return 0 < k_I < k_p * k_v
    else:
        return 0 < k_I < k_p * k_v * lambda_min
