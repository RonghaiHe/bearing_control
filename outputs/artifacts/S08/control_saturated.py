"""
Control Law (15) with Saturation for Acceleration Saturation

The bearing-based formation control law with saturation function for
bounded acceleration input in practical implementations.

u_i = sat{-sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]}

References:
- Section IV-B, pp. 10-12
- Lemma 4: Properties of integral function Phi(x)
- Lemma 5: Properties of saturation function
- Theorem 7: Global asymptotic stability for constant v_l
- Theorem 8: Global asymptotic stability for time-varying v_l
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_projection_matrix
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian


def saturation(x: np.ndarray, beta: float = 1.0, method: str = 'tanh') -> np.ndarray:
    """
    Saturation function for bounded acceleration.
    
    Two options:
    1. sat(x) = sign(x) * min(|x|, beta) (hard saturation)
    2. sat(x) = beta * tanh(x) (smooth saturation)
    
    Args:
        x: Input vector
        beta: Saturation bound
        method: 'hard' or 'tanh'
    
    Returns:
        Saturated output
    """
    if method == 'hard':
        return np.sign(x) * np.minimum(np.abs(x), beta)
    elif method == 'tanh':
        return beta * np.tanh(x)
    else:
        raise ValueError(f"Unknown saturation method: {method}")


def phi_function(x: np.ndarray, beta: float = 1.0, method: str = 'tanh') -> np.ndarray:
    """
    Integral of saturation function (Lemma 4).
    
    Phi(x) = integral_0^x sat(tau) dtau
    
    For tanh: Phi(x) = beta * log(cosh(x))
    
    Args:
        x: Input vector
        beta: Saturation bound
        method: 'hard' or 'tanh'
    
    Returns:
        Phi(x) (component-wise)
    """
    if method == 'hard':
        # For hard saturation: Phi(x) = 0.5 * (|x| + min(|x|, beta))^2 / |x| for x != 0
        # Simplified: Phi(x) = 0.5 * x * sat(x) + 0.5 * beta * min(|x|, beta)
        sat_x = saturation(x, beta, 'hard')
        return 0.5 * x * sat_x + 0.5 * beta * np.minimum(np.abs(x), beta)
    elif method == 'tanh':
        # For tanh: Phi(x) = beta * log(cosh(x))
        return beta * np.log(np.cosh(x))
    else:
        raise ValueError(f"Unknown method: {method}")


class ControlLaw15:
    """
    Bearing-based formation control law (15) with saturation.
    
    u_i = sat{-sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]}
    
    Handles bounded acceleration input in practical implementations.
    Global asymptotic stability via Lyapunov approach (Theorem 7).
    """
    
    def __init__(self, graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                 k_p: float = 1.0, k_v: float = 1.0, beta: float = 1.0, 
                 saturation_method: str = 'tanh'):
        """
        Args:
            graph: Formation graph topology
            desired_bearings: Desired bearing vectors for each edge
            k_p: Position control gain (positive)
            k_v: Velocity control gain (positive)
            beta: Saturation bound
            saturation_method: 'hard' or 'tanh'
        """
        self.graph = graph
        self.desired_bearings = desired_bearings
        self.k_p = k_p
        self.k_v = k_v
        self.beta = beta
        self.saturation_method = saturation_method
        
        self.projections = {}
        for (i, j), g in desired_bearings.items():
            self.projections[(i, j)] = compute_projection_matrix(g)
    
    def compute_control(self, state: FormationState, agent_idx: int) -> np.ndarray:
        """
        Compute saturated control input for a follower agent.
        
        u_i = sat{-sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]}
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
                
                control += P_g @ (self.k_p * dp + self.k_v * dv)
        
        # Apply saturation
        return -saturation(control, self.beta, self.saturation_method)
    
    def compute_all_controls(self, state: FormationState) -> np.ndarray:
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i)
        
        return controls
    
    def lyapunov_function(self, state: FormationState) -> float:
        """
        Compute Lyapunov function for stability analysis (Theorem 7).
        
        V = 1^T * Phi(-k_p*B_ff*delta_p - k_v*B_ff*delta_v)
          + 1^T * Phi(-k_p*B_ff*delta_p)
          + k_p * delta_v^T * B_ff * delta_v
        """
        p_f = state.get_follower_positions().flatten()
        v_f = state.get_follower_velocities().flatten()
        p_l = state.get_leader_positions().flatten()
        v_l = state.get_leader_velocities().flatten()
        
        B = compute_bearing_laplacian(self.graph, self.desired_bearings)
        B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, self.graph)
        
        delta_p = p_f + np.linalg.solve(B_ff, B_fl @ p_l)
        delta_v = v_f + np.linalg.solve(B_ff, B_fl @ v_l)
        
        term1 = np.sum(phi_function(-self.k_p * B_ff @ delta_p - self.k_v * B_ff @ delta_v,
                                   self.beta, self.saturation_method))
        term2 = np.sum(phi_function(-self.k_p * B_ff @ delta_p,
                                   self.beta, self.saturation_method))
        term3 = self.k_p * delta_v.T @ B_ff @ delta_v
        
        return term1 + term2 + term3


class ControlLaw18:
    """
    Extension for time-varying leader velocity with saturation (Eq. 18).
    
    u_i = K_i^{-1} * sat{-sum_{j∈N_i} P_g*_ij [k_p(p_i - p_j) + k_v(v_i - v_j)]}
        + K_i^{-1} * sum_{j∈N_i} P_g*_ij * v_j_dot
    """
    
    def __init__(self, graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                 k_p: float = 1.0, k_v: float = 1.0, beta: float = 1.0,
                 saturation_method: str = 'tanh'):
        self.graph = graph
        self.desired_bearings = desired_bearings
        self.k_p = k_p
        self.k_v = k_v
        self.beta = beta
        self.saturation_method = saturation_method
        
        self.projections = {}
        for (i, j), g in desired_bearings.items():
            self.projections[(i, j)] = compute_projection_matrix(g)
        
        d = len(list(desired_bearings.values())[0])
        self.K_matrices = {}
        for i in graph.follower_indices:
            K_i = np.zeros((d, d))
            for j in graph.neighbors[i]:
                if (i, j) in self.projections:
                    K_i += self.projections[(i, j)]
            self.K_matrices[i] = K_i
    
    def compute_control(self, state: FormationState, agent_idx: int,
                       neighbor_accelerations: Dict[int, np.ndarray] = None) -> np.ndarray:
        if agent_idx in self.graph.leader_indices:
            return np.zeros(state.d)
        
        p_i = state.positions[agent_idx]
        v_i = state.velocities[agent_idx]
        
        sum_term = np.zeros(state.d)
        accel_term = np.zeros(state.d)
        
        for j in self.graph.neighbors[agent_idx]:
            if (agent_idx, j) in self.projections:
                P_g = self.projections[(agent_idx, j)]
                p_j = state.positions[j]
                v_j = state.velocities[j]
                
                dp = p_i - p_j
                dv = v_i - v_j
                
                sum_term += P_g @ (self.k_p * dp + self.k_v * dv)
                
                v_j_dot = np.zeros(state.d)
                if neighbor_accelerations and j in neighbor_accelerations:
                    v_j_dot = neighbor_accelerations[j]
                accel_term += P_g @ v_j_dot
        
        K_i = self.K_matrices[agent_idx]
        sat_term = saturation(sum_term, self.beta, self.saturation_method)
        
        return -np.linalg.solve(K_i, sat_term) + np.linalg.solve(K_i, accel_term)
    
    def compute_all_controls(self, state: FormationState,
                            leader_accelerations: np.ndarray = None) -> np.ndarray:
        n = self.graph.n
        d = state.d
        controls = np.zeros((n, d))
        
        neighbor_accelerations = {}
        if leader_accelerations is not None:
            for idx, i in enumerate(self.graph.leader_indices):
                neighbor_accelerations[i] = leader_accelerations[idx]
        
        for i in self.graph.follower_indices:
            controls[i] = self.compute_control(state, i, neighbor_accelerations)
        
        return controls
