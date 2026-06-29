"""
Collision Avoidance Conditions Verification

Implements and verifies the collision-free conditions from Theorems 9 and 10.
These provide sufficient conditions on initial formation to ensure no collision
between any pair of agents.

References:
- Section IV-C, pp. 12-13
- Theorem 9: Collision-free condition for constant v_l (control law 7)
- Theorem 10: Collision-free condition for time-varying v_l (control law 10)
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, FormationState, compute_bearing
from S02.bearing_laplacian import compute_bearing_laplacian, partition_bearing_laplacian


def compute_minimum_pairwise_distance(positions: np.ndarray) -> float:
    """
    Compute minimum distance between any pair of agents.
    
    min_{i,j} ||p_i - p_j||
    
    Args:
        positions: n x d array of agent positions
    
    Returns:
        Minimum pairwise distance
    """
    n = positions.shape[0]
    min_dist = float('inf')
    
    for i in range(n):
        for j in range(i + 1, n):
            dist = np.linalg.norm(positions[i] - positions[j])
            min_dist = min(min_dist, dist)
    
    return min_dist


def verify_theorem9(positions: np.ndarray, velocities: np.ndarray,
                   graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                   target_positions: np.ndarray, gamma: float,
                   k_p: float = 1.0) -> dict:
    """
    Verify collision-free condition from Theorem 9.
    
    Condition (19):
    k_p * delta_p(0)^T * B_ff * delta_p(0) + delta_v(0)^T * delta_v(0)
    < (k_p * lambda_min(B_ff) / n_f) * (min_{i,j} ||p_i*(t) - p_j*(t)|| - gamma)^2
    
    Args:
        positions: Initial positions (n x d)
        velocities: Initial velocities (n x d)
        graph: Formation graph
        desired_bearings: Desired bearings
        target_positions: Target formation positions
        gamma: Desired minimum distance
        k_p: Position gain
    
    Returns:
        Dictionary with verification results
    """
    n = graph.n
    d = positions.shape[1]
    n_f = graph.n_f
    
    # Compute bearing Laplacian
    B = compute_bearing_laplacian(graph, desired_bearings)
    B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, graph)
    
    # Compute initial tracking errors
    p_f = positions[graph.follower_indices].flatten()
    p_l = positions[graph.leader_indices].flatten()
    v_f = velocities[graph.follower_indices].flatten()
    v_l = velocities[graph.leader_indices].flatten()
    
    delta_p_0 = p_f + np.linalg.solve(B_ff, B_fl @ p_l)
    delta_v_0 = v_f + np.linalg.solve(B_ff, B_fl @ v_l)
    
    # Left-hand side of (19)
    lhs = k_p * delta_p_0.T @ B_ff @ delta_p_0 + delta_v_0.T @ delta_v_0
    
    # Compute minimum distance in target formation
    min_target_dist = compute_minimum_pairwise_distance(target_positions)
    
    # Right-hand side of (19)
    lambda_min = np.min(np.linalg.eigvalsh(B_ff))
    rhs = (k_p * lambda_min / n_f) * (min_target_dist - gamma) ** 2
    
    condition_satisfied = lhs < rhs
    
    return {
        'theorem': 9,
        'lhs': lhs,
        'rhs': rhs,
        'condition_satisfied': condition_satisfied,
        'min_target_distance': min_target_dist,
        'lambda_min_Bff': lambda_min,
        'gamma': gamma,
        'margin': rhs - lhs
    }


def verify_theorem10(positions: np.ndarray, velocities: np.ndarray,
                    graph: FormationGraph, desired_bearings: Dict[Tuple[int, int], np.ndarray],
                    target_positions: np.ndarray, gamma: float,
                    k_p: float = 1.0) -> dict:
    """
    Verify collision-free condition from Theorem 10.
    
    Condition:
    k_p * delta_p(0)^T * delta_p(0) + delta_v(0)^T * delta_v(0)
    < (k_p / n_f) * (min_{i,j} ||p_i*(t) - p_j*(t)|| - gamma)^2
    
    Args:
        positions: Initial positions (n x d)
        velocities: Initial velocities (n x d)
        graph: Formation graph
        desired_bearings: Desired bearings
        target_positions: Target formation positions
        gamma: Desired minimum distance
        k_p: Position gain
    
    Returns:
        Dictionary with verification results
    """
    n = graph.n
    d = positions.shape[1]
    n_f = graph.n_f
    
    # Compute bearing Laplacian
    B = compute_bearing_laplacian(graph, desired_bearings)
    B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, graph)
    
    # Compute initial tracking errors
    p_f = positions[graph.follower_indices].flatten()
    p_l = positions[graph.leader_indices].flatten()
    v_f = velocities[graph.follower_indices].flatten()
    v_l = velocities[graph.leader_indices].flatten()
    
    delta_p_0 = p_f + np.linalg.solve(B_ff, B_fl @ p_l)
    delta_v_0 = v_f + np.linalg.solve(B_ff, B_fl @ v_l)
    
    # Left-hand side
    lhs = k_p * delta_p_0.T @ delta_p_0 + delta_v_0.T @ delta_v_0
    
    # Compute minimum distance in target formation
    min_target_dist = compute_minimum_pairwise_distance(target_positions)
    
    # Right-hand side
    rhs = (k_p / n_f) * (min_target_dist - gamma) ** 2
    
    condition_satisfied = lhs < rhs
    
    return {
        'theorem': 10,
        'lhs': lhs,
        'rhs': rhs,
        'condition_satisfied': condition_satisfied,
        'min_target_distance': min_target_dist,
        'gamma': gamma,
        'margin': rhs - lhs
    }


def check_collision_during_simulation(positions_history: np.ndarray, 
                                     gamma: float = 0.0) -> dict:
    """
    Check if any collision occurs during simulation.
    
    Args:
        positions_history: T x n x d array of positions over time
        gamma: Minimum allowed distance
    
    Returns:
        Dictionary with collision check results
    """
    n_steps, n_agents, d = positions_history.shape
    
    min_distance = float('inf')
    min_distance_time = 0
    collision_occurred = False
    
    for step in range(n_steps):
        positions = positions_history[step]
        current_min = compute_minimum_pairwise_distance(positions)
        
        if current_min < min_distance:
            min_distance = current_min
            min_distance_time = step
        
        if current_min < gamma:
            collision_occurred = True
    
    return {
        'collision_occurred': collision_occurred,
        'minimum_distance': min_distance,
        'minimum_distance_time_step': min_distance_time,
        'gamma': gamma,
        'safety_margin': min_distance - gamma
    }


def analyze_collision_safety(graph: FormationGraph, desired_bearings: Dict,
                            positions_history: np.ndarray) -> dict:
    """
    Comprehensive collision safety analysis.
    
    Args:
        graph: Formation graph
        desired_bearings: Desired bearings
        positions_history: Simulation positions
    
    Returns:
        Complete analysis results
    """
    # Compute target positions (assuming leaders follow desired pattern)
    target_positions = positions_history[-1]  # Use final positions as target
    
    # Check collision during simulation
    collision_check = check_collision_during_simulation(positions_history, gamma=0.0)
    
    # Compute initial tracking error
    initial_positions = positions_history[0]
    B = compute_bearing_laplacian(graph, desired_bearings)
    B_ll, B_lf, B_fl, B_ff = partition_bearing_laplacian(B, graph)
    
    p_f = initial_positions[graph.follower_indices].flatten()
    p_l = initial_positions[graph.leader_indices].flatten()
    
    delta_p_norm = np.linalg.norm(p_f + np.linalg.solve(B_ff, B_fl @ p_l))
    
    return {
        'collision_check': collision_check,
        'initial_tracking_error_norm': delta_p_norm,
        'n_agents': graph.n,
        'n_followers': graph.n_f
    }
