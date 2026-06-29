"""
Bearing Laplacian Matrix Implementation

The bearing Laplacian matrix B characterizes the interconnection topology
and the bearings of the formation. It plays a central role in bearing-based
formation control and network localization.

References:
- Section II-B, pp. 4-5
- Definition: B_ij = 0 for i≠j, (i,j)∉E
                B_ij = -P_g* for i≠j, (i,j)∈E
                B_ii = sum_{k∈N_i} P_g*_ik
"""

import numpy as np
from typing import List, Tuple, Dict
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from S01.formation_framework import FormationGraph, compute_bearing, compute_projection_matrix


def compute_bearing_laplacian(graph: FormationGraph, 
                              desired_bearings: Dict[Tuple[int, int], np.ndarray]) -> np.ndarray:
    """
    Compute the bearing Laplacian matrix B.
    
    The bearing Laplacian is a d*n x d*n matrix with block structure:
    - B_ij = 0 (dxd zero matrix) if i≠j and (i,j)∉E
    - B_ij = -P_g*_ij (dxd projection matrix) if i≠j and (i,j)∈E
    - B_ii = sum_{k∈N_i} P_g*_ik (dxd diagonal block)
    
    Args:
        graph: Formation graph topology
        desired_bearings: Dictionary mapping (i,j) to desired bearing vectors
    
    Returns:
        B: d*n x d*n bearing Laplacian matrix
    """
    n = graph.n
    d = len(list(desired_bearings.values())[0])
    
    # Initialize bearing Laplacian
    B = np.zeros((d * n, d * n))
    
    # Compute projection matrices for all edges
    projections = {}
    for (i, j), g in desired_bearings.items():
        projections[(i, j)] = compute_projection_matrix(g)
    
    # Fill diagonal blocks: B_ii = sum_{k∈N_i} P_g*_ik
    for i in range(n):
        block_sum = np.zeros((d, d))
        for j in graph.neighbors[i]:
            if (i, j) in projections:
                block_sum += projections[(i, j)]
        B[d*i:d*(i+1), d*i:d*(i+1)] = block_sum
    
    # Fill off-diagonal blocks: B_ij = -P_g*_ij for (i,j)∈E
    for (i, j), P_g in projections.items():
        B[d*i:d*(i+1), d*j:d*(j+1)] = -P_g
    
    return B


def partition_bearing_laplacian(B: np.ndarray, graph: FormationGraph) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Partition bearing Laplacian into leader-leader, leader-follower, 
    follower-leader, and follower-follower blocks.
    
    B = [B_ll  B_lf]
        [B_fl  B_ff]
    
    Args:
        B: d*n x d*n bearing Laplacian
        graph: Formation graph with leader/follower indices
    
    Returns:
        Tuple of (B_ll, B_lf, B_fl, B_ff)
    """
    n = graph.n
    d = B.shape[0] // n
    
    n_l = graph.n_l
    n_f = graph.n_f
    
    # Extract blocks
    B_ll = B[d*0:d*n_l, d*0:d*n_l]
    B_lf = B[d*0:d*n_l, d*n_l:d*n]
    B_fl = B[d*n_l:d*n, d*0:d*n_l]
    B_ff = B[d*n_l:d*n, d*n_l:d*n]
    
    return B_ll, B_lf, B_fl, B_ff


def compute_target_formation_positions(B_ff: np.ndarray, B_fl: np.ndarray, 
                                       leader_positions: np.ndarray) -> np.ndarray:
    """
    Compute target formation positions for followers.
    
    p_f* = -B_ff^{-1} * B_fl * p_l
    
    This is the unique solution when B_ff is nonsingular (Theorem 1).
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
        B_fl: Follower-leader block of bearing Laplacian
        leader_positions: Flattened leader positions
    
    Returns:
        Target positions for followers
    """
    return -np.linalg.solve(B_ff, B_fl @ leader_positions)


def compute_target_formation_velocities(B_ff: np.ndarray, B_fl: np.ndarray,
                                        leader_velocities: np.ndarray) -> np.ndarray:
    """
    Compute target formation velocities for followers.
    
    v_f* = -B_ff^{-1} * B_fl * v_l
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
        B_fl: Follower-leader block of bearing Laplacian
        leader_velocities: Flattened leader velocities
    
    Returns:
        Target velocities for followers
    """
    return -np.linalg.solve(B_ff, B_fl @ leader_velocities)


def check_target_uniqueness(B_ff: np.ndarray, tolerance: float = 1e-10) -> bool:
    """
    Check if the target formation is unique (Theorem 1).
    
    The target formation is unique iff B_ff is nonsingular.
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
        tolerance: Tolerance for singularity check
    
    Returns:
        True if B_ff is nonsingular (target is unique)
    """
    # Check condition number - large condition number indicates near-singularity
    try:
        cond_num = np.linalg.cond(B_ff)
        return cond_num < 1.0 / tolerance
    except np.linalg.LinAlgError:
        return False


def compute_bff_eigenvalues(B_ff: np.ndarray) -> np.ndarray:
    """
    Compute eigenvalues of B_ff.
    
    According to Lemma 2, B_ff is symmetric positive semidefinite,
    so all eigenvalues should be non-negative.
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
    
    Returns:
        Array of eigenvalues (sorted)
    """
    # B_ff should be symmetric, use eigvalsh for numerical stability
    eigenvalues = np.linalg.eigvalsh(B_ff)
    return np.sort(eigenvalues)


def verify_lemma1(B: np.ndarray, positions: np.ndarray, n: int, d: int) -> bool:
    """
    Verify Lemma 1: Null(B) ⊇ span{1_n ⊗ I_d, p*}
    
    The null space of B should contain:
    1. Translational motions: 1_n ⊗ I_d
    2. Scaling motions: p*
    
    Args:
        B: Bearing Laplacian matrix
        positions: n x d array of formation positions
        n: Number of agents
        d: Dimension
    
    Returns:
        True if lemma is satisfied
    """
    # Flatten positions
    p_flat = positions.flatten()
    
    # Create translation vector: 1_n ⊗ I_d (block form)
    # This represents all agents moving by the same d-dimensional vector
    # For null space check, we just need one representative
    translation = np.tile(np.ones(d), n)
    
    # Check if B * translation ≈ 0
    BT = B @ translation
    
    # Check if B * p ≈ 0
    Bp = B @ p_flat
    
    tol = 1e-8
    return np.linalg.norm(BT) < tol and np.linalg.norm(Bp) < tol


def verify_lemma2_symmetry(B_ff: np.ndarray) -> bool:
    """
    Verify Lemma 2: B_ff is symmetric.
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
    
    Returns:
        True if B_ff is symmetric
    """
    return np.allclose(B_ff, B_ff.T)


def verify_lemma2_psd(B_ff: np.ndarray) -> bool:
    """
    Verify Lemma 2: B_ff is positive semidefinite.
    
    All eigenvalues should be >= 0.
    
    Args:
        B_ff: Follower-follower block of bearing Laplacian
    
    Returns:
        True if B_ff is positive semidefinite
    """
    eigenvalues = np.linalg.eigvalsh(B_ff)
    return np.all(eigenvalues >= -1e-10)
