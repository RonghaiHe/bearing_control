"""
Bearing-Based Formation Control Framework

Implementation of core classes and functions for bearing-based formation maneuver control
as described in Zhao and Zelazo (2017).

References:
- Section II: Problem formulation
- Bearing definition: g_ij = (p_j - p_i) / ||p_j - p_i||
- Projection matrix: P_g = I_d - g * g^T
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class AgentState:
    """State of an agent (position and velocity)"""
    position: np.ndarray
    velocity: np.ndarray
    
    @property
    def dim(self) -> int:
        """Dimension of the space"""
        return len(self.position)


class FormationGraph:
    """
    Graph topology for multi-agent formation.
    
    The graph G = (V, E) describes the information flow among agents.
    - V: set of agent indices
    - E: set of directed edges (i, j) meaning agent i can access agent j's info
    """
    
    def __init__(self, n_agents: int, edges: List[Tuple[int, int]], 
                 leader_indices: List[int]):
        """
        Args:
            n_agents: Total number of agents
            edges: List of directed edges (i, j)
            leader_indices: Indices of leader agents (first n_l agents)
        """
        self.n = n_agents
        self.edges = edges
        self.leader_indices = leader_indices
        self.follower_indices = [i for i in range(n_agents) if i not in leader_indices]
        
        self.n_l = len(leader_indices)
        self.n_f = len(self.follower_indices)
        
        # Build neighbor sets
        self.neighbors = {i: set() for i in range(n_agents)}
        for i, j in edges:
            self.neighbors[i].add(j)
    
    def is_bidirectional(self) -> bool:
        """Check if all edges among followers are bidirectional"""
        for i in self.follower_indices:
            for j in self.follower_indices:
                if i != j:
                    if (i, j) in self.edges and (j, i) not in self.edges:
                        return False
        return True


def compute_bearing(p_i: np.ndarray, p_j: np.ndarray) -> np.ndarray:
    """
    Compute bearing of agent j relative to agent i.
    
    g_ij = (p_j - p_i) / ||p_j - p_i||
    
    Args:
        p_i: Position of agent i
        p_j: Position of agent j
    
    Returns:
        Unit vector representing the bearing
    """
    diff = p_j - p_i
    norm = np.linalg.norm(diff)
    if norm < 1e-10:
        raise ValueError(f"Bearing undefined for coincident agents at {p_i} and {p_j}")
    return diff / norm


def compute_projection_matrix(g: np.ndarray) -> np.ndarray:
    """
    Compute the orthogonal projection matrix for a bearing vector.
    
    P_g = I_d - g * g^T
    
    This projects any vector onto the orthogonal complement of span{g}.
    
    Args:
        g: Bearing unit vector (d-dimensional)
    
    Returns:
        d x d orthogonal projection matrix
    """
    d = len(g)
    return np.eye(d) - np.outer(g, g)


def compute_all_bearings(positions: np.ndarray, edges: List[Tuple[int, int]]) -> dict:
    """
    Compute bearings for all edges in the graph.
    
    Args:
        positions: n x d array of agent positions
        edges: List of directed edges (i, j)
    
    Returns:
        Dictionary mapping (i, j) to bearing vector g_ij
    """
    bearings = {}
    for i, j in edges:
        bearings[(i, j)] = compute_bearing(positions[i], positions[j])
    return bearings


def compute_all_projection_matrices(bearings: dict) -> dict:
    """
    Compute projection matrices for all bearings.
    
    Args:
        bearings: Dictionary mapping (i, j) to bearing vector
    
    Returns:
        Dictionary mapping (i, j) to projection matrix P_g
    """
    projections = {}
    for (i, j), g in bearings.items():
        projections[(i, j)] = compute_projection_matrix(g)
    return projections


class FormationState:
    """
    Complete state of the formation at a given time.
    """
    
    def __init__(self, graph: FormationGraph, positions: np.ndarray, 
                 velocities: np.ndarray):
        """
        Args:
            graph: Formation graph topology
            positions: n x d array of agent positions
            velocities: n x d array of agent velocities
        """
        self.graph = graph
        self.positions = positions
        self.velocities = velocities
        self.d = positions.shape[1]
        
        # Compute bearings and projections
        self.bearings = compute_all_bearings(positions, graph.edges)
        self.projections = compute_all_projection_matrices(self.bearings)
    
    def get_leader_positions(self) -> np.ndarray:
        """Get positions of leader agents"""
        return self.positions[self.graph.leader_indices]
    
    def get_follower_positions(self) -> np.ndarray:
        """Get positions of follower agents"""
        return self.positions[self.graph.follower_indices]
    
    def get_leader_velocities(self) -> np.ndarray:
        """Get velocities of leader agents"""
        return self.velocities[self.graph.leader_indices]
    
    def get_follower_velocities(self) -> np.ndarray:
        """Get velocities of follower agents"""
        return self.velocities[self.graph.follower_indices]
    
    def compute_centroid(self) -> np.ndarray:
        """Compute formation centroid"""
        return np.mean(self.positions, axis=0)
    
    def compute_scale(self) -> float:
        """
        Compute formation scale.
        
        s = sqrt(1/n * sum_i ||p_i - c||^2)
        """
        c = self.compute_centroid()
        diffs = self.positions - c
        return np.sqrt(np.mean(np.sum(diffs**2, axis=1)))
    
    def flatten_positions(self) -> np.ndarray:
        """Flatten positions to a vector [p_1^T, ..., p_n^T]^T"""
        return self.positions.flatten()
    
    def flatten_velocities(self) -> np.ndarray:
        """Flatten velocities to a vector [v_1^T, ..., v_n^T]^T"""
        return self.velocities.flatten()


def create_target_formation(graph: FormationGraph, 
                           desired_bearings: dict,
                           leader_positions: np.ndarray,
                           leader_velocities: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute target formation positions and velocities.
    
    The target formation is uniquely determined by bearings and leader positions
    when B_ff is nonsingular (Theorem 1).
    
    p_f* = -B_ff^{-1} * B_fl * p_l
    v_f* = -B_ff^{-1} * B_fl * v_l
    
    Args:
        graph: Formation graph
        desired_bearings: Desired bearing constraints
        leader_positions: Positions of leaders
        leader_velocities: Velocities of leaders
    
    Returns:
        Target positions and velocities for all agents
    """
    # This will be implemented in bearing_laplacian.py (S02)
    # For now, return placeholder
    n = graph.n
    d = leader_positions.shape[1]
    target_positions = np.zeros((n, d))
    target_velocities = np.zeros((n, d))
    
    # Set leader positions/velocities
    for idx, i in enumerate(graph.leader_indices):
        target_positions[i] = leader_positions[idx]
        target_velocities[i] = leader_velocities[idx]
    
    return target_positions, target_velocities
