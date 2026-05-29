"""
reward_functions.py — Modular reward components for Multi-UAV RL training.

All rewards are designed to be combined via CompositeReward.
Weights are loaded from configs/environment_config.yaml (pomdp section).

Research Context (POMDP Reward Proxy)
--------------------------------------
The uav_swarm_controller already computes coverage_percent every 125 steps
as a simulation metric. These reward classes formalize that into a
differentiable, step-level signal for RL training.

Coordinate System: ENU (East=X, North=Y, Up=Z). Arena: ±90 m.

Status: SKELETON — compute() methods return 0.0.
        Implement during RL training phase (Phase 2 of research).
"""

import math
from typing import Dict, List, Tuple


# ─── Type aliases ─────────────────────────────────────────────────────────────
Position3D = Tuple[float, float, float]  # (x, y, z) in ENU metres
UAVPositions = List[Position3D]
CrowdPositions = List[Position3D]


# ─── Base class ───────────────────────────────────────────────────────────────

class BaseReward:
    """Abstract base for all reward components."""

    def compute(
        self,
        uav_positions: UAVPositions,
        crowd_positions: CrowdPositions,
        step: int,
        **kwargs,
    ) -> float:
        """
        Compute scalar reward for the current simulation state.

        Args:
            uav_positions:   List of (x, y, z) for each UAV.
            crowd_positions: List of (x, y, z) for each crowd agent.
            step:            Current simulation step count.
            **kwargs:        Extra state (e.g., previous positions).

        Returns:
            Scalar reward value (float).
        """
        raise NotImplementedError

    def __repr__(self):
        return f"{self.__class__.__name__}()"


# ─── Coverage Reward ──────────────────────────────────────────────────────────

class CoverageReward(BaseReward):
    """
    Rewards UAVs for maximizing area coverage.

    Formula (planned):
        coverage = |unique_cells_covered| / total_cells × 100
        reward = weight × coverage / 100

    The cell-based coverage metric is already implemented in
    uav_swarm_controller.py (compute_coverage). This reward class
    will wrap that metric for use in RL training.

    Parameters:
        cell_size:  Grid cell size in metres (default 5.0, matches config).
        arena_half: Half-width of the arena in metres (default 90.0).
        weight:     Scaling factor for this reward component.
    """

    def __init__(
        self,
        cell_size: float = 5.0,
        arena_half: float = 90.0,
        weight: float = 0.6,
    ):
        self.cell_size = cell_size
        self.arena_half = arena_half
        self.weight = weight
        self._total_cells = (2 * arena_half / cell_size) ** 2

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """
        TODO: Implement using UAV altitude-based footprint.
              Port logic from uav_swarm_controller.compute_coverage().
        """
        # PLACEHOLDER — returns 0.0
        # Implementation:
        #   covered = set()
        #   for (x, y, z) in uav_positions:
        #       view_r = z * 0.7
        #       ... grid cell enumeration ...
        #   return self.weight * len(covered) / self._total_cells
        return 0.0

    def _altitude_footprint(self, altitude: float) -> float:
        """Estimate ground coverage radius from altitude (linear model)."""
        return altitude * 0.7  # matches uav_swarm_controller logic


# ─── Collision Penalty ────────────────────────────────────────────────────────

class CollisionPenalty(BaseReward):
    """
    Penalizes UAVs for flying too close to each other or to obstacles.

    Formula (planned):
        For each UAV pair within safe_distance:
            penalty += -collision_weight × (1 - dist / safe_distance)
        For each UAV within obstacle_margin of a building:
            penalty += -obstacle_weight

    Parameters:
        safe_distance:   Minimum inter-UAV distance in metres (default 5.0).
        collision_weight: Penalty scale for UAV-UAV proximity.
        obstacle_weight:  Penalty scale for UAV-obstacle proximity.
    """

    def __init__(
        self,
        safe_distance: float = 5.0,
        collision_weight: float = 1.0,
        obstacle_weight: float = 0.5,
    ):
        self.safe_distance = safe_distance
        self.collision_weight = collision_weight
        self.obstacle_weight = obstacle_weight

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """
        TODO: Implement using UAV position pairs.
              Building positions available from Webots supervisor via kwargs.
        """
        # PLACEHOLDER — returns 0.0
        # Implementation:
        #   penalty = 0.0
        #   for i, pi in enumerate(uav_positions):
        #       for j, pj in enumerate(uav_positions):
        #           if i >= j: continue
        #           d = _dist3d(pi, pj)
        #           if d < self.safe_distance:
        #               penalty -= self.collision_weight * (1 - d / self.safe_distance)
        #   return penalty
        return 0.0


# ─── Tracking Reward ──────────────────────────────────────────────────────────

class TrackingReward(BaseReward):
    """
    Rewards UAVs for keeping crowd agents within their field of view.

    Formula (planned):
        agents_in_fov = count(crowd agents within any UAV's observation radius)
        reward = weight × agents_in_fov / total_crowd_agents

    This implements the POMDP tracking component alongside CoverageReward.
    Matches the attention_alpha mechanism in uav_swarm_controller._patrol_target().

    Parameters:
        observation_radius: Horizontal radius of each UAV's sensor (metres).
        weight:             Scaling factor.
    """

    def __init__(
        self,
        observation_radius: float = 12.0,
        weight: float = 0.4,
    ):
        self.observation_radius = observation_radius
        self.weight = weight

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """
        TODO: Implement using horizontal distance UAV→crowd agent.
              Altitude-corrected: footprint grows with Z.
        """
        # PLACEHOLDER — returns 0.0
        # Implementation:
        #   if not crowd_positions: return 0.0
        #   tracked = set()
        #   for ci, (cx, cy, cz) in enumerate(crowd_positions):
        #       for (ux, uy, uz) in uav_positions:
        #           horiz_dist = math.hypot(ux - cx, uy - cy)
        #           effective_r = self.observation_radius + uz * 0.3
        #           if horiz_dist <= effective_r:
        #               tracked.add(ci)
        #               break
        #   return self.weight * len(tracked) / len(crowd_positions)
        return 0.0


# ─── Composite Reward ─────────────────────────────────────────────────────────

class CompositeReward(BaseReward):
    """
    Combines multiple reward components into a single scalar.

    Each component is weighted and summed. Components can be enabled/disabled
    individually during training ablation studies.

    Default composition matches configs/environment_config.yaml:
        coverage_weight = 0.6
        tracking_weight = 0.4
        collision_weight = -1.0 (penalty)

    Usage:
        reward_fn = CompositeReward([
            CoverageReward(weight=0.6),
            TrackingReward(weight=0.4),
            CollisionPenalty(collision_weight=1.0),
        ])
        r = reward_fn.compute(uav_positions, crowd_positions, step)
    """

    def __init__(self, components: List[BaseReward] = None):
        if components is None:
            components = [
                CoverageReward(weight=0.6),
                TrackingReward(weight=0.4),
                CollisionPenalty(collision_weight=1.0),
            ]
        self.components = components

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Sum all component rewards."""
        return sum(
            c.compute(uav_positions, crowd_positions, step, **kwargs)
            for c in self.components
        )

    def __repr__(self):
        comp_str = ", ".join(repr(c) for c in self.components)
        return f"CompositeReward([{comp_str}])"


# ─── Utility helpers ──────────────────────────────────────────────────────────

def _dist3d(a: Position3D, b: Position3D) -> float:
    """Euclidean distance between two 3D points."""
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _dist2d(a: Position3D, b: Position3D) -> float:
    """Horizontal (XY-plane) distance, ignores altitude."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
