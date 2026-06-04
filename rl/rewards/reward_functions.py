"""
reward_functions.py — Modular reward components for Multi-UAV RL training.

All rewards are designed to be combined via CompositeReward.
Weights are loaded from configs/environment_config.yaml (rl.reward_weights section).

Research Context (POMDP Reward Proxy)
--------------------------------------
This file formalizes continuous, smooth reward signals for RL training.
It includes:
1. CoverageReward  — Altitude-based instantaneous footprint coverage
2. CollisionPenalty — Inter-UAV and building obstacle proximity penalties
3. OverlapPenalty  — Footprint intersection shaping penalty
4. SeparationReward — Bonus for maintaining spacing in an optimal band
5. TrackingReward  — FOV-based Target Recognition Rate tracking reward

Phase 1 Bug Fixes Applied
--------------------------
- CoverageReward and TrackingReward previously returned 0.0 because
  `coverage` and `trr` kwargs were never passed by uav_swarm_env.step().
  Fixed in uav_swarm_env.py; these components now receive their values.
- CollisionPenalty building-proximity arm now also accepts raw
  `building_positions` + `uav_positions` as fallback when `min_obs_dists`
  is not pre-computed by the caller.
- Added compute_with_breakdown() for per-component reward diagnostics.
- RewardDebug console logging is rate-limited (every 125 steps + episode end).
"""

import os
import math
import yaml
from typing import Dict, List, Tuple, Optional

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
            **kwargs:        Extra state (e.g., coverage, trr, min_obs_dists).

        Returns:
            Scalar reward value (float).
        """
        raise NotImplementedError

    def __repr__(self):
        return f"{self.__class__.__name__}()"


# ─── Coverage Reward ──────────────────────────────────────────────────────────

class CoverageReward(BaseReward):
    """
    Rewards UAVs for maximizing area coverage based on altitude footprints.

    Formula:
        coverage = |unique_cells_covered| / total_cells   (pre-computed by env)
        reward = weight * coverage

    IMPORTANT: The caller (uav_swarm_env.step) must pass:
        coverage=<float in [0.0, 1.0]>
    as a keyword argument. If absent the reward defaults to 0.0.

    Parameters:
        weight: Scaling factor for this reward component (default 0.6).
    """

    def __init__(self, weight: float = 0.6):
        self.weight = weight

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Calculate the instantaneous footprint coverage reward."""
        coverage = kwargs.get("coverage", 0.0)
        return self.weight * coverage


# ─── Collision Penalty ────────────────────────────────────────────────────────

class CollisionPenalty(BaseReward):
    """
    Penalizes UAVs for flying too close to each other or to obstacles.

    Formulas:
        For each UAV pair within safe_distance:
            penalty -= collision_weight * (1 - dist / safe_distance)
        For each UAV within obstacle_margin of a building center:
            penalty -= obstacle_weight * (1 - dist / obstacle_margin)

    The caller may pass either:
        min_obs_dists: List[float] — pre-computed min distance per UAV to any building
    OR:
        building_positions: List[Position3D] — raw building coords; distances
                            are then computed here using uav_positions.

    Parameters:
        safe_distance:    Minimum inter-UAV distance in metres (default 5.0).
        collision_weight: Penalty scale for UAV-UAV proximity.
        obstacle_margin:  Minimum safety clearance to buildings in metres (default 10.0).
        obstacle_weight:  Penalty scale for UAV-obstacle proximity.
    """

    def __init__(
        self,
        safe_distance: float = 5.0,
        collision_weight: float = 1.0,
        obstacle_margin: float = 10.0,
        obstacle_weight: float = 0.5,
    ):
        self.safe_distance = safe_distance
        self.collision_weight = collision_weight
        self.obstacle_margin = obstacle_margin
        self.obstacle_weight = obstacle_weight

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Compute proximity penalties."""
        penalty = 0.0
        num_uavs = len(uav_positions)

        # 1. UAV-UAV Proximity
        for i in range(num_uavs):
            for j in range(i + 1, num_uavs):
                d = _dist3d(uav_positions[i], uav_positions[j])
                if d < self.safe_distance:
                    penalty -= self.collision_weight * (1.0 - d / self.safe_distance)

        # 2. UAV-Obstacle Proximity
        # Accept pre-computed min_obs_dists if provided; otherwise derive from
        # raw building_positions to fix the missing-kwarg bug.
        min_obs_dists = kwargs.get("min_obs_dists", None)

        if min_obs_dists is None:
            # Fallback: compute from raw building_positions
            building_positions = kwargs.get("building_positions", [])
            if building_positions and uav_positions:
                min_obs_dists = []
                for u_pos in uav_positions:
                    min_d = float("inf")
                    for b_pos in building_positions:
                        d = math.hypot(u_pos[0] - b_pos[0], u_pos[1] - b_pos[1])
                        if d < min_d:
                            min_d = d
                    min_obs_dists.append(min_d)
            else:
                min_obs_dists = []

        for min_obs_dist in min_obs_dists:
            if min_obs_dist < self.obstacle_margin:
                penalty -= self.obstacle_weight * (1.0 - min_obs_dist / self.obstacle_margin)

        return penalty


# ─── Overlap Penalty ──────────────────────────────────────────────────────────

class OverlapPenalty(BaseReward):
    """
    Penalizes drones whose footprints overlap excessively beyond clearance threshold eta.

    Formula:
        For each pair where d_xy < eta * (r_i + r_j):
            penalty -= weight * (1 - d_xy / (eta * (r_i + r_j)))

    Parameters:
        weight:    Scaling factor.
        threshold: eta parameter (default 0.8, permits 20% overlap zone without penalty).
    """

    def __init__(self, weight: float = 0.3, threshold: float = 0.8):
        self.weight = weight
        self.threshold = threshold

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Compute footprint overlap penalty."""
        penalty = 0.0
        num_uavs = len(uav_positions)

        for i in range(num_uavs):
            r_i = uav_positions[i][2] * 0.7  # active camera radius based on Z
            for j in range(i + 1, num_uavs):
                r_j = uav_positions[j][2] * 0.7
                d_xy = _dist2d(uav_positions[i], uav_positions[j])
                overlap_limit = self.threshold * (r_i + r_j)
                if d_xy < overlap_limit:
                    penalty -= self.weight * (1.0 - d_xy / overlap_limit)

        return penalty


# ─── Separation Reward ────────────────────────────────────────────────────────

class SeparationReward(BaseReward):
    """
    Positive shaping reward for maintaining a healthy spatial spread.

    Formula:
        For each UAV, if nearest neighbor distance is in [min_dist, max_dist] range:
            reward += weight

    Parameters:
        weight:   Scaling factor (default 0.05).
        min_dist: Lower bound of separation window (default 10.0m).
        max_dist: Upper bound of separation window (default 25.0m).
    """

    def __init__(
        self,
        weight: float = 0.05,
        min_dist: float = 10.0,
        max_dist: float = 25.0,
    ):
        self.weight = weight
        self.min_dist = min_dist
        self.max_dist = max_dist

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Compute separation reward for healthy spacing."""
        reward = 0.0
        num_uavs = len(uav_positions)

        for i in range(num_uavs):
            min_nn_dist = float("inf")
            for j in range(num_uavs):
                if i == j:
                    continue
                d = _dist2d(uav_positions[i], uav_positions[j])
                if d < min_nn_dist:
                    min_nn_dist = d
            if self.min_dist <= min_nn_dist <= self.max_dist:
                reward += self.weight

        return reward


# ─── Tracking Reward ──────────────────────────────────────────────────────────

class TrackingReward(BaseReward):
    """
    Rewards UAVs for keeping crowd agents within their field of view.

    Formula:
        TRR = (number of unique tracked crowd agents) / (total crowd agents)
        reward = weight * TRR

    IMPORTANT: The caller (uav_swarm_env.step) must pass:
        trr=<float in [0.0, 1.0]>
    as a keyword argument. If absent the reward defaults to 0.0.

    Parameters:
        weight: Scaling factor (default 0.4).
    """

    def __init__(self, weight: float = 0.4):
        self.weight = weight

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Compute Target Recognition Rate (TRR) based tracking reward."""
        trr = kwargs.get("trr", 0.0)
        return self.weight * trr


# ─── Composite Reward ─────────────────────────────────────────────────────────

class CompositeReward(BaseReward):
    """
    Combines all modular reward components, parsing weights dynamically from config.

    Default compositions:
        Coverage:    0.6
        Tracking:    0.4
        Collision:   1.0
        Overlap:     0.3
        Separation:  0.05

    Use compute_with_breakdown() to get per-component values for diagnostics.
    RewardDebug console output is rate-limited to every 125 steps + episode end.
    """

    # Class-level logging state (shared across all instances in a process)
    _last_debug_step: int = -1

    def __init__(self, components: List[BaseReward] = None):
        if components is not None:
            self.components = components
            return

        # Attempt to load weights from config
        self.components = []
        coverage_w = 0.6
        tracking_w = 0.4
        collision_w = 1.0
        overlap_w = 0.3
        separation_w = 0.05

        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        config_path = os.path.join(project_root, "configs", "environment_config.yaml")
        if os.path.isfile(config_path):
            try:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                rl_config = config.get("rl", {})
                weights = rl_config.get("reward_weights", {})
                coverage_w   = weights.get("coverage",         coverage_w)
                collision_w  = weights.get("collision_penalty", collision_w)
                # Convert negative config penalty weight to positive scaling parameter
                collision_w  = abs(collision_w)
                tracking_w   = weights.get("tracking",         tracking_w)
                overlap_w    = weights.get("overlap_penalty",  overlap_w)
                separation_w = weights.get("separation_reward", separation_w)
            except Exception as e:
                print(f"[WARN] Failed to load reward weights from config: {e}. Using defaults.")

        # Bind active components
        self.components.append(CoverageReward(weight=coverage_w))
        self.components.append(TrackingReward(weight=tracking_w))
        self.components.append(CollisionPenalty(collision_weight=collision_w))
        self.components.append(OverlapPenalty(weight=overlap_w))
        self.components.append(SeparationReward(weight=separation_w))

    def compute(self, uav_positions, crowd_positions, step, **kwargs) -> float:
        """Sum all modular reward components (no breakdown logging)."""
        total = 0.0
        for c in self.components:
            total += c.compute(uav_positions, crowd_positions, step, **kwargs)
        return total

    def compute_with_breakdown(
        self,
        uav_positions: UAVPositions,
        crowd_positions: CrowdPositions,
        step: int,
        log_debug: bool = False,
        episode_end: bool = False,
        **kwargs,
    ) -> Tuple[float, Dict[str, float]]:
        """
        Compute total reward and return a per-component breakdown dict.

        Args:
            uav_positions:  List of (x, y, z) per UAV.
            crowd_positions: List of (x, y, z) per crowd agent.
            step:           Current episode step (used for rate-limited logging).
            log_debug:      If True, print [RewardDebug] line when rate-limit allows.
            episode_end:    If True, always print breakdown summary regardless of step.
            **kwargs:       Forwarded to each component (coverage, trr, min_obs_dists, etc.).

        Returns:
            (total_reward: float, breakdown: Dict[str, float])
        """
        breakdown: Dict[str, float] = {}
        total = 0.0

        for c in self.components:
            val = c.compute(uav_positions, crowd_positions, step, **kwargs)
            # Use short class name as key, e.g. "CoverageReward" → "coverage"
            key = type(c).__name__.replace("Reward", "").replace("Penalty", "_penalty").lower()
            breakdown[key] = round(val, 5)
            total += val

        breakdown["total"] = round(total, 5)

        # ── Rate-limited console logging ──────────────────────────────────────
        should_log = False
        if episode_end:
            should_log = True
        elif log_debug and (step % 125 == 0) and (step != CompositeReward._last_debug_step):
            should_log = True
            CompositeReward._last_debug_step = step

        if should_log:
            tag = "[EpisodeEnd] " if episode_end else f"[Step {step:>5}] "
            cov_r  = breakdown.get("coverage",    0.0)
            trk_r  = breakdown.get("tracking",    0.0)
            col_p  = breakdown.get("collision_penalty", 0.0)
            ovl_p  = breakdown.get("overlap_penalty",   0.0)  # may not exist
            sep_r  = breakdown.get("separation",   0.0)
            total_r = breakdown["total"]
            print(
                f"[RewardDebug] {tag}"
                f"coverage={cov_r:+.4f}  "
                f"tracking={trk_r:+.4f}  "
                f"collision={col_p:+.4f}  "
                f"overlap={ovl_p:+.4f}  "
                f"separation={sep_r:+.4f}  "
                f"| TOTAL={total_r:+.4f}"
            )

        return total, breakdown

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
