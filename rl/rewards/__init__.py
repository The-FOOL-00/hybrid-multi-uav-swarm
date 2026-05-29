"""
rl.rewards — Modular reward function library.

Status: SKELETON — functions return 0.0 as placeholders.
        Fill in implementations during RL training phase.
"""
from rl.rewards.reward_functions import (
    CoverageReward,
    CollisionPenalty,
    TrackingReward,
    CompositeReward,
)

__all__ = [
    "CoverageReward",
    "CollisionPenalty",
    "TrackingReward",
    "CompositeReward",
]
