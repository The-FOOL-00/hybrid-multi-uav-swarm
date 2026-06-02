"""
collision_avoidance
===================
Phase 1 placeholder — future advanced obstacle avoidance methods.

Phase 1 (current): Simple combined-repulsion in SingleDroneController.update()
    - Reads building XY centres from scene
    - Computes weighted sum of repulsion vectors (weight ∝ proximity)
    - Blends 60 % repulsion + 40 % toward target

Phase 2 plan:
    - PotentialFieldAvoidance : artificial potential field (APF)
    - VelocityObstacle        : VO / RVO reactive avoidance
    - DynamicWindowApproach   : DWA for velocity-constrained robots

Usage (planned):
    from collision_avoidance.potential_field import PotentialFieldAvoidance
    avoider = PotentialFieldAvoidance(obstacles, safety_radius=10.0)
    force   = avoider.compute(drone_pos, goal_pos)
"""
# Nothing extra implemented yet — see SingleDroneController._compute_avoidance()
