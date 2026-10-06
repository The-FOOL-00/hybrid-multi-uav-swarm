import time
import math
from .planner_base import PlannerBase

class PotentialFieldPlanner(PlannerBase):
    """Continuous-space Artificial Potential Field path planner with A* fallback, inheriting from PlannerBase."""
    
    def __init__(self, world_config, grid_config):
        """
        Initialize Potential Field planner.
        
        Args:
            world_config: Dict containing 'buildings' list
            grid_config: Dict containing 'resolution', 'grid_margin', 
                        'obstacle_inflation', 'hyperparams'
        """
        super().__init__(world_config, grid_config)
        self.world_config = world_config
        self.grid_config = grid_config
        
        # APF specific hyperparameters
        self.attractive_gain = self.hyperparams.get("attractive_gain", 1.0)
        self.repulsive_gain = self.hyperparams.get("repulsive_gain", 15.0)
        self.influence_distance = self.hyperparams.get("influence_distance", 15.0)
        self.goal_threshold = self.hyperparams.get("goal_threshold", 1.0)
        self.max_steps = self.hyperparams.get("max_steps", 1000)
        
        print(f"[PotentialField] Planner initialized | k_att: {self.attractive_gain} | "
              f"k_rep: {self.repulsive_gain} | rho_0: {self.influence_distance}m | "
              f"goal_thresh: {self.goal_threshold}m | max_steps: {self.max_steps} | inflation: {self.inflation}m")

    def get_snapped_start(self, start):
        """Find the nearest collision-free point in continuous space."""
        return self._nearest_free_point(start)

    # -- Segment collision checking -------------------------------------------

    def _is_collision_free(self, p1, p2) -> bool:
        """
        Check if line segment p1->p2 is free of obstacles in continuous space.
        Computes the minimum distance from each building center to the line segment.
        """
        x1, y1 = p1
        x2, y2 = p2
        
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx ** 2 + dy ** 2
        
        for b in self.buildings:
            cx = b.get("x") if "x" in b else b.get("center_x")
            cy = b.get("y") if "y" in b else b.get("center_y")
            r = (b.get("r") if "r" in b else b.get("radius")) + self.inflation
            
            if l2 < 1e-6:
                # Segment is extremely short, check distance from endpoints
                dist = math.hypot(x1 - cx, y1 - cy)
                if dist <= r:
                    return False
                continue
                
            # Projection factor t, clamped to [0, 1] segment bounds
            t = ((cx - x1) * dx + (cy - y1) * dy) / l2
            t = max(0.0, min(1.0, t))
            
            # Closest point on segment
            closest_x = x1 + t * dx
            closest_y = y1 + t * dy
            
            # Distance from closest point to building center
            dist = math.hypot(closest_x - cx, closest_y - cy)
            if dist <= r:
                return False
                
        return True

    def _nearest_free_point(self, point, max_dist=30.0, step=1.0):
        """Find the nearest collision-free point near the given point using a spiral search."""
        if self._is_collision_free(point, point):
            return point
            
        x, y = point
        # Search in concentric rings
        r = step
        while r <= max_dist:
            # Try 16 angles
            for i in range(16):
                theta = i * (2.0 * math.pi / 16.0)
                nx = x + r * math.cos(theta)
                ny = y + r * math.sin(theta)
                if self._is_collision_free((nx, ny), (nx, ny)):
                    return (round(nx, 4), round(ny, 4))
            r += step
            
        return point # fallback if nothing found

    # -- Path Smoothing -------------------------------------------------------

    def _smooth_path_los(self, waypoints):
        """Greedy line-of-sight path smoother using continuous coordinates."""
        if len(waypoints) <= 2:
            return waypoints
            
        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            # Look backwards from the end to find the furthest visible point
            j = len(waypoints) - 1
            while j > i + 1:
                p1 = smoothed[-1]
                p2 = waypoints[j]
                if self._is_collision_free(p1, p2):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j
            
        print(f"[PotentialField] Smoothed: {len(waypoints)} -> {len(smoothed)} waypoints")
        return smoothed

    # -- Base Class Interface Method -----------------------------------------

    def plan(self, start, goal, obstacles=None):
        """
        Compute path from start to goal avoiding obstacles using Artificial Potential Fields.
        
        Args:
            start: Tuple (x, y) in world meters
            goal: Tuple (x, y) in world meters
            obstacles: Unused
            
        Returns:
            List of waypoints [(x, y), ...] in world meters (2D)
        """
        start_t = time.perf_counter()
        
        # Ensure start and goal are tuple coordinates
        start_xy = (round(start[0], 4), round(start[1], 4))
        goal_xy = (round(goal[0], 4), round(goal[1], 4))
        
        # Snap start/goal to nearest free points if inside obstacles
        start_xy = self._nearest_free_point(start_xy)
        goal_xy = self._nearest_free_point(goal_xy)
        
        if start_xy == goal_xy:
            waypoints = [goal]
            self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
            self.path_length_m = self._compute_path_length(waypoints)
            self.smoothness_score = 1.0
            return waypoints
            
        # APF descent loop
        current = start_xy
        path = [current]
        
        k_att = self.attractive_gain
        k_rep = self.repulsive_gain
        rho_0 = self.influence_distance
        goal_threshold = self.goal_threshold
        max_steps = self.max_steps
        step_size = self.resolution  # use resolution as step size
        
        pos_history = []
        history_len = 15
        
        step_count = 0
        fallback = False
        
        while step_count < max_steps:
            dist_to_goal = math.hypot(goal_xy[0] - current[0], goal_xy[1] - current[1])
            if dist_to_goal <= goal_threshold:
                path.append(goal_xy)
                break
                
            # 1. Attractive force direction
            fx_att = k_att * (goal_xy[0] - current[0]) / dist_to_goal
            fy_att = k_att * (goal_xy[1] - current[1]) / dist_to_goal
            
            # 2. Repulsive force direction
            fx_rep = 0.0
            fy_rep = 0.0
            
            for b in self.buildings:
                cx = b.get("x") if "x" in b else b.get("center_x")
                cy = b.get("y") if "y" in b else b.get("center_y")
                r = (b.get("r") if "r" in b else b.get("radius")) + self.inflation
                
                dx = current[0] - cx
                dy = current[1] - cy
                dist_to_center = math.hypot(dx, dy)
                dist_to_surf = dist_to_center - r
                
                if dist_to_surf <= 0.0:
                    # Inside building! Extremely strong force pointing straight out
                    if dist_to_center > 1e-4:
                        fx_rep += 1000.0 * (dx / dist_to_center)
                        fy_rep += 1000.0 * (dy / dist_to_center)
                    else:
                        fx_rep += 1000.0
                elif dist_to_surf < rho_0:
                    # Repulsive force = k_rep * (1/d - 1/rho_0) * (1/d^2) * grad_d
                    factor = k_rep * (1.0 / dist_to_surf - 1.0 / rho_0) * (1.0 / (dist_to_surf ** 2))
                    if dist_to_center > 1e-4:
                        fx_rep += factor * (dx / dist_to_center)
                        fy_rep += factor * (dy / dist_to_center)
                        
            # 3. Total force
            fx = fx_att + fx_rep
            fy = fy_att + fy_rep
            f_mag = math.hypot(fx, fy)
            
            if f_mag < 1e-4:
                fallback = True
                print(f"[PotentialField] Local minimum: zero force at {current}")
                break
                
            # Next position step
            next_x = current[0] + (fx / f_mag) * step_size
            next_y = current[1] + (fy / f_mag) * step_size
            next_pos = (round(next_x, 4), round(next_y, 4))
            
            # Check for segment collision before committing to the step
            if not self._is_collision_free(current, next_pos):
                fallback = True
                print(f"[PotentialField] Segment collision detected from {current} to {next_pos}")
                break
                
            current = next_pos
            path.append(current)
            self.nodes_explored += 1
            
            # Oscillating/stuck detection (history check)
            pos_history.append(current)
            if len(pos_history) > history_len:
                pos_history.pop(0)
                xs = [p[0] for p in pos_history]
                ys = [p[1] for p in pos_history]
                if (max(xs) - min(xs)) < 0.05 and (max(ys) - min(ys)) < 0.05:
                    fallback = True
                    print(f"[PotentialField] Local minimum: oscillation/stasis detected at {current}")
                    break
                    
            step_count += 1
            
        if step_count >= max_steps:
            fallback = True
            print(f"[PotentialField] Max steps ({max_steps}) exceeded.")
            
        if fallback:
            # Fall back to A*
            print("[PotentialField] Falling back to A* global path planner")
            from .astar_planner import AStarPlanner
            fallback_planner = AStarPlanner(self.world_config, self.grid_config)
            return fallback_planner.plan(start, goal)
            
        # Path found successfully
        world_path = path[1:]
        world_path = self._smooth_path_los(world_path)
        
        self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
        complete_path = [start] + world_path
        self.path_length_m = self._compute_path_length(complete_path)
        self.smoothness_score = self._compute_smoothness(complete_path)
        
        print(f"[PotentialField] Path found: {len(world_path)} waypoints | "
              f"Time: {self.compute_time_ms:.2f}ms | Steps: {self.nodes_explored} | "
              f"Len: {self.path_length_m:.1f}m | Smooth: {self.smoothness_score}")
              
        return world_path
