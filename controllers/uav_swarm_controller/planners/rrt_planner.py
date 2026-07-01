import time
import math
import random
from .planner_base import PlannerBase

class RRTPlanner(PlannerBase):
    """Continuous-space Bidirectional RRT path planner inheriting from PlannerBase."""
    
    def __init__(self, world_config, grid_config):
        """
        Initialize RRT planner.
        
        Args:
            world_config: Dict containing 'buildings' list
            grid_config: Dict containing 'resolution', 'grid_margin', 
                        'obstacle_inflation', 'hyperparams'
        """
        super().__init__(world_config, grid_config)
        
        # RRT specific hyperparameters
        self.max_iterations = self.hyperparams.get("max_iterations", 5000)
        self.step_size = self.hyperparams.get("step_size", 2.0)
        self.bidirectional = self.hyperparams.get("bidirectional", True)
        
        # Seed RNG: random.seed(42) for determinism
        random.seed(42)
        
        print(f"[RRT] Planner initialized | max_iter: {self.max_iterations} | "
              f"step_size: {self.step_size}m | bidirectional: {self.bidirectional} | inflation: {self.inflation}m")

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

    # -- Helper methods -------------------------------------------------------

    def _nearest_node(self, tree, point):
        """Find the node in the tree closest to the point."""
        min_dist = float("inf")
        nearest = None
        for node in tree:
            dist = math.hypot(node[0] - point[0], node[1] - point[1])
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest, min_dist

    def _steer(self, from_node, toward_point, step_size):
        """Steer from from_node toward toward_point by at most step_size."""
        fx, fy = from_node
        tx, ty = toward_point
        dx = tx - fx
        dy = ty - fy
        dist = math.hypot(dx, dy)
        if dist <= step_size:
            return toward_point
        # Normalize and scale
        scale = step_size / dist
        nx = fx + dx * scale
        ny = fy + dy * scale
        return (nx, ny)

    def _reconstruct_path(self, tree_start, tree_goal, conn_start, conn_goal):
        """
        Reconstruct the full path from start to goal.
        Path flows start -> ... -> conn_start -> conn_goal -> ... -> goal
        """
        # Trace start tree backwards from conn_start to start
        path_start = []
        curr = conn_start
        while curr is not None:
            path_start.append(curr)
            curr = tree_start[curr]
        path_start.reverse()  # now start -> ... -> conn_start
        
        # Trace goal tree backwards from conn_goal to goal
        path_goal = []
        curr = conn_goal
        while curr is not None:
            path_goal.append(curr)
            curr = tree_goal[curr]
        # path_goal is conn_goal -> ... -> goal
        
        return path_start + path_goal

    # -- Path Smoothing -------------------------------------------------------

    def _smooth_path_los(self, waypoints):
        """Greedy line-of-sight path smoother using continuous coordinates."""
        if len(waypoints) <= 2:
            return waypoints
            
        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            j = len(waypoints) - 1
            while j > i + 1:
                p1 = smoothed[-1]
                p2 = waypoints[j]
                if self._is_collision_free(p1, p2):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j
            
        print(f"[RRT] Smoothed: {len(waypoints)} -> {len(smoothed)} waypoints")
        return smoothed

    # -- Core Bidirectional RRT search ----------------------------------------

    def _rrt_bidirectional_search(self, start, goal, max_iterations):
        """
        Core Bidirectional RRT algorithm.
        Returns: list of (x, y) coordinates from start to goal.
        """
        # Seed RNG locally on each search for determinism and metrics validation
        random.seed(42)
        
        # Start and Goal trees: node -> parent
        tree_start = {start: None}
        tree_goal = {goal: None}
        
        # Keep track of active and inactive tree references
        # active_tree expands, conn_tree is what we try to connect to
        # active_is_start keeps track of which tree is which
        active_tree = tree_start
        conn_tree = tree_goal
        active_is_start = True
        
        nodes_count = 2
        
        for iteration in range(max_iterations):
            # 1. Sample a random point (with 10% tree-bias toward the other tree)
            if random.random() < 0.1:
                rand_pt = random.choice(list(conn_tree.keys()))
            else:
                rx = random.uniform(-self.grid_margin, self.grid_margin)
                ry = random.uniform(-self.grid_margin, self.grid_margin)
                rand_pt = (rx, ry)
                
            # 2. Extend the active tree towards rand_pt
            nearest_active, _ = self._nearest_node(active_tree, rand_pt)
            new_node = self._steer(nearest_active, rand_pt, self.step_size)
            
            # Check segment collision
            if not self._is_collision_free(nearest_active, new_node):
                continue
                
            new_node = (round(new_node[0], 4), round(new_node[1], 4))
            
            # Avoid loops / duplicate insertion
            if new_node in active_tree:
                continue
                
            active_tree[new_node] = nearest_active
            nodes_count += 1
            
            # 3. Try to connect new_node to the other tree (conn_tree)
            nearest_conn, _ = self._nearest_node(conn_tree, new_node)
            conn_node = self._steer(nearest_conn, new_node, self.step_size)
            
            if self._is_collision_free(nearest_conn, conn_node):
                conn_node = (round(conn_node[0], 4), round(conn_node[1], 4))
                if conn_node not in conn_tree:
                    conn_tree[conn_node] = nearest_conn
                    nodes_count += 1
                
                # Check if we can connect the active tree and connection tree directly
                if math.hypot(new_node[0] - conn_node[0], new_node[1] - conn_node[1]) <= self.step_size:
                    if self._is_collision_free(new_node, conn_node):
                        # Connected! Reconstruct path
                        self.nodes_explored += nodes_count
                        if active_is_start:
                            return self._reconstruct_path(active_tree, conn_tree, new_node, conn_node)
                        else:
                            return self._reconstruct_path(conn_tree, active_tree, conn_node, new_node)
                            
            # 4. Swap trees for symmetric bidirectional growth
            active_tree, conn_tree = conn_tree, active_tree
            active_is_start = not active_is_start
            
        self.nodes_explored += nodes_count
        return []

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

    # -- Base Class Interface Method -----------------------------------------

    def plan(self, start, goal, obstacles=None):
        """
        Compute path from start to goal avoiding obstacles.
        
        Args:
            start: Tuple (x, y) in world meters
            goal: Tuple (x, y) in world meters
            obstacles: Unused
            
        Returns:
            List of waypoints [(x, y), ...] in world meters (2D)
        """
        self.nodes_explored = 0
        start_t = time.perf_counter()
        
        # Ensure start and goal are tuple coordinates for hashability
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
            
        # Core Bidirectional RRT search
        raw_path = self._rrt_bidirectional_search(start_xy, goal_xy, self.max_iterations)
        
        if not raw_path:
            print("[RRT] WARNING: No path found - falling back to straight line")
            self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
            self.path_length_m = 0.0
            self.smoothness_score = 1.0
            return []
            
        # Remove start position from waypoints as expected by controller
        world_path = raw_path[1:]
        
        # Smooth path with Line-Of-Sight
        world_path = self._smooth_path_los(world_path)
        
        # Record stats
        self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
        
        # Include start position in the path sequence to compute stats correctly
        complete_path = [start] + world_path
        self.path_length_m = self._compute_path_length(complete_path)
        self.smoothness_score = self._compute_smoothness(complete_path)
        
        print(f"[RRT] Path found: {len(world_path)} waypoints | "
              f"Time: {self.compute_time_ms:.2f}ms | Nodes: {self.nodes_explored} | "
              f"Len: {self.path_length_m:.1f}m | Smooth: {self.smoothness_score}")
              
        return world_path
