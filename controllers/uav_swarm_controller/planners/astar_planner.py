import time
import math
import heapq
from .planner_base import PlannerBase

class AStarPlanner(PlannerBase):
    """Grid-based A* path planner inheriting from PlannerBase."""
    
    def __init__(self, world_config, grid_config):
        """
        Initialize A* planner.
        
        Args:
            world_config: Dict containing 'buildings' list
            grid_config: Dict containing 'resolution', 'grid_margin', 
                        'obstacle_inflation', 'hyperparams'
        """
        super().__init__(world_config, grid_config)
        
        # Grid dimensions
        span = 2.0 * self.grid_margin
        self.cols = int(math.ceil(span / self.resolution)) + 1
        self.rows = self.cols  # Square world
        
        # Determine heuristic type from hyperparams
        self.heuristic_type = self.hyperparams.get("heuristic", "euclidean")
        
        # Build the occupancy grid (True = blocked)
        self.grid = self._build_grid()
        
        total_cells = self.cols * self.rows
        blocked = sum(1 for r in range(self.rows)
                      for c in range(self.cols) if self.grid[r][c])
        print(f"[A*] Grid built: {self.cols}x{self.rows} cells "
              f"({self.resolution}m/cell) | {blocked}/{total_cells} blocked "
              f"| world +/-{self.grid_margin}m")
              
    def _build_grid(self) -> list:
        """Return a 2-D list[row][col] of booleans (True = obstacle)."""
        grid = [[False] * self.cols for _ in range(self.rows)]
        inflated_buildings = [
            {
                "x": b.get("x") if "x" in b else b.get("center_x"),
                "y": b.get("y") if "y" in b else b.get("center_y"),
                "r": (b.get("r") if "r" in b else b.get("radius")) + self.inflation
            }
            for b in self.buildings
        ]
        for row in range(self.rows):
            wy = row * self.resolution - self.grid_margin
            for col in range(self.cols):
                wx = col * self.resolution - self.grid_margin
                for b in inflated_buildings:
                    dist = math.hypot(wx - b["x"], wy - b["y"])
                    if dist <= b["r"]:
                        grid[row][col] = True
                        break  # no need to check further buildings
        return grid

    # -- Coordinate conversion helpers ---------------------------------------

    def _world_to_grid(self, world_pos):
        """Convert (x, y) world meters -> (col, row) grid cells."""
        x, y = world_pos
        col = int((x + self.grid_margin) / self.resolution)
        row = int((y + self.grid_margin) / self.resolution)
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return col, row

    def _grid_to_world(self, grid_cell):
        """Convert (col, row) grid cells -> (x, y) world meters."""
        col, row = grid_cell
        x = col * self.resolution - self.grid_margin
        y = row * self.resolution - self.grid_margin
        return x, y

    def _is_free(self, grid_cell):
        """Returns True if cell is within bounds and not blocked by an obstacle."""
        col, row = grid_cell
        return 0 <= col < self.cols and 0 <= row < self.rows and not self.grid[row][col]

    # -- Heuristic -----------------------------------------------------------

    def _heuristic(self, pos, goal):
        """Compute h(n) heuristic (in cell units)."""
        col, row = pos
        gc, gr = goal
        if self.heuristic_type == "manhattan":
            return abs(col - gc) + abs(row - gr)
        else:  # default: euclidean
            return math.hypot(col - gc, row - gr)

    # -- Nearest free cell BFS fallback ---------------------------------------

    def _nearest_free(self, col: int, row: int, max_search: int = 10):
        """BFS outward to find the closest free cell from a blocked one."""
        if self._is_free((col, row)):
            return col, row
        visited = set()
        queue = [(col, row)]
        visited.add((col, row))
        while queue:
            next_queue = []
            for c, r in queue:
                for dc in range(-1, 2):
                    for dr in range(-1, 2):
                        nc, nr = c + dc, r + dr
                        if (nc, nr) in visited:
                            continue
                        visited.add((nc, nr))
                        if self._is_free((nc, nr)):
                            return nc, nr
                        next_queue.append((nc, nr))
            queue = next_queue
            if len(visited) > (2 * max_search + 1) ** 2:
                break
        return col, row  # give up - return original

    # -- Segment collision checking -------------------------------------------

    def _segment_collision_free(self, p1, p2) -> bool:
        """
        Check if the straight segment p1->p2 is free of obstacles.
        Uses cell-based point-sampling along the segment.
        """
        ax, ay = p1
        bx, by = p2
        samples = 20
        for i in range(samples + 1):
            t = i / samples
            wx = ax + t * (bx - ax)
            wy = ay + t * (by - ay)
            col, row = self._world_to_grid((wx, wy))
            if not self._is_free((col, row)):
                return False
        return True

    # -- Core A* algorithm ----------------------------------------------------

    def _astar_search(self, start, goal):
        """
        Core A* algorithm running on 8-connected grid.
        Returns: list of (col, row) cells from start to goal.
        """
        sc, sr = start
        gc, gr = goal
        
        # Priority queue: (f_score, g_score, col, row)
        open_heap = []
        heapq.heappush(open_heap, (0.0, 0.0, sc, sr))
        
        came_from = {(sc, sr): None}
        g_score = {(sc, sr): 0.0}
        
        # 8-connected neighbours (dc, dr, move_cost)
        NEIGHBOURS = [
            (-1, -1, 1.4142), (-1, 0, 1.0), (-1, 1, 1.4142),
            ( 0, -1, 1.0),                   ( 0, 1, 1.0),
            ( 1, -1, 1.4142), ( 1, 0, 1.0), ( 1, 1, 1.4142),
        ]
        
        closed = set()
        explored_count = 0
        
        while open_heap:
            _, g, col, row = heapq.heappop(open_heap)
            node = (col, row)
            
            if node in closed:
                continue
            closed.add(node)
            explored_count += 1
            
            if node == (gc, gr):
                # Reconstruct path
                path = []
                current = (gc, gr)
                while current is not None:
                    path.append(current)
                    current = came_from.get(current)
                path.reverse()
                self.nodes_explored += explored_count
                return path
                
            for dc, dr, move_cost in NEIGHBOURS:
                nc, nr = col + dc, row + dr
                if not self._is_free((nc, nr)):
                    continue
                neighbour = (nc, nr)
                new_g = g + move_cost
                if new_g < g_score.get(neighbour, float("inf")):
                    g_score[neighbour] = new_g
                    f = new_g + self._heuristic((nc, nr), (gc, gr))
                    heapq.heappush(open_heap, (f, new_g, nc, nr))
                    came_from[neighbour] = node
                    
        self.nodes_explored += explored_count
        return []

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
                if self._segment_collision_free(p1, p2):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j
            
        print(f"[A*] Smoothed: {len(waypoints)} -> {len(smoothed)} waypoints")
        return smoothed

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
        
        # Convert coordinates to grid
        sc, sr = self._world_to_grid(start)
        gc, gr = self._world_to_grid(goal)
        
        # Snap start/goal to nearest free grid cell if inside obstacles
        sc, sr = self._nearest_free(sc, sr)
        gc, gr = self._nearest_free(gc, gr)
        
        if (sc, sr) == (gc, gr):
            waypoints = [goal]
            self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
            self.path_length_m = self._compute_path_length(waypoints)
            self.smoothness_score = 1.0
            return waypoints
            
        # Core A* search
        cell_path = self._astar_search((sc, sr), (gc, gr))
        
        if not cell_path:
            print("[A*] WARNING: No path found - falling back to straight line")
            self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
            self.path_length_m = 0.0
            self.smoothness_score = 1.0
            return []
            
        # Convert path cells back to world coordinates
        world_path = [self._grid_to_world(cell) for cell in cell_path]
        
        # Remove start position (index 0) from waypoints as expected by controller
        world_path = world_path[1:]
        
        # If possible, replace the snapped goal cell with the exact goal coordinate
        if world_path:
            gc_orig, gr_orig = self._world_to_grid(goal)
            if self._is_free((gc_orig, gr_orig)):
                world_path[-1] = goal
            else:
                print(f"[A*] Goal {goal} is blocked/inflated. Keeping snapped target {world_path[-1]} to avoid collision.")
                
        # Smooth path
        world_path = self._smooth_path_los(world_path)
        
        # Record stats
        self.compute_time_ms += (time.perf_counter() - start_t) * 1000.0
        
        # Include start position in the path sequence to compute stats correctly
        complete_path = [start] + world_path
        self.path_length_m = self._compute_path_length(complete_path)
        self.smoothness_score = self._compute_smoothness(complete_path)
        
        print(f"[A*] Path found: {len(world_path)} waypoints | "
              f"Time: {self.compute_time_ms:.2f}ms | Nodes: {self.nodes_explored} | "
              f"Len: {self.path_length_m:.1f}m | Smooth: {self.smoothness_score}")
              
        return world_path
