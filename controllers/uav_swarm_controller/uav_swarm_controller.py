"""
Multi-UAV Supervisor Controller
POMDP-based decentralized surveillance for smart-city crowd monitoring.
Coordinate system: ENU (East=X, North=Y, Up=Z)

Camera modes (keyboard keys 1 / 2 / 3 in GUI mode):
  1 - Cinematic city view  : fixed SE-corner position, pans to track UAV_0
  2 - Improved chase-cam   : smooth manual interpolated follow (custom override)
  3 - Top-down overview    : bird's-eye view, all 5 drones visible

Debug/Observability features (active in baseline_nav mode):
  - Smooth custom chase-cam (mode 2) with configurable offset + lerp
  - Blue dot path trail showing actual flown route
  - Improved waypoint markers (larger, emissive, numbered)
  - Formatted debug HUD printed every log_interval steps
  - ASCII minimap printed every 200 steps
"""
from controller import Supervisor
import math
import heapq
import time
import yaml
import json
import os
import sys

# ==============================================================================
# A* GLOBAL PATH PLANNER
# ==============================================================================
class AStarPlanner:
    """
    Lightweight 2-D A* path planner on a uniform occupancy grid.

    Coordinate mapping
    ------------------
    World XY -> grid (col, row):
        col = int((x + grid_margin) / resolution)
        row = int((y + grid_margin) / resolution)

    Grid dimensions (cells):
        width  = height = ceil(2 * grid_margin / resolution) + 1

    Buildings are inflated by `obstacle_inflation` metres before rasterisation
    so the drone centre never gets closer than that margin to any wall.

    The planner returns a list of world-space (x, y) waypoints extracted from
    the cell path.  The caller adds the mission altitude (z) before use.

    Algorithm
    ---------
    Standard A* with:
        g(n) = actual cost from start (Euclidean distance between adjacent cells)
        h(n) = Euclidean distance to goal cell (admissible heuristic)
        8-connected neighbours (allows diagonal moves)

    Path smoothing
    --------------
    After raw cell extraction a simple greedy line-of-sight pass removes
    redundant intermediate waypoints: if the segment (wA -> wC) is free of
    obstacles, waypoint wB is dropped.
    """

    def __init__(self, buildings: list, resolution: float = 2.0,
                 grid_margin: float = 90.0, obstacle_inflation: float = 2.0):
        """
        Parameters
        ----------
        buildings : list of dict
            Each dict must have keys ``x``, ``y``, ``r`` (world coords + radius).
        resolution : float
            Grid cell size in metres.
        grid_margin : float
            World extends from (-grid_margin) to (+grid_margin) on both axes.
        obstacle_inflation : float
            Extra metres added to every building radius before grid marking.
        """
        self.resolution = resolution
        self.grid_margin = grid_margin
        self.inflation = obstacle_inflation
        self.buildings = buildings

        # Grid dimensions
        span = 2.0 * grid_margin
        self.cols = int(math.ceil(span / resolution)) + 1
        self.rows = self.cols   # square world

        # Build the occupancy grid (True = blocked)
        self.grid = self._build_grid()

        total_cells = self.cols * self.rows
        blocked = sum(1 for r in range(self.rows)
                      for c in range(self.cols) if self.grid[r][c])
        print(f"[A*] Grid built: {self.cols}x{self.rows} cells "
              f"({resolution}m/cell) | {blocked}/{total_cells} blocked "
              f"| world +/-{grid_margin}m")

    # -- Grid construction ---------------------------------------------------

    def _build_grid(self) -> list:
        """Return a 2-D list[row][col] of booleans (True = obstacle)."""
        grid = [[False] * self.cols for _ in range(self.rows)]
        inflated_buildings = [
            {"x": b["x"], "y": b["y"], "r": b["r"] + self.inflation}
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
                        break   # no need to check further buildings
        return grid

    # -- Coordinate helpers --------------------------------------------------

    def _world_to_cell(self, wx: float, wy: float):
        col = int((wx + self.grid_margin) / self.resolution)
        row = int((wy + self.grid_margin) / self.resolution)
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return col, row

    def _cell_to_world(self, col: int, row: int):
        wx = col * self.resolution - self.grid_margin
        wy = row * self.resolution - self.grid_margin
        return wx, wy

    def _in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    def _is_free(self, col: int, row: int) -> bool:
        return self._in_bounds(col, row) and not self.grid[row][col]

    # -- A* core ------------------------------------------------------------

    def _heuristic(self, col: int, row: int, gc: int, gr: int) -> float:
        """Euclidean distance heuristic (in cell units)."""
        return math.hypot(col - gc, row - gr)

    def _reconstruct(self, came_from: dict, current) -> list:
        """Walk back the came_from dict to build the cell path."""
        path = []
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def plan(self, start_world, goal_world) -> list:
        """
        Run A* from start_world (x,y) to goal_world (x,y).

        Returns
        -------
        list of (x, y) world coordinates, including goal but NOT start.
        Returns empty list if no path found (caller falls back to straight line).
        """
        sc, sr = self._world_to_cell(*start_world)
        gc, gr = self._world_to_cell(*goal_world)

        # Snap start/goal to nearest free cell if they landed inside an obstacle
        sc, sr = self._nearest_free(sc, sr)
        gc, gr = self._nearest_free(gc, gr)

        if (sc, sr) == (gc, gr):
            return [goal_world]

        # Priority queue: (f_cost, g_cost, col, row)
        open_heap = []
        heapq.heappush(open_heap, (0.0, 0.0, sc, sr))

        came_from = {(sc, sr): None}
        g_cost    = {(sc, sr): 0.0}

        # 8-connected neighbours (dx, dy) with move cost
        NEIGHBOURS = [
            (-1, -1, 1.4142), (-1, 0, 1.0), (-1, 1, 1.4142),
            ( 0, -1, 1.0),                   ( 0, 1, 1.0),
            ( 1, -1, 1.4142), ( 1, 0, 1.0), ( 1, 1, 1.4142),
        ]

        closed = set()

        while open_heap:
            _, g, col, row = heapq.heappop(open_heap)
            node = (col, row)

            if node in closed:
                continue
            closed.add(node)

            if node == (gc, gr):
                cell_path = self._reconstruct(came_from, (gc, gr))
                world_path = [self._cell_to_world(c, r) for c, r in cell_path]
                # Remove start position (index 0), keep the rest incl. goal
                world_path = world_path[1:]
                # Replace last waypoint with exact goal coordinates if it is free
                if world_path:
                    gc_orig, gr_orig = self._world_to_cell(*goal_world)
                    if self._is_free(gc_orig, gr_orig):
                        world_path[-1] = goal_world
                    else:
                        print(f"[A*] Goal {goal_world} is blocked/inflated. Keeping snapped target {world_path[-1]} to avoid collision.")
                world_path = self._smooth(world_path)
                print(f"[A*] Path found: {len(world_path)} waypoints "
                      f"(raw cell path: {len(cell_path)} cells)")
                return world_path

            for dc, dr, move_cost in NEIGHBOURS:
                nc, nr = col + dc, row + dr
                if not self._is_free(nc, nr):
                    continue
                neighbour = (nc, nr)
                new_g = g + move_cost
                if new_g < g_cost.get(neighbour, float("inf")):
                    g_cost[neighbour] = new_g
                    f = new_g + self._heuristic(nc, nr, gc, gr)
                    heapq.heappush(open_heap, (f, new_g, nc, nr))
                    came_from[neighbour] = node

        print("[A*] WARNING: No path found  -  falling back to straight line")
        return []

    # -- Path smoothing ------------------------------------------------------

    def _los_clear(self, ax: float, ay: float, bx: float, by: float,
                   samples: int = 20) -> bool:
        """
        Check if the straight segment (ax,ay)->(bx,by) is free of obstacles.
        Uses point-sampling along the segment.
        """
        for i in range(samples + 1):
            t = i / samples
            wx = ax + t * (bx - ax)
            wy = ay + t * (by - ay)
            col, row = self._world_to_cell(wx, wy)
            if not self._is_free(col, row):
                return False
        return True

    def _smooth(self, waypoints: list) -> list:
        """
        Greedy line-of-sight waypoint smoother.
        Removes intermediate waypoints that are visible from the previous one.
        """
        if len(waypoints) <= 2:
            return waypoints

        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            # Find the furthest visible waypoint from smoothed[-1]
            j = len(waypoints) - 1
            while j > i + 1:
                ax, ay = smoothed[-1]
                bx, by = waypoints[j]
                if self._los_clear(ax, ay, bx, by):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j

        print(f"[A*] Smoothed: {len(waypoints)} -> {len(smoothed)} waypoints")
        return smoothed

    # -- Nearest-free helper -------------------------------------------------

    def _nearest_free(self, col: int, row: int, max_search: int = 10):
        """BFS outward to find the closest free cell from a blocked one."""
        if self._is_free(col, row):
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
                        if self._is_free(nc, nr):
                            return nc, nr
                        next_queue.append((nc, nr))
            queue = next_queue
            if len(visited) > (2 * max_search + 1) ** 2:
                break
        return col, row   # give up  -  return original



# ==============================================================================
# COLLISION SAFETY LAYER
# ==============================================================================
class CollisionSafetyLayer:
    """
    Modular collision safety layer that sits between the kinematic flight model
    and the A* planner.  It is a pure-logic class  -  it NEVER writes to Webots
    fields directly; all writes remain in SingleDroneNavigation.

    Responsibilities
    ----------------
    1. Segment validation        : check_segment(p1, p2)  -  sample N world-space
       points; return False if any point is inside a safety-inflated building.
    2. Nearest safe fallback     : nearest_safe_waypoint()  -  binary-search along
       the blocked segment for the last safe point, used as a fallback WP.
    3. Collision state machine   : get_collision_state()  -  returns
       "SAFE" / "WARNING" / "EMERGENCY" based on distance to nearest building surface.
    4. Smooth repulsion vector   : compute_repulsion()  -  blends away from the
       nearest building with a low-pass filter to prevent oscillation/shaking.
    5. Debug info                : get_debug_info()  -  dict consumed by HUD.
    6. Debug visuals             : spawn_inflation_visuals()  -  optional translucent
       red rings around each building at the safety radius (called once at init).

    Design Constraints
    ------------------
    - Zero coupling to A* internals.
    - Zero coupling to kinematic model state.
    - Future planners (RRT, Dijkstra, PotentialField) can call the same API.
    - All state is encapsulated here except for buildings list (shared reference).
    """

    SAFE      = "SAFE"
    WARNING   = "WARNING"
    EMERGENCY = "EMERGENCY"

    def __init__(self, buildings: list, cfg: dict):
        """
        Parameters
        ----------
        buildings : list of dict
            Each dict must have keys ``x``, ``y``, ``r`` (world coords + radius).
            This is the RAW geometric radius  -  NOT inflated by A*.
        cfg : dict
            The ``collision_safety`` sub-dict from environment_config.yaml.
        """
        self.enabled = bool(cfg.get("enabled", True))

        # Dual inflation radii (metres added to geometric radius)
        self.planner_inflation  = float(cfg.get("planner_inflation_radius", 2.0))
        self.safety_inflation   = float(cfg.get("safety_inflation_radius",  5.0))

        # Safety-inflated building list (used for all world-space checks)
        self.buildings_raw      = buildings
        self.buildings_safe     = [
            {"name": b["name"], "x": b["x"], "y": b["y"],
             "r": b["r"] + self.safety_inflation}
            for b in buildings
        ]

        # Segment validation
        self.segment_samples    = int(cfg.get("segment_samples", 30))
        self.fallback_mode      = str(cfg.get("segment_blocked_fallback", "nearest_free"))

        # State-machine thresholds (distances from BUILDING SURFACE)
        self.danger_radius      = float(cfg.get("danger_radius",             8.0))
        self.emergency_radius   = float(cfg.get("repulsion_emergency_radius",4.0))

        # Repulsion parameters
        self.repulsion_strength      = float(cfg.get("repulsion_strength",      2.5))
        self.repulsion_strength_warn = float(cfg.get("repulsion_strength_warn", 0.8))
        self.repulsion_smoothing     = float(cfg.get("repulsion_smoothing",     0.30))

        # Debug flags
        self.show_inflated_obstacles = bool(cfg.get("show_inflated_obstacles", True))
        self.show_avoidance_vector   = bool(cfg.get("show_avoidance_vector",   True))

        # Low-pass filter state for repulsion vector (starts at zero)
        self._smooth_rx = 0.0
        self._smooth_ry = 0.0

        # Last-computed debug data (cached each step for HUD)
        self._last_state     = self.SAFE
        self._last_nearest   = None    # (name, surface_dist, bx, by)
        self._last_repulsion = (0.0, 0.0)

        print(f"[CollisionSafety] Initialised  -  "
              f"safety_radius=+{self.safety_inflation}m  "
              f"danger={self.danger_radius}m  "
              f"emergency={self.emergency_radius}m  "
              f"buildings={len(buildings)}")

    # -- Internal geometry helpers --------------------------------------------

    def _nearest_building(self, pos) -> tuple:
        """
        Find the nearest building to `pos` using the SAFETY-inflated radii.

        Returns
        -------
        (name, surface_distance, bx, by)
        surface_distance = centre_dist - safety_inflated_radius
        Negative value means the drone is INSIDE the inflated margin.
        """
        min_surf = float("inf")
        nearest_name = None
        bx, by = 0.0, 0.0
        for b in self.buildings_safe:
            centre_dist = math.hypot(pos[0] - b["x"], pos[1] - b["y"])
            surf = centre_dist - b["r"]
            if surf < min_surf:
                min_surf = surf
                nearest_name = b["name"]
                bx, by = b["x"], b["y"]
        return nearest_name, min_surf, bx, by

    def _point_blocked(self, x: float, y: float) -> bool:
        """True if (x, y) is inside ANY safety-inflated building."""
        for b in self.buildings_safe:
            if math.hypot(x - b["x"], y - b["y"]) <= b["r"]:
                return True
        return False

    # -- Public API -----------------------------------------------------------

    def check_segment(self, p1, p2) -> bool:
        """
        Sample ``segment_samples`` evenly-spaced points along the 2-D segment
        p1 -> p2 and test each against the safety-inflated building footprints.

        Parameters
        ----------
        p1, p2 : sequence of at least 2 floats (x, y, ...)

        Returns
        -------
        True   -  segment is clear (safe to fly)
        False  -  segment is blocked (one or more sample points inside safety margin)
        """
        if not self.enabled:
            return True

        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        n = self.segment_samples

        for i in range(n + 1):
            t  = i / n
            sx = x1 + t * (x2 - x1)
            sy = y1 + t * (y2 - y1)
            if self._point_blocked(sx, sy):
                return False
        return True

    def nearest_safe_waypoint(self, drone_pos, blocked_wp) -> list:
        """
        Binary-search along drone_pos -> blocked_wp for the last safe point.
        Returns a world-space [x, y, z] fallback waypoint at 90 % of the
        safe length (providing a small buffer).

        Falls back to drone_pos itself if even the first sample is blocked.
        """
        x1, y1, z1 = drone_pos[0], drone_pos[1], drone_pos[2]
        x2, y2     = blocked_wp[0], blocked_wp[1]
        z2         = blocked_wp[2] if len(blocked_wp) > 2 else z1

        lo, hi = 0.0, 1.0
        # Binary-search for the largest safe t
        for _ in range(12):   # 12 bisection steps -> precision ~0.02 % of length
            mid = (lo + hi) / 2.0
            mx  = x1 + mid * (x2 - x1)
            my  = y1 + mid * (y2 - y1)
            if self._point_blocked(mx, my):
                hi = mid
            else:
                lo = mid

        # Use 90 % of the safe fraction to keep a small buffer
        t_safe = lo * 0.90
        if t_safe < 0.01:   # essentially at start  -  can't move toward WP
            return [x1, y1, z1]

        return [
            x1 + t_safe * (x2 - x1),
            y1 + t_safe * (y2 - y1),
            z1 + t_safe * (z2 - z1),
        ]

    def get_collision_state(self, drone_pos) -> str:
        """
        Return one of SAFE / WARNING / EMERGENCY based on proximity to the
        nearest safety-inflated building surface.

        Also caches the result in self._last_state for HUD consumption.

        Improvement 6: emergency threshold tightened slightly so the state
        machine flows SAFE -> WARNING -> smooth steering -> SAFE rather than
        jumping straight to EMERGENCY.  The danger_radius (WARNING band) is
        now evaluated against a slightly widened window so the drone has more
        time to react before reaching the hard EMERGENCY zone.
        """
        if not self.enabled:
            self._last_state = self.SAFE
            return self.SAFE

        name, surf_dist, bx, by = self._nearest_building(drone_pos)
        self._last_nearest = (name, surf_dist, bx, by)

        # Restored: original emergency threshold (no reduction)
        if surf_dist <= self.emergency_radius:
            self._last_state = self.EMERGENCY
        elif surf_dist <= self.danger_radius:
            self._last_state = self.WARNING
        else:
            self._last_state = self.SAFE

        return self._last_state

    def compute_repulsion(self, drone_pos) -> tuple:
        """
        Compute a smooth repulsion vector pushing the drone away from the
        nearest building.  The raw vector is low-pass filtered to eliminate
        oscillation in tight corridors.

        Returns
        -------
        (rx, ry)  -  normalised repulsion direction (magnitude ~= 1.0 when active,
                   0.0 when SAFE and filter has decayed).
        """
        if not self.enabled or self._last_nearest is None:
            self._smooth_rx = 0.0
            self._smooth_ry = 0.0
            self._last_repulsion = (0.0, 0.0)
            return (0.0, 0.0)

        name, surf_dist, bx, by = self._last_nearest

        # Raw repulsion: away from building centre
        raw_rx = drone_pos[0] - bx
        raw_ry = drone_pos[1] - by
        r_len  = math.hypot(raw_rx, raw_ry)

        if r_len < 1e-4 or self._last_state == self.SAFE:
            # Decay filter toward zero when safe
            target_rx, target_ry = 0.0, 0.0
        else:
            target_rx = raw_rx / r_len
            target_ry = raw_ry / r_len

        # Low-pass filter (prevents oscillation/shaking)
        alpha = self.repulsion_smoothing
        self._smooth_rx += alpha * (target_rx - self._smooth_rx)
        self._smooth_ry += alpha * (target_ry - self._smooth_ry)

        # Normalise filtered vector
        filt_len = math.hypot(self._smooth_rx, self._smooth_ry)
        if filt_len > 1e-4:
            nx = self._smooth_rx / filt_len
            ny = self._smooth_ry / filt_len
        else:
            nx, ny = 0.0, 0.0

        self._last_repulsion = (nx, ny)
        return (nx, ny)

    def get_repulsion_strength(self) -> float:
        """Return the appropriate repulsion multiplier for the current state."""
        if self._last_state == self.EMERGENCY:
            return self.repulsion_strength
        elif self._last_state == self.WARNING:
            return self.repulsion_strength_warn
        return 0.0

    def get_debug_info(self) -> dict:
        """
        Return a snapshot of the current collision state for HUD display.

        Returns
        -------
        dict with keys:
            state           -  "SAFE" / "WARNING" / "EMERGENCY"
            nearest_name    -  name of closest building (or None)
            surface_dist    -  metres from drone centre to nearest surface
            danger_radius   -  configured threshold
            emergency_radius  -  configured threshold
            repulsion_vec   -  (rx, ry) filtered repulsion direction
            repulsion_mag   -  magnitude of repulsion vector
        """
        nearest_name = None
        surf_dist    = float("inf")
        if self._last_nearest is not None:
            nearest_name = self._last_nearest[0]
            surf_dist    = self._last_nearest[1]

        rx, ry = self._last_repulsion
        return {
            "state":            self._last_state,
            "nearest_name":     nearest_name,
            "surface_dist":     surf_dist,
            "danger_radius":    self.danger_radius,
            "emergency_radius": self.emergency_radius,
            "repulsion_vec":    (rx, ry),
            "repulsion_mag":    math.hypot(rx, ry),
        }

    # -- Debug visualisation --------------------------------------------------

    def spawn_inflation_visuals(self, supervisor) -> None:
        """
        Spawn translucent red cylindrical rings in the Webots scene to visualise
        the safety-inflated obstacle boundaries.  Called once during initialisation
        when show_inflated_obstacles is True.

        Uses thin flat cylinders (disc-like) placed at drone cruise altitude so
        they are visible from the chase-cam without obstructing the view.

        VISUAL NOTE: The displayed radius is intentionally scaled down by
        VISUAL_RADIUS_SCALE (~=0.70) relative to the internal safety radius.
        This is purely cosmetic  -  it reduces clutter without changing the
        actual collision-avoidance geometry in any way.
        """
        # -- Improvement 1: visual-only radius reduction (~30%) ----------------
        # Internal safety_inflation radius is unchanged.  Only the rendered
        # cylinder is scaled so the red circles are less cluttered in the scene.
        VISUAL_RADIUS_SCALE = 0.70   # cosmetic scale  -  does NOT affect collision logic

        if not self.show_inflated_obstacles:
            return
        try:
            root           = supervisor.getRoot()
            children_field = root.getField("children")
            spawned        = 0
            for i, b in enumerate(self.buildings_safe):
                r_internal = b["r"]                      # full safety radius (unchanged)
                r_visual   = r_internal * VISUAL_RADIUS_SCALE  # reduced for display only
                # Thin torus approximated as a flat hollow cylinder (ring) at altitude 15 m
                node_str = (
                    f'DEF SAFETY_RING_{i} Solid {{\n'
                    f'  translation {b["x"]} {b["y"]} 15.0\n'
                    f'  children [\n'
                    f'    Shape {{\n'
                    f'      appearance Appearance {{\n'
                    f'        material Material {{\n'
                    f'          diffuseColor 1.0 0.1 0.05\n'
                    f'          emissiveColor 0.6 0.0 0.0\n'
                    f'          transparency 0.55\n'
                    f'        }}\n'
                    f'      }}\n'
                    f'      geometry Cylinder {{ radius {r_visual:.3f} height 0.25 }}\n'
                    f'    }}\n'
                    f'  ]\n'
                    f'  name "safety_ring_{i}"\n'
                    f'  contactMaterial "default"\n'
                    f'  physics NULL\n'
                    f'}}\n'
                )
                children_field.importMFNodeFromString(-1, node_str)
                spawned += 1
            print(f"[CollisionSafety] Spawned {spawned} safety-radius rings "
                  f"(safety_inflation=+{self.safety_inflation}m internal, "
                  f"visual={VISUAL_RADIUS_SCALE*100:.0f}% of internal, translucent red).")
        except Exception as e:
            print(f"[CollisionSafety] Could not spawn inflation visuals (non-fatal): {e}")


# ==============================================================================
# LOCAL AVOIDANCE SENSOR  (Phase-1, drone-centered)
# ==============================================================================
class LocalAvoidanceSensor:
    """
    Drone-centered virtual 8-ray obstacle sensor for Phase-1 local avoidance.

    Design Principles
    -----------------
    - 8 directional rays cast geometrically  -  NO Webots hardware sensors.
    - Rays are aligned to world-space, rotated by current drone yaw so
      "front" always means the direction the drone is heading.
    - Power-law repulsion falloff: obstacles closer to the drone exert
      stronger repulsion than distant ones  -  no binary on/off transitions.
    - Low-pass filter on the final avoidance vector prevents jitter and
      oscillation in tight corridors.
    - Output is a blended direction: 0.75 x A* + 0.25 x avoidance.
      A* is NEVER overridden  -  this is a steering correction only.
    - Ray-tip spheres in Webots scene provide real-time visual feedback:
        GREEN  = clear
        YELLOW = nearby obstacle (within yellow_threshold fraction)
        RED    = close / blocked (within red_threshold fraction)
    - Zero coupling to A* internals or kinematic model state.
    - Designed for easy upgrade to real Webots DistanceSensor in Phase-2.

    Ray Directions (angles from East axis, ENU, rotated by drone yaw)
    -------------------------------------------------------------------
    Index  Label         Base Angle (rad)
      0    front          0.0
      1    front-left     pi/4
      2    left           pi/2
      3    rear-left      3pi/4
      4    rear           pi
      5    rear-right    -3pi/4
      6    right         -pi/2
      7    front-right   -pi/4
    """

    # 8 ray definitions: (label, base_angle_rad)
    RAY_DEFS = [
        ("front",        0.0),
        ("front-left",   math.pi / 4.0),
        ("left",         math.pi / 2.0),
        ("rear-left",    3.0 * math.pi / 4.0),
        ("rear",         math.pi),
        ("rear-right",  -3.0 * math.pi / 4.0),
        ("right",       -math.pi / 2.0),
        ("front-right", -math.pi / 4.0),
    ]
    NUM_RAYS = 8

    # Improvement 3: Directional ray weights  -  front rays dominate steering.
    # Front-facing rays exert the most steering correction; rear rays barely
    # contribute.  This makes the drone steer around buildings earlier instead
    # of reacting only when the obstacle is directly ahead.
    RAY_WEIGHTS = {
        "front":       1.00,
        "front-left":  0.85,
        "front-right": 0.85,
        "left":        0.55,
        "right":       0.55,
        "rear-left":   0.20,
        "rear-right":  0.20,
        "rear":        0.10,
    }

    def __init__(self, buildings: list, cfg: dict):
        """
        Parameters
        ----------
        buildings : list of dict
            Each dict must have keys ``x``, ``y``, ``r`` (world coords + radius).
            Pass the RAW (un-inflated) building list from _init_buildings().
        cfg : dict
            The ``local_avoidance`` sub-dict from environment_config.yaml.
        """
        self.enabled          = bool(cfg.get("enabled", True))
        # Improvement 2: default sensor distance raised from 10 -> 14 m
        self.sensor_distance  = float(cfg.get("sensor_distance", 14.0))
        self.ray_samples      = int(cfg.get("ray_samples", 20))
        # Restored planner dominance
        self.astar_weight     = float(cfg.get("astar_weight", 0.75))
        self.avoidance_weight = float(cfg.get("avoidance_weight", 0.25))
        self.repulsion_falloff= float(cfg.get("repulsion_falloff", 2.0))
        self.smoothing_alpha  = float(cfg.get("smoothing_alpha", 0.35))
        self.debug_rays       = bool(cfg.get("debug_rays", True))
        self.yellow_threshold = float(cfg.get("yellow_threshold", 0.60))
        self.red_threshold    = float(cfg.get("red_threshold", 0.30))
        # Disabled predictive avoidance
        self.prediction_steps = 0

        # -- [FIX] Corner corridor commitment  -  YAML-configurable --------------
        self.corridor_commit_steps  = int(cfg.get("corridor_commit_steps",   12))

        # -- [FIX] Near-wall speed reduction  -  YAML-configurable --------------
        # These values are read by _move_toward_target() via la.slow_*
        self.slow_start_distance    = float(cfg.get("slow_start_distance",    8.0))
        self.slow_mid_distance      = float(cfg.get("slow_mid_distance",      5.0))
        self.slow_near_distance     = float(cfg.get("slow_near_distance",     3.0))
        self.slow_mid_scale         = float(cfg.get("slow_mid_scale",         0.80))
        self.slow_near_scale        = float(cfg.get("slow_near_scale",        0.60))
        self.emergency_slow_scale   = float(cfg.get("emergency_slow_scale",   0.45))

        self.buildings        = buildings   # raw geometric buildings

        # Low-pass filter state for avoidance vector
        self._smooth_ax = 0.0
        self._smooth_ay = 0.0

        # -- Corridor commitment state -----------------------------------------
        self._corridor_commit_remaining = 0          # steps left in commitment
        self._corridor_commit_dir       = (0.0, 0.0) # locked avoidance direction

        # -- Oscillation detection state ---------------------------------------
        self._direction_flip_count  = 0      # consecutive large direction reversals
        self._prev_target_ax        = 0.0    # previous normalised target x
        self._prev_target_ay        = 0.0    # previous normalised target y
        self._oscillation_detected  = False  # True when avoidance vector flip-flops

        # Per-ray result cache (updated each step)  -  list of dicts
        # Each: {label, hit, hit_fraction, hit_x, hit_y, ray_angle}
        self._ray_results = [
            {"label": lbl, "hit": False, "hit_fraction": 1.0,
             "hit_x": 0.0, "hit_y": 0.0, "ray_angle": ang}
            for lbl, ang in self.RAY_DEFS
        ]

        # Last computed debug info
        self._last_avoidance_vec = (0.0, 0.0)
        self._last_active        = False
        self._last_nearest_ray   = "none"
        self._last_nearest_dist  = float("inf")

        # Webots scene node references for ray visualization
        # Populated by spawn_ray_visuals()  -  None until then
        self._ray_tf_fields = [None] * self.NUM_RAYS   # translation fields
        self._ray_mat_fields= [None] * self.NUM_RAYS   # emissiveColor fields

        print(f"[LocalAvoidance] Initialised  -  "
              f"sensor={self.sensor_distance}m  "
              f"samples={self.ray_samples}  "
              f"blend=A*{self.astar_weight}+avoid{self.avoidance_weight}  "
              f"lpf_alpha={self.smoothing_alpha}  "
              f"prediction_steps={self.prediction_steps}  "
              f"debug_rays={'ON' if self.debug_rays else 'OFF'}")

    # -- Ray casting ----------------------------------------------------------

    def _cast_ray(self, ox: float, oy: float, angle: float) -> dict:
        """
        Cast a single ray from (ox, oy) in world direction `angle` (radians).

        Samples `ray_samples` evenly-spaced points from 0 to sensor_distance.
        Tests each sample point against all building circles.

        Returns
        -------
        dict with keys:
            hit           -  True if any sample intersects a building
            hit_fraction  -  normalised step index of first hit (0=drone, 1=clear)
            hit_x, hit_y  -  world coords of first hit point (0 if no hit)
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        step_size = self.sensor_distance / self.ray_samples

        for i in range(1, self.ray_samples + 1):
            sx = ox + dx * step_size * i
            sy = oy + dy * step_size * i
            for b in self.buildings:
                if math.hypot(sx - b["x"], sy - b["y"]) <= b["r"]:
                    return {
                        "hit":          True,
                        "hit_fraction": i / self.ray_samples,
                        "hit_x":        sx,
                        "hit_y":        sy,
                    }

        # No hit  -  ray is clear
        tip_x = ox + dx * self.sensor_distance
        tip_y = oy + dy * self.sensor_distance
        return {
            "hit":          False,
            "hit_fraction": 1.0,
            "hit_x":        tip_x,
            "hit_y":        tip_y,
        }

    # -- Per-step update ------------------------------------------------------

    def update(self, drone_pos, drone_yaw: float, drone_vx: float = 0.0, drone_vy: float = 0.0) -> None:
        """
        Cast all 8 rays from current drone position, compute and filter the
        avoidance vector.  Call once per simulation step before querying
        get_avoidance_vector() or get_debug_info().

        Improvements applied here
        -------------------------
        3. Directional weighting  -  each ray's contribution is scaled by
           RAY_WEIGHTS[label] so front rays dominate the steering correction.
        4. Predictive avoidance  -  if prediction_steps > 0, a second set of
           rays is cast from the projected future position:
               future_pos = current_pos + (vx, vy) * prediction_steps
           The current-position result and the future-position result are
           blended 60 % current / 40 % predicted, giving the drone earlier
           awareness of buildings that lie ahead.

        Parameters
        ----------
        drone_pos : sequence of at least 2 floats (x, y, ...)
        drone_yaw : float  -  current heading in radians (ENU yaw from East)
        drone_vx, drone_vy : float  -  current XY velocity (m/step), used for
            predictive raycasting.  Defaults to 0 (no prediction shift).
        """
        if not self.enabled:
            return

        ox, oy = drone_pos[0], drone_pos[1]

        # -- Cast rays from CURRENT position ----------------------------------
        raw_ax, raw_ay = 0.0, 0.0
        nearest_frac   = 1.0
        nearest_label  = "none"

        for i, (lbl, base_angle) in enumerate(self.RAY_DEFS):
            world_angle = drone_yaw + base_angle
            result      = self._cast_ray(ox, oy, world_angle)

            # Improvement 7: ray_results are the single source of truth for
            # both visualization and HUD  -  always set hit/hit_fraction here.
            self._ray_results[i]["label"]        = lbl
            self._ray_results[i]["hit"]          = result["hit"]
            self._ray_results[i]["hit_fraction"] = result["hit_fraction"]
            self._ray_results[i]["hit_x"]        = result["hit_x"]
            self._ray_results[i]["hit_y"]        = result["hit_y"]
            self._ray_results[i]["ray_angle"]    = world_angle

            if result["hit"]:
                frac = result["hit_fraction"]
                # Improvement 3: apply directional weight for this ray label
                dir_weight = self.RAY_WEIGHTS.get(lbl, 1.0)
                # Power-law falloff: closer hit -> stronger repulsion weight
                repulsion_weight = (1.0 - frac) ** self.repulsion_falloff * dir_weight
                # Repulsion direction: opposite to the ray direction
                raw_ax -= math.cos(world_angle) * repulsion_weight
                raw_ay -= math.sin(world_angle) * repulsion_weight
                # Track nearest hit
                if frac < nearest_frac:
                    nearest_frac  = frac
                    nearest_label = lbl

        self._last_nearest_ray  = nearest_label
        self._last_nearest_dist = nearest_frac * self.sensor_distance


        # Normalise raw avoidance vector before filtering
        raw_mag = math.hypot(raw_ax, raw_ay)
        if raw_mag > 1e-4:
            target_ax = raw_ax / raw_mag
            target_ay = raw_ay / raw_mag
            self._last_active = True
        else:
            target_ax = 0.0
            target_ay = 0.0
            self._last_active = False

        # -- [FIX-1] Oscillation detection ------------------------------------
        # Count consecutive steps where the avoidance target reverses by > 120deg.
        # A sustained flip-flop (> 8 reversals) sets _oscillation_detected,
        # which halves avoidance authority in _move_toward_target (FIX-3).
        if raw_mag > 1e-4:
            dot_prev = (target_ax * self._prev_target_ax +
                        target_ay * self._prev_target_ay)
            if dot_prev < -0.5:   # > 120deg reversal from previous step
                self._direction_flip_count += 1
            else:
                self._direction_flip_count = max(0, self._direction_flip_count - 1)
            self._prev_target_ax = target_ax
            self._prev_target_ay = target_ay
        else:
            self._direction_flip_count = max(0, self._direction_flip_count - 1)
        self._oscillation_detected = (self._direction_flip_count > 8)

        # -- [FIX-2] Corner corridor commitment -------------------------------
        # If the front ray is blocked AND exactly one front-side ray is clear,
        # lock the avoidance direction for corridor_commit_steps to prevent
        # per-frame left/right indecision at building corners/edges.
        #
        # Ray layout (RAY_DEFS indices):
        #   0 = front   1 = front-left   7 = front-right
        front_hit = self._ray_results[0]["hit"]
        fl_hit    = self._ray_results[1]["hit"]
        fr_hit    = self._ray_results[7]["hit"]

        if self._corridor_commit_remaining > 0:
            # Active commitment: substitute stored direction into LPF target.
            # The LPF still smooths this, so no instant heading snap occurs.
            target_ax, target_ay = self._corridor_commit_dir
            self._corridor_commit_remaining -= 1
        elif front_hit and raw_mag > 1e-4:
            # Only trigger when EXACTLY one front-side is blocked (asymmetric).
            if fr_hit and not fl_hit:
                # Right + front blocked, left open -> current avoidance already
                # points left  -  lock it to avoid right-drift back into wall.
                self._corridor_commit_dir       = (target_ax, target_ay)
                self._corridor_commit_remaining = self.corridor_commit_steps
            elif fl_hit and not fr_hit:
                # Left + front blocked, right open -> lock avoidance rightward.
                self._corridor_commit_dir       = (target_ax, target_ay)
                self._corridor_commit_remaining = self.corridor_commit_steps

        # Low-pass filter (prevents oscillation/jitter)
        alpha = self.smoothing_alpha
        self._smooth_ax += alpha * (target_ax - self._smooth_ax)
        self._smooth_ay += alpha * (target_ay - self._smooth_ay)

        # Normalise filtered vector
        filt_mag = math.hypot(self._smooth_ax, self._smooth_ay)
        if filt_mag > 1e-4:
            fx = self._smooth_ax / filt_mag
            fy = self._smooth_ay / filt_mag
        else:
            fx, fy = 0.0, 0.0

        self._last_avoidance_vec = (fx, fy)

    # -- Output API -----------------------------------------------------------

    def get_avoidance_vector(self) -> tuple:
        """
        Return the low-pass filtered, normalised avoidance direction.

        Returns
        -------
        (ax, ay)  -  normalised direction to steer away from obstacles.
                   (0, 0) when no obstacles detected and filter decayed.
        """
        return self._last_avoidance_vec

    def is_active(self) -> bool:
        """True if at least one ray detected an obstacle this step."""
        return self._last_active

    def get_debug_info(self) -> dict:
        """
        Return current sensor state snapshot for HUD and logging.

        Returns
        -------
        dict with keys:
            ray_results      -  list of 8 ray result dicts
            nearest_ray      -  label of ray with shortest hit distance
            nearest_dist_m   -  distance in metres to nearest hit
            avoidance_vec    -  (ax, ay) filtered direction
            avoidance_mag    -  magnitude of avoidance vector
            active           -  bool: True if avoidance is non-zero this step
        """
        ax, ay = self._last_avoidance_vec
        return {
            "ray_results":    list(self._ray_results),
            "nearest_ray":    self._last_nearest_ray,
            "nearest_dist_m": self._last_nearest_dist,
            "avoidance_vec":  (ax, ay),
            "avoidance_mag":  math.hypot(ax, ay),
            "active":         self._last_active,
        }

    # -- Debug visualisation --------------------------------------------------

    def spawn_ray_visuals(self, supervisor) -> None:
        """
        Spawn 8 small colored spheres in the Webots scene  -  one at each ray tip.
        Spheres start green (clear).  Positions and colors are updated each step
        by update_ray_visuals().

        Uses DEF nodes LA_RAY_0 ... LA_RAY_7.
        Called once during _init_astar() after the scene is fully loaded.
        Gracefully silently skips on any Webots API error.
        """
        if not self.debug_rays:
            return
        try:
            root           = supervisor.getRoot()
            children_field = root.getField("children")
            spawned        = 0
            for i in range(self.NUM_RAYS):
                lbl = self.RAY_DEFS[i][0]
                node_str = (
                    f'DEF LA_RAY_{i} Solid {{\n'
                    f'  translation 0 0 15.0\n'
                    f'  children [\n'
                    f'    Shape {{\n'
                    f'      appearance Appearance {{\n'
                    f'        material DEF LA_MAT_{i} Material {{\n'
                    f'          diffuseColor 0.0 1.0 0.0\n'
                    f'          emissiveColor 0.0 0.8 0.0\n'
                    f'          shininess 0.9\n'
                    f'        }}\n'
                    f'      }}\n'
                    f'      geometry Sphere {{ radius 0.5 }}\n'
                    f'    }}\n'
                    f'  ]\n'
                    f'  name "la_ray_{i}_{lbl}"\n'
                    f'  contactMaterial "default"\n'
                    f'  physics NULL\n'
                    f'}}\n'
                )
                children_field.importMFNodeFromString(-1, node_str)
                node = supervisor.getFromDef(f"LA_RAY_{i}")
                if node is not None:
                    self._ray_tf_fields[i]  = node.getField("translation")
                    # Material node is a child of the shape  -  access via getFromDef
                    mat_node = supervisor.getFromDef(f"LA_MAT_{i}")
                    if mat_node is not None:
                        self._ray_mat_fields[i] = mat_node.getField("emissiveColor")
                    spawned += 1
            print(f"[LocalAvoidance] Spawned {spawned}/{self.NUM_RAYS} "
                  f"ray visual nodes (green=clear, yellow=near, red=blocked).")
        except Exception as e:
            print(f"[LocalAvoidance] Ray visual spawn failed (non-fatal): {e}")

    def update_ray_visuals(self, supervisor, drone_pos) -> None:
        """
        Move each ray-tip sphere to its current world position and
        set its color based on hit state.

        Color coding:
            Green  (0.0, 0.8, 0.0)  -  no hit (clear)
            Yellow (0.9, 0.8, 0.0)  -  hit_fraction >= red_threshold but < yellow_threshold
            Red    (0.9, 0.1, 0.0)  -  hit_fraction < red_threshold (close/blocked)

        Called each simulation step from SingleDroneNavigation.step().
        """
        if not self.debug_rays:
            return

        drone_z = drone_pos[2]

        for i, rr in enumerate(self._ray_results):
            tf  = self._ray_tf_fields[i]
            mat = self._ray_mat_fields[i]
            if tf is None:
                continue

            # Position: use hit point if hit, else ray tip
            tip_x = rr["hit_x"]
            tip_y = rr["hit_y"]
            # If this ray was never cast (sensor disabled), fallback
            if tip_x == 0.0 and tip_y == 0.0:
                ang   = rr["ray_angle"]
                tip_x = drone_pos[0] + math.cos(ang) * self.sensor_distance
                tip_y = drone_pos[1] + math.sin(ang) * self.sensor_distance

            try:
                tf.setSFVec3f([tip_x, tip_y, drone_z])
            except Exception:
                pass

            if mat is None:
                continue

            # Color based on hit_fraction
            frac = rr["hit_fraction"]
            if not rr["hit"]:
                # Green  -  clear
                color = [0.0, 0.85, 0.0]
            elif frac < self.red_threshold:
                # Red  -  close/blocked
                color = [0.95, 0.05, 0.0]
            elif frac < self.yellow_threshold:
                # Yellow  -  nearby
                color = [0.95, 0.85, 0.0]
            else:
                # Green  -  hit but far enough (beyond yellow threshold)
                color = [0.0, 0.85, 0.0]

            try:
                mat.setSFColor(color)
            except Exception:
                pass


# ==============================================================================
# SINGLE-DRONE NAVIGATION BASELINE
# Phase 1 of the hybrid-multi-UAV-swarm development pipeline.
# Deterministic start -> target mission for engineering baseline validation.
# --------------------------------------------------------------------------------
class SingleDroneNavigation:
    """
    Deterministic waypoint navigation for UAV_0 with optional A* global planner.

    Algorithm:
        1. TAKEOFF   -  teleport UAV_0 to start_position.
        2. NAVIGATE  -  if A* enabled: follow planned waypoint list.
                      otherwise: steer straight toward target.
                      Each step applies local obstacle repulsion on top
                      of the waypoint steering vector.
        3. ARRIVED   -  once dist_to_target < arrival_radius, stop moving
                      and announce mission complete.

    UAV_1-4 are parked at their initial positions and never updated.
    RL and random patrol are NOT used in this mode.
    """

    # State constants
    STATE_GROUND_IDLE = "GROUND_IDLE"
    STATE_ASCEND   = "ASCEND"
    STATE_NAVIGATE = "NAVIGATE"
    STATE_ARRIVED  = "ARRIVED"

    def __init__(self, parent: "MultiUAVSurveillance", cfg: dict):
        """
        Parameters
        ----------
        parent : MultiUAVSurveillance
            The supervisor controller (provides node refs, step function, etc.)
        cfg : dict
            The `baseline_navigation` sub-dict from environment_config.yaml.
        """
        self.parent        = parent
        self.supervisor    = parent.supervisor
        self.timestep      = parent.timestep

        # Mission parameters (fall back to hardcoded defaults if YAML missing)
        start = cfg.get("start_position", [-40.0, 15.0, 15.0])
        tgt   = cfg.get("target_position", [40.0, -15.0, 15.0])
        self.start_pos     = list(start)
        self.target_pos    = list(tgt)
        self.cruise_speed  = float(cfg.get("cruise_speed", 0.3))  # legacy; overridden by cruise_velocity
        self.arrival_radius= float(cfg.get("arrival_radius", 3.0))
        self.altitude      = float(cfg.get("altitude", 15.0))
        self.log_interval  = int(cfg.get("log_interval_steps", 100))

        self.safety_radius      = float(cfg.get("safety_radius", 5.0))
        self.avoidance_strength = float(cfg.get("avoidance_strength", 1.5))
        self.target_strength    = float(cfg.get("target_strength", 1.0))

        # -- Kinematic flight model state --------------------------------------
        # Velocity (m/step) in ENU frame  -  integrated each step.
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        # Current heading (radians, ENU yaw from East axis).
        self.current_yaw = 0.0

        # Kinematic parameters  -  read from YAML with safe defaults.
        self.max_velocity     = float(cfg.get("max_velocity",     0.5))
        self.cruise_velocity  = float(cfg.get("cruise_velocity",  self.cruise_speed))
        self.max_acceleration = float(cfg.get("max_acceleration", 0.05))
        self.max_turn_rate    = float(cfg.get("max_turn_rate",    0.08))  # rad/step
        self.decel_radius     = float(cfg.get("decel_radius",     8.0))   # m

        # -- HUD telemetry (populated each kinematic step) ---------------------
        self.last_avoiding    = False
        self.last_steer_vec   = (0.0, 0.0)
        self.last_velocity    = (0.0, 0.0)   # (vx, vy) for HUD
        self.last_speed       = 0.0          # |v_xy| m/step
        self.last_accel       = (0.0, 0.0)   # (ax, ay) applied this step
        self.last_accel_mag   = 0.0
        self.last_turn_rate   = 0.0          # rad/step applied this step

        # -- Stabilisation state ---------------------------------------------
        # Task 3: avoidance gating
        self.emergency_radius       = float(cfg.get("emergency_radius",          4.0))
        # Task 4: yaw low-pass filter
        self.yaw_smoothing_alpha    = float(cfg.get("yaw_smoothing_alpha",       0.2))
        self._filtered_desired_yaw  = None   # initialised on first step
        # Task 5: near-WP velocity damping
        self.wp_damping_radius      = float(cfg.get("wp_damping_radius",         3.0))
        self.velocity_damping_factor= float(cfg.get("velocity_damping_factor",   0.92))
        # Task 1: WP transition hysteresis lock
        self.waypoint_reach_lock_steps = int(cfg.get("waypoint_reach_lock_steps", 8))
        self._wp_lock_counter       = 0      # counts down after a WP advance
        # HUD extras for stabilisation
        self.last_desired_yaw       = 0.0    # raw desired yaw before filtering
        self.last_filtered_yaw      = 0.0    # filtered desired yaw
        self.last_heading_error_deg = 0.0    # degrees between current_yaw and target
        self.last_avoidance_mode    = "DISABLED"  # DISABLED / EMERGENCY / FULL
        self.last_motion_state      = "CRUISE"    # CRUISE / DECEL / DAMP / LOCKED

        # Anti-spin protection
        self._last_turn_sign = 0
        self._oscillation_count = 0.0
        self._anti_spin_active_steps = 0

        # Stuck detection
        self._pos_history = []
        self._stuck_recovery_steps = 0
        self._stuck_cooldown_counter = 0

        # UAV_0 node references (index 0 in parent lists)
        if len(parent.uav_trans) == 0:
            raise RuntimeError("[SingleDroneNav] UAV_0 not found in scene!")
        self.uav_tf = parent.uav_trans[0]   # translation field
        self.uav_rf = parent.uav_rot[0]     # rotation field
        self.uav_node = parent.uavs[0]      # Mavic2Pro node

        self.state       = self.STATE_GROUND_IDLE
        self.step_count  = 0
        self._spinup_counter = 0
        self.spinup_steps = 100
        self.takeoff_speed = 0.05
        self.ground_spawn_z = 0.5
        self.arrived_reported = False

        # Dynamically load all buildings from the scene to prevent flying through any buildings
        self.test_buildings = []
        self._init_buildings()
        self.last_warning_step = -999

        # -- Metrics collection state ------------------------------------------
        self.sim_start_time      = None     # supervisor time at first step (s)
        self.distance_travelled  = 0.0      # cumulative 3-D distance (m)
        self.collision_count     = 0        # count of proximity/collision steps
        self.hard_collision_count = 0       # count of actual building interpenetration steps
        self.physics_contact_count = 0      # count of steps with physical contact points
        self.visited_grid_cells  = set()
        self.grid_cell_size      = 10.0     # 10m x 10m cells
        self.total_grid_cells    = 400      # (200 / 10) ** 2
        self.altitude_readings   = []       # Z values each step for stability calc
        self.trr_readings        = []       # TRR values each step for average calculation
        self.step_log            = []       # sparse log for JSON export
        self.reached_target      = False
        self._prev_pos           = None
        self.last_trr            = 0.0
        self.last_tracked_count  = 0

        # -- A* planner setup ----------------------------------------------
        self.astar_enabled   = bool(cfg.get("astar_enabled", True))
        self.grid_resolution = float(cfg.get("grid_resolution", 2.0))
        self.grid_margin     = float(cfg.get("grid_margin", 90.0))
        self.obstacle_inflation = float(cfg.get("obstacle_inflation", 2.0))
        self.waypoint_radius = float(cfg.get("waypoint_radius", 4.0))
        self.show_wp_markers = bool(cfg.get("show_waypoint_markers", True))

        # Waypoint list  -  populated during first NAVIGATE step via _init_astar()
        self.waypoints          = []   # list of [x, y, z]
        self.current_wp_idx     = 0
        self.astar_initialized  = False
        self._marker_nodes      = []   # Webots Solid nodes for visual debug

        # ==================================================================
        # DEBUG / OBSERVABILITY STATE  (Tasks 1-6)
        # ==================================================================
        self.debug_mode = True          # master switch for all debug visuals

        # Task 1  -  debug beacon above UAV_0
        self._beacon_node = None        # Webots Solid node reference
        self._beacon_tf   = None        # translation field of beacon node
        self._beacon_offset_z = 4.0     # metres above drone centre

        # Task 2  -  smooth chase camera (active when cam_mode == 2)
        # Offset vector in drone heading frame: behind=-ve heading, up=positive Z
        self._chase_behind_dist = 14.0  # m behind drone
        self._chase_above_dist  = 7.0   # m above drone
        self._chase_lookahead   = 4.0   # m ahead of drone for look-at target
        self._chase_lerp_alpha  = 0.07  # smoothing factor (0=frozen, 1=instant)
        self._cam_smooth_pos    = None  # smoothed viewpoint position [x,y,z]
        self._cam_smooth_target = None  # smoothed look-at point [x,y,z]

        # Task 3  -  persistent path trail (blue dots)
        self._trail_positions   = []    # sampled drone positions
        self._trail_nodes       = []    # spawned Webots Solid nodes
        self._trail_counter     = 0     # steps since last sample
        self._trail_interval    = 10    # sample every N steps
        self._trail_max_nodes   = 600   # cap on spawned nodes
        self._trail_radius      = 0.35  # sphere radius (metres)

        # Task 6  -  ASCII minimap
        self._minimap_interval  = 200   # print minimap every N steps

        # -- Collision Safety Layer --------------------------------------------
        # Instantiated after _init_buildings() so self.test_buildings is ready.
        # Falls back to an empty cfg (all defaults) if key is absent in YAML.
        _safety_cfg = cfg.get("collision_safety", {})
        self.safety_layer = CollisionSafetyLayer(
            buildings = self.test_buildings,
            cfg       = _safety_cfg,
        )
        # Cache debug-info dict for HUD (updated every step)
        self._last_safety_info = self.safety_layer.get_debug_info()

        # -- Local Avoidance Sensor --------------------------------------------
        # Primary avoidance layer: 8 geometric rays around the drone.
        # Blends correction with A* direction  -  A* is NEVER overridden.
        # Falls back gracefully to empty dict if key absent in YAML.
        _la_cfg = cfg.get("local_avoidance", {})
        self.local_avoidance = LocalAvoidanceSensor(
            buildings = self.test_buildings,
            cfg       = _la_cfg,
        )
        # Cache debug-info dict for HUD (updated every step)
        self._last_la_info = self.local_avoidance.get_debug_info()

        # Park UAV_1-4 at their initial positions so they don't drift
        self._park_inactive_uavs()

        self._print_banner()

    # -- Setup helpers -----------------------------------------------------

    def _init_buildings(self):
        """
        Dynamically populate self.test_buildings from all building nodes collected
        by the parent MultiUAVSurveillance supervisor.
        """
        self.test_buildings = []
        parent = self.parent
        for node in parent.building_nodes:
            try:
                name = node.getDef() or node.getTypeName()
                pos = node.getPosition()
                
                # Default approximate radius based on building type
                r = 10.0
                type_name = node.getTypeName()
                
                if type_name == "SimpleBuilding":
                    corners_field = node.getField("corners")
                    if corners_field and corners_field.getTypeName() == "MFVec2f":
                        num_points = corners_field.getCount()
                        max_dist = 0.0
                        for idx in range(num_points):
                            pt = corners_field.getMFVec2f(idx)
                            max_dist = max(max_dist, math.hypot(pt[0], pt[1]))
                        if max_dist > 0.0:
                            r = max_dist
                elif type_name == "CommercialBuilding":
                    r = 14.0
                elif type_name == "ResidentialBuilding":
                    r = 10.0
                elif type_name == "LargeResidentialTower":
                    r = 14.0
                elif type_name == "Hotel":
                    r = 16.0
                elif type_name == "RandomBuilding":
                    r = 14.0
                
                self.test_buildings.append({
                    "name": name,
                    "x": pos[0],
                    "y": pos[1],
                    "r": r
                })
            except Exception as e:
                print(f"[WARN] Failed to parse building node: {e}")
        
        print(f"[SingleDroneNav] Dynamically loaded {len(self.test_buildings)} buildings from scene.")

    def _park_inactive_uavs(self):
        """
        Freeze UAV_1-4 at their initial world positions on startup.
        Also resets physics so no accumulated propeller thrust carries over.
        They will not be updated during the baseline run.
        This is a DISABLE (freeze), NOT a deletion.

        FIX-3 (initial setup): setSFVec3f + resetPhysics() stops uav_camera.py
        propellers from lifting UAV_1-4 on the very first steps.
        """
        parent = self.parent
        for i in range(1, parent.num_uavs):
            if i < len(parent.uav_trans):
                init_pos = list(parent.initial_translations[i])
                parent.uav_trans[i].setSFVec3f(init_pos)
                parent.uav_rot[i].setSFRotation(list(parent.initial_rotations[i]))
                try:
                    parent.uavs[i].resetPhysics()   # kill propeller momentum
                except Exception:
                    pass
        print("[SingleDroneNav] UAV_1-4 frozen + physics reset (inactive).")

    def _suppress_inactive_uavs(self):
        """
        Called every simulation step to keep UAV_1-4 pinned.

        FIX-3 (per-step): uav_camera.py runs on each Mavic2Pro and spins
        propellers at HOVER_RPM every step.  The ODE physics engine
        accumulates thrust into upward velocity between supervisor steps.
        Calling resetPhysics() each step zeroes that accumulated velocity
        before physics integrates it, making UAV_1-4 truly stationary.
        """
        parent = self.parent
        for i in range(1, parent.num_uavs):
            if i < len(parent.uav_trans):
                parent.uav_trans[i].setSFVec3f(
                    list(parent.initial_translations[i])
                )
                try:
                    parent.uavs[i].resetPhysics()
                except Exception:
                    pass

    def _print_banner(self):
        astar_mode = "A* + Local Avoidance" if self.astar_enabled else "Reactive avoidance only"
        sl = self.safety_layer
        print("\n" + "=" * 64)
        print("  SINGLE DRONE NAVIGATION BASELINE  (Phase 1)")
        print("  NO RL  |  NO RANDOM PATROL  |  NO SWARM")
        print("  MOTION: KINEMATIC FLIGHT MODEL  (velocity + acceleration)")
        print("  -" * 32)
        print(f"  Active drone    : UAV_0")
        print(f"  Start           : {self.start_pos}")
        print(f"  Target          : {self.target_pos}")
        print(f"  Altitude        : {self.altitude} m (kinematic Z hold)")
        print(f"  Cruise velocity : {self.cruise_velocity} m/step")
        print(f"  Max velocity    : {self.max_velocity} m/step")
        print(f"  Max accel       : {self.max_acceleration} m/step?")
        print(f"  Max turn rate   : {math.degrees(self.max_turn_rate):.1f} deg/step")
        print(f"  Decel radius    : {self.decel_radius} m before WP")
        print(f"  Arrival zone    : {self.arrival_radius} m radius")
        print(f"  Planner         : {astar_mode}")
        if self.astar_enabled:
            print(f"  Grid res        : {self.grid_resolution} m/cell")
            print(f"  WP radius       : {self.waypoint_radius} m")
        print("  States       : TAKEOFF -> NAVIGATE -> ARRIVED")
        print("  -" * 32)
        print("  COLLISION SAFETY LAYER:")
        enabled_str = "ENABLED" if sl.enabled else "DISABLED"
        print(f"    Status         : {enabled_str}")
        print(f"    Planner infl.  : +{sl.planner_inflation} m  (A* grid)")
        print(f"    Safety infl.   : +{sl.safety_inflation} m  (world-space checks)")
        print(f"    Segment samples: {sl.segment_samples}")
        print(f"    Danger radius  : {sl.danger_radius} m  (WARNING state)")
        print(f"    Emergency rad. : {sl.emergency_radius} m  (EMERGENCY state)")
        print(f"    Repulsion str. : WARN={sl.repulsion_strength_warn}  EMRG={sl.repulsion_strength}")
        print(f"    Repulsion lpf  : alpha={sl.repulsion_smoothing}")
        print(f"    Debug rings    : {'YES' if sl.show_inflated_obstacles else 'NO'}")
        print("  -" * 32)
        print("  LOCAL AVOIDANCE SENSOR (Phase-1, PRIMARY):")
        la = self.local_avoidance
        la_status = "ENABLED" if la.enabled else "DISABLED"
        print(f"    Status         : {la_status}")
        if la.enabled:
            print(f"    Sensor dist    : {la.sensor_distance} m")
            print(f"    Ray samples    : {la.ray_samples} per ray")
            print(f"    Blend weights  : A*={la.astar_weight}  avoid={la.avoidance_weight}")
            print(f"    Repulsion fall : power^{la.repulsion_falloff}")
            print(f"    LPF alpha      : {la.smoothing_alpha}")
            print(f"    Yellow thresh  : hit_frac < {la.yellow_threshold}")
            print(f"    Red threshold  : hit_frac < {la.red_threshold}")
            print(f"    Debug rays     : {'YES (green/yellow/red spheres)' if la.debug_rays else 'NO'}")
        print("  -" * 32)
        print("  DEBUG FEATURES ACTIVE:")
        print("    [T1] Smooth chase-cam    (key 2 to activate)")
        print("    [T2] Blue path trail     (every 10 steps)")
        print("    [T3] Improved WP markers (larger, emissive)")
        print("    [T4] Full debug HUD      (every 100 steps, speed/accel/yaw)")
        print("    [T5] ASCII minimap       (every 200 steps)")
        print("    [T6] Safety rings        (translucent red, safety_inflation radius)")
        print("    [T7] Local avoidance rays (8 colored spheres: green/yellow/red)")
        print("=" * 64 + "\n")


    # -- Navigation logic -------------------------------------------------

    @staticmethod
    def _dist3(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    @staticmethod
    def _dist2(a, b):
        """Horizontal distance (XY plane only)."""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _init_astar(self):
        """
        Run A* once (on first NAVIGATE step) to generate the waypoint list.
        Called lazily so the Webots scene is fully loaded before we query it.
        Includes startup angle sanity check (Task 2): if WP[0] is "behind" the
        drone relative to the overall mission direction, it is skipped to prevent
        the startup jitter caused by an immediate 180-degree heading reversal.
        """
        print("\n[A*] Initialising path planner...")
        planner = AStarPlanner(
            buildings        = self.test_buildings,
            resolution       = self.grid_resolution,
            grid_margin      = self.grid_margin,
            obstacle_inflation = self.obstacle_inflation,
        )
        start_xy = (self.start_pos[0], self.start_pos[1])
        
        # Define lawnmower sweep key waypoints covering the full 200m x 200m world along street centerlines
        sweep_targets = [
            [-90.0, -80.0],
            [90.0, -80.0],
            [90.0, -40.0],
            [-90.0, -40.0],
            [-90.0, 0.0],
            [90.0, 0.0],
            [90.0, 40.0],
            [-90.0, 40.0],
            [-90.0, 80.0],
            [90.0, 80.0],
            [self.target_pos[0], self.target_pos[1]]
        ]

        print(f"[A*] Planning lawnmower sweep through checkpoints: {sweep_targets}")
        
        raw_wps = []
        current_start = start_xy
        for idx, target in enumerate(sweep_targets):
            segment = planner.plan(current_start, target)
            if segment:
                raw_wps.extend(segment)
                current_start = target
            else:
                print(f"[A*] Warning: leg to {target} failed planning  -  using straight-line fallback")
                raw_wps.append(target)
                current_start = target

        # Convert (x, y) -> [x, y, z] with mission altitude
        self.waypoints = [[wx, wy, self.altitude] for wx, wy in raw_wps]
        self.current_wp_idx = 0

        # -- Task 2: Startup angle sanity check -------------------------------
        # Compare angle (start->WP0) vs (start->first sweep target).
        # If WP0 is more than 90deg off the first sweep direction, skip it.
        # This prevents the drone doing a startup U-turn that causes jitter.
        if len(self.waypoints) > 1:
            sx, sy = self.start_pos[0], self.start_pos[1]
            gx, gy = sweep_targets[0][0], sweep_targets[0][1]
            w0x, w0y = self.waypoints[0][0], self.waypoints[0][1]

            # Direction vectors
            to_goal_x, to_goal_y = gx - sx, gy - sy
            to_wp0_x,  to_wp0_y  = w0x - sx, w0y - sy

            goal_len = math.hypot(to_goal_x, to_goal_y)
            wp0_len  = math.hypot(to_wp0_x,  to_wp0_y)

            if goal_len > 1e-4 and wp0_len > 1e-4:
                dot = (to_goal_x * to_wp0_x + to_goal_y * to_wp0_y) / (goal_len * wp0_len)
                dot = max(-1.0, min(1.0, dot))   # clamp for acos safety
                angle_deg = math.degrees(math.acos(dot))
                if angle_deg > 90.0:
                    print(f"[A*] Skipping unstable first waypoint "
                          f"(angle to first sweep target: {angle_deg:.1f}deg > 90deg)")
                    self.waypoints.pop(0)
                else:
                    print(f"[A*] First waypoint OK (angle to first sweep target: {angle_deg:.1f}deg)")

        print(f"[A*] Waypoint plan ({len(self.waypoints)} waypoints):")
        for i, wp in enumerate(self.waypoints):
            marker = " <- FINAL TARGET" if i == len(self.waypoints) - 1 else ""
            print(f"  WP[{i:02d}] ({wp[0]:7.2f}, {wp[1]:7.2f}, {wp[2]:.1f}){marker}")
        print()

        # -- Collision Safety Layer: post-planning segment validation -----------
        # A* can occasionally produce smoothed segments that clip a building
        # corner at the grid-cell boundary.  Re-validate every consecutive WP
        # pair in world-space at the (larger) safety_inflation_radius.
        if self.safety_layer.enabled and len(self.waypoints) > 1:
            print("[CollisionSafety] Running post-plan segment validation...")
            fixed   = 0
            prev_wp = [self.start_pos[0], self.start_pos[1], self.altitude]
            for i in range(len(self.waypoints)):
                wp = self.waypoints[i]
                if not self.safety_layer.check_segment(prev_wp, wp):
                    fallback = self.safety_layer.nearest_safe_waypoint(prev_wp, wp)
                    print(f"  [CollisionSafety] Segment to WP[{i:02d}] BLOCKED  -  "
                          f"replaced with nearest-safe fallback "
                          f"({fallback[0]:.1f}, {fallback[1]:.1f})")
                    self.waypoints[i] = fallback
                    fixed += 1
                prev_wp = self.waypoints[i]
            if fixed == 0:
                print("  [CollisionSafety] All segments clear  -  no corrections needed.")
            else:
                print(f"  [CollisionSafety] Fixed {fixed} blocked segment(s).")

        # Spawn visual markers if requested
        if self.show_wp_markers:
            self._spawn_waypoint_markers()

        # Spawn debug beacon above drone (Task 1)
        if self.debug_mode:
            self._spawn_debug_beacon()

        # Spawn safety-inflation visualisation rings (CollisionSafety T6)
        self.safety_layer.spawn_inflation_visuals(self.supervisor)

        # Spawn local avoidance ray-tip spheres (LocalAvoidanceSensor debug)
        if self.local_avoidance.enabled:
            self.local_avoidance.spawn_ray_visuals(self.supervisor)

        self.astar_initialized = True

    def _spawn_waypoint_markers(self):
        """
        Spawn improved waypoint markers (Task 4):
          - Bigger spheres (radius 1.5 vs 1.0)
          - Bright yellow intermediate, bright green final
          - Strong emissive glow so markers are visible from far away
          - Floated +1 m above the waypoint altitude
          - Numbered console printout per waypoint
        Uses supervisor importMFNodeFromString  -  gracefully skips on any error.
        """
        print("[WP Markers] Spawning improved waypoint markers:")
        try:
            root = self.supervisor.getRoot()
            children_field = root.getField("children")
            total = len(self.waypoints)
            for i, wp in enumerate(self.waypoints):
                is_final = (i == total - 1)
                if is_final:
                    # Bright green  -  final target
                    r, g, b  = 0.0, 1.0, 0.15
                    er, eg, eb = 0.0, 0.85, 0.1
                    label = "FINAL TARGET"
                else:
                    # Bright yellow  -  intermediate waypoint
                    r, g, b  = 1.0, 1.0, 0.0
                    er, eg, eb = 0.9, 0.9, 0.0
                    label = "intermediate"

                # Float marker +1 m above its waypoint altitude
                marker_z = wp[2] + 1.0

                node_str = (
                    f'DEF ASTAR_WP_{i} Solid {{\n'
                    f'  translation {wp[0]} {wp[1]} {marker_z}\n'
                    f'  children [\n'
                    f'    Shape {{\n'
                    f'      appearance Appearance {{\n'
                    f'        material Material {{\n'
                    f'          diffuseColor {r} {g} {b}\n'
                    f'          emissiveColor {er} {eg} {eb}\n'
                    f'          shininess 0.9\n'
                    f'        }}\n'
                    f'      }}\n'
                    f'      geometry Sphere {{ radius 1.5 }}\n'
                    f'    }}\n'
                    f'  ]\n'
                    f'  name "astar_wp_{i}"\n'
                    f'  contactMaterial "default"\n'
                    f'  physics NULL\n'
                    f'}}\n'
                )
                children_field.importMFNodeFromString(-1, node_str)
                # Numbered debug print (Task 4 requirement)
                print(f"  WP[{i}]  ({wp[0]:7.2f}, {wp[1]:7.2f}, {wp[2]:.1f})  "
                      f"[{label}]")

            print(f"[WP Markers] {total} markers spawned  "
                  f"(yellow=intermediate, green=final).")
        except Exception as e:
            print(f"[WP Markers] Not spawned (non-fatal): {e}")

    # -- Task 1: Debug Beacon -----------------------------------------------

    def _spawn_debug_beacon(self):
        """No-op: visual debug beacon disabled by request."""
        pass

    def _update_debug_beacon(self, drone_pos):
        """No-op: visual debug beacon disabled by request."""
        pass

    # -- Task 3: Persistent Path Trail -------------------------------------

    def _update_path_trail(self, drone_pos):
        """
        Accumulate small blue spheres to show the actual flown path.
        Samples every _trail_interval steps. Caps at _trail_max_nodes.
        Older nodes are NOT removed (they form the persistent trail).
        """
        self._trail_counter += 1
        if self._trail_counter < self._trail_interval:
            return
        self._trail_counter = 0

        # Cap node count to avoid memory bloat
        if len(self._trail_nodes) >= self._trail_max_nodes:
            return

        idx = len(self._trail_nodes)
        x, y, z = drone_pos[0], drone_pos[1], drone_pos[2]

        node_str = (
            f'DEF TRAIL_{idx} Solid {{\n'
            f'  translation {x} {y} {z}\n'
            f'  children [\n'
            f'    Shape {{\n'
            f'      appearance Appearance {{\n'
            f'        material Material {{\n'
            f'          diffuseColor 0.0 0.4 1.0\n'
            f'          emissiveColor 0.0 0.35 0.9\n'
            f'          shininess 0.7\n'
            f'        }}\n'
            f'      }}\n'
            f'      geometry Sphere {{ radius {self._trail_radius} }}\n'
            f'    }}\n'
            f'  ]\n'
            f'  name "trail_{idx}"\n'
            f'  contactMaterial "default"\n'
            f'  physics NULL\n'
            f'}}\n'
        )
        try:
            root = self.supervisor.getRoot()
            children_field = root.getField("children")
            children_field.importMFNodeFromString(-1, node_str)
            self._trail_nodes.append(idx)   # just store count
        except Exception:
            pass   # trail spawn failure is silent  -  never block simulation

    # -- Task 6: ASCII Minimap ---------------------------------------------

    def _print_minimap(self, drone_pos):
        """
        Lightweight ASCII minimap printed to console every _minimap_interval steps.
        Grid: 21 cols x 10 rows representing the +/-90m world.
        Symbols:  D=drone  T=target  B=building  W=waypoint  .=open space
        """
        COLS = 21
        ROWS = 10
        WORLD = 90.0   # half-width

        def to_grid(wx, wy):
            c = int((wx + WORLD) / (2.0 * WORLD) * (COLS - 1))
            r = int((1.0 - (wy + WORLD) / (2.0 * WORLD)) * (ROWS - 1))
            c = max(0, min(COLS - 1, c))
            r = max(0, min(ROWS - 1, r))
            return c, r

        grid = [['.'] * COLS for _ in range(ROWS)]

        # Mark buildings
        for b in self.test_buildings:
            c, r = to_grid(b["x"], b["y"])
            grid[r][c] = 'B'

        # Mark A* waypoints
        for i, wp in enumerate(self.waypoints):
            c, r = to_grid(wp[0], wp[1])
            if grid[r][c] not in ('D', 'T'):
                grid[r][c] = 'W'

        # Mark target
        tc, tr = to_grid(self.target_pos[0], self.target_pos[1])
        grid[tr][tc] = 'T'

        # Mark drone (overrides everything)
        dc, dr = to_grid(drone_pos[0], drone_pos[1])
        grid[dr][dc] = 'D'

        # Render
        sep = '-' * (COLS * 2 + 1)
        print(f"\n+{sep}+")
        for row in grid:
            print('| ' + ' '.join(row) + ' |')
        print(f"+{sep}+")
        wp_idx = self.current_wp_idx + 1 if self.waypoints else 0
        total_wp = len(self.waypoints)
        print(f"  ASCII Minimap | D=drone  T=target  B=building  W=waypoint  "
              f"WP={wp_idx}/{total_wp}  "
              f"pos=({drone_pos[0]:.0f},{drone_pos[1]:.0f})")
        print()

    def _move_toward_target(self):
        """
        KINEMATIC FLIGHT MODEL with stabilisation layer.

        Algorithm (per step):
          1.  Select steering target (current A* waypoint or final target).
          2.  Compute raw desired heading toward target.
          3.  Obstacle repulsion  -  GATED: active only inside emergency_radius
              when A* is enabled (Task 3).  Full repulsion when A* disabled.
          4.  Low-pass filter desired_yaw (Task 4  -  yaw_smoothing_alpha).
          5.  Rotate current_yaw toward filtered desired_yaw by max_turn_rate.
          6.  Desired speed: cruise or decel ramp near WP.
          7.  Velocity damping inside wp_damping_radius (Task 5).
          8.  Accelerate vx/vy toward desired by max_acceleration.
          9.  Integrate position: x += vx, y += vy.
          10. Z kinematic hold.

        Stores telemetry in self.last_* for HUD display.
        Returns new [x, y, z] position.
        """
        cur = list(self.uav_tf.getSFVec3f())

        # -- 1. Select steering target (A* waypoint or final target) -------
        if self.astar_enabled and self.waypoints:
            wp = self.waypoints[self.current_wp_idx]
            tx, ty = wp[0], wp[1]
        else:
            tx, ty = self.target_pos[0], self.target_pos[1]

        dx, dy = tx - cur[0], ty - cur[1]
        horiz_dist = math.hypot(dx, dy)

        # Already on target  -  bleed off velocity and hold
        if horiz_dist < 1e-4:
            self.last_avoiding      = False
            self.last_steer_vec     = (0.0, 0.0)
            self.last_avoidance_mode= "DISABLED"
            self.last_motion_state  = "HOLD"
            self.vx *= 0.7
            self.vy *= 0.7
            self.vz *= 0.7
            self.last_velocity    = (self.vx, self.vy)
            self.last_speed       = math.hypot(self.vx, self.vy)
            self.last_accel       = (0.0, 0.0)
            self.last_accel_mag   = 0.0
            self.last_turn_rate   = 0.0
            return [cur[0] + self.vx, cur[1] + self.vy, self.altitude]

        # -- 2. Raw desired heading toward target -------------------------
        raw_desired_yaw = math.atan2(dy, dx)

        # -- 3. Avoidance: two-tier system ------------------------------------
        #
        #   Tier 1 (PRIMARY): LocalAvoidanceSensor  -  8 drone-centered rays.
        #     Active whenever rays detect obstacles.  Blends A* direction with
        #     avoidance direction using configured weights.  Smooth LPF output.
        #
        #   Tier 2 (FAIL-SAFE): CollisionSafetyLayer  -  building-surface distance.
        #     Active only when drone is dangerously close (WARNING/EMERGENCY).
        #     Retained as last-resort protection if local sensing misses something.
        #
        # Both layers can fire simultaneously; their contributions are additive.
        # A* waypoint target vector is NEVER removed  -  only blended/steered.

        self.last_avoiding   = False
        avoidance_mode       = "DISABLED"

        # -- Stuck detection --------------------------------------------------
        if self._stuck_cooldown_counter > 0:
            self._stuck_cooldown_counter -= 1

        self._pos_history.append(list(cur))
        if len(self._pos_history) > 80:
            old_pos = self._pos_history.pop(0)
            if (self._stuck_recovery_steps == 0 and 
                self._stuck_cooldown_counter == 0 and 
                self._dist3(cur, old_pos) < 2.0 and 
                self.local_avoidance.is_active()):
                print("[STUCK_RECOVERY] Entering sidestep mode")
                self._stuck_recovery_steps = 40

        if self._stuck_recovery_steps > 0:
            self._stuck_recovery_steps -= 1
            if self._stuck_recovery_steps == 0:
                print("[STUCK_RECOVERY] Exit")
                self._stuck_cooldown_counter = 120

        # -- Tier 1: Local Avoidance Sensor (primary) -------------------------
        la = self.local_avoidance
        if la.enabled:
            # update() already called in step() before _move_toward_target()
            la_vec = la.get_avoidance_vector()   # (ax, ay) LPF'd, normalised
            la_mag = math.hypot(la_vec[0], la_vec[1])

            # Apply Anti-Spin modifications
            eff_avoidance_weight = la.avoidance_weight
            if self._anti_spin_active_steps > 0:
                eff_avoidance_weight *= 0.5
            # -- [FIX-3] Sensor-level oscillation auto-damping -----------------
            # Complements yaw-based anti-spin: if the avoidance VECTOR itself is
            # flip-flopping (detected in LocalAvoidanceSensor.update), halve its
            # authority BEFORE the heading blend, preventing heading oscillation
            # from propagating upward to the yaw controller.
            if la._oscillation_detected:
                eff_avoidance_weight *= 0.5

            # If stuck recovery is active, rotate the avoidance vector by +/- 90 degrees
            if self._stuck_recovery_steps > 0 and la_mag > 1e-4:
                # Rotate avoidance vector by +/-90 degrees based on free space
                forward_ux = math.cos(self.current_yaw)
                forward_uy = math.sin(self.current_yaw)
                left_ux = -math.sin(self.current_yaw)
                left_uy = math.cos(self.current_yaw)

                # Left side score (rays 1, 2, 3)
                left_score = (la._ray_results[1]["hit_fraction"] +
                              la._ray_results[2]["hit_fraction"] +
                              la._ray_results[3]["hit_fraction"])

                # Right side score (rays 5, 6, 7)
                right_score = (la._ray_results[7]["hit_fraction"] +
                               la._ray_results[6]["hit_fraction"] +
                               la._ray_results[5]["hit_fraction"])

                # Candidates: v_ccw (+90 deg) and v_cw (-90 deg)
                # v_ccw = (-ay, ax)
                v_ccw = (-la_vec[1], la_vec[0])
                # v_cw = (ay, -ax)
                v_cw = (la_vec[1], -la_vec[0])

                # Projections
                proj_left_ccw = v_ccw[0] * left_ux + v_ccw[1] * left_uy
                proj_fwd_ccw = v_ccw[0] * forward_ux + v_ccw[1] * forward_uy

                proj_left_cw = v_cw[0] * left_ux + v_cw[1] * left_uy
                proj_fwd_cw = v_cw[0] * forward_ux + v_cw[1] * forward_uy

                # Selection scores with a forward bias of 1.0
                diff_score = left_score - right_score
                score_ccw = diff_score * proj_left_ccw + 1.0 * proj_fwd_ccw
                score_cw = diff_score * proj_left_cw + 1.0 * proj_fwd_cw

                if score_ccw >= score_cw:
                    la_vec = v_ccw
                else:
                    la_vec = v_cw

            if la_mag > 1e-4 and eff_avoidance_weight > 0.0:
                # A* unit direction toward current waypoint / target
                astar_ux = dx / horiz_dist
                astar_uy = dy / horiz_dist
                
                # Clamp avoidance steering magnitude so it only slightly biases heading
                # Note: with astar_weight=0.75, avoid_weight=0.25 it's already capped,
                # but we explicitly clamp the local avoidance contribution here.
                clamped_la_x = la_vec[0] * eff_avoidance_weight
                clamped_la_y = la_vec[1] * eff_avoidance_weight
                
                # Blend: astar_weight x A* + avoidance
                blend_x = astar_ux * la.astar_weight + clamped_la_x
                blend_y = astar_uy * la.astar_weight + clamped_la_y
                bl = math.hypot(blend_x, blend_y)
                if bl > 1e-4:
                    raw_desired_yaw = math.atan2(blend_y / bl, blend_x / bl)
                self.last_avoiding = True
                avoidance_mode     = "LOCAL_SENSOR"
            self._last_la_info = la.get_debug_info()
        else:
            self._last_la_info = {}

        # -- Tier 2: CollisionSafetyLayer (emergency fail-safe only) ----------
        # This was the old primary avoidance.  Now it fires ONLY when the drone
        # is already very close to a building (WARNING / EMERGENCY state), acting
        # as a last-resort correction if the local sensor blending was insufficient.
        sl          = self.safety_layer
        coll_state  = sl._last_state           # cached from step()'s pre-move call
        rep_vec     = sl.compute_repulsion(cur)# (rx, ry) filtered, normalised
        rep_str     = sl.get_repulsion_strength()  # 0 / warn_str / emrg_str

        if sl.enabled and rep_str > 0.0:
            self.last_avoiding = True
            rx, ry = rep_vec
            # Additive emergency correction on top of whatever heading we have now
            # (re-use the current raw_desired_yaw direction as the "target" component)
            cur_dx = math.cos(raw_desired_yaw)
            cur_dy = math.sin(raw_desired_yaw)
            blend_x = cur_dx * self.target_strength + rx * rep_str
            blend_y = cur_dy * self.target_strength + ry * rep_str
            bl = math.hypot(blend_x, blend_y)
            if bl > 1e-4:
                raw_desired_yaw = math.atan2(blend_y / bl, blend_x / bl)
            # Tag the state to show both sensors fired
            if avoidance_mode == "LOCAL_SENSOR":
                avoidance_mode = f"LOCAL+{coll_state}"
            else:
                avoidance_mode = coll_state   # "WARNING" or "EMERGENCY"
        elif not sl.enabled:
            # Legacy fallback (CollisionSafetyLayer disabled entirely)
            closest_name_legacy, min_dist_legacy, bx_l, by_l = \
                self._check_collision_distance(cur)
            if self.astar_enabled:
                avoidance_active_l = (closest_name_legacy and
                                      min_dist_legacy < self.emergency_radius)
                if avoidance_active_l and avoidance_mode == "DISABLED":
                    avoidance_mode = "LEGACY_EMRG"
            else:
                avoidance_active_l = (closest_name_legacy and
                                      min_dist_legacy < self.safety_radius)
                if avoidance_active_l and avoidance_mode == "DISABLED":
                    avoidance_mode = "LEGACY_FULL"
            if avoidance_active_l:
                self.last_avoiding = True
                rx = cur[0] - bx_l
                ry = cur[1] - by_l
                r_dist = math.hypot(rx, ry)
                if r_dist > 1e-4:
                    blend_x = (dx / horiz_dist) * self.target_strength + \
                              (rx / r_dist) * self.avoidance_strength
                    blend_y = (dy / horiz_dist) * self.target_strength + \
                              (ry / r_dist) * self.avoidance_strength
                    bl = math.hypot(blend_x, blend_y)
                    if bl > 1e-4:
                        raw_desired_yaw = math.atan2(blend_y / bl, blend_x / bl)

        self.last_avoidance_mode = avoidance_mode


        # -- 4. Low-pass filter on desired_yaw (Task 4) -------------------
        # Initialise filter state on first step from current heading
        if self._filtered_desired_yaw is None:
            self._filtered_desired_yaw = raw_desired_yaw

        # Wrap difference so filter interpolates the short way around the circle
        yaw_diff = raw_desired_yaw - self._filtered_desired_yaw
        while yaw_diff >  math.pi: yaw_diff -= 2.0 * math.pi
        while yaw_diff < -math.pi: yaw_diff += 2.0 * math.pi
        self._filtered_desired_yaw += self.yaw_smoothing_alpha * yaw_diff
        # Keep filtered yaw in (-pi, pi]
        while self._filtered_desired_yaw >  math.pi: self._filtered_desired_yaw -= 2.0 * math.pi
        while self._filtered_desired_yaw < -math.pi: self._filtered_desired_yaw += 2.0 * math.pi

        desired_yaw = self._filtered_desired_yaw

        # -- 5. Smooth yaw toward filtered desired_yaw (max_turn_rate per step) -
        delta_yaw = desired_yaw - self.current_yaw
        while delta_yaw >  math.pi: delta_yaw -= 2.0 * math.pi
        while delta_yaw < -math.pi: delta_yaw += 2.0 * math.pi
        turn = max(-self.max_turn_rate, min(self.max_turn_rate, delta_yaw))
        
        # Anti-spin protection logic
        turn_sign = 1 if turn > 1e-3 else (-1 if turn < -1e-3 else 0)
        if turn_sign != 0:
            if turn_sign != self._last_turn_sign:
                self._oscillation_count += 1
                self._last_turn_sign = turn_sign
            else:
                # decay count if turning consistently in one direction
                self._oscillation_count = max(0.0, self._oscillation_count - 0.1)

        if self._oscillation_count > 20:
            print("[ANTI-SPIN] Drone oscillation detected. Reducing avoidance authority.")
            self._anti_spin_active_steps = 40
            self._oscillation_count = 0.0

        if self._anti_spin_active_steps > 0:
            self._anti_spin_active_steps -= 1

        self.current_yaw += turn
        self.last_turn_rate = turn
        while self.current_yaw >  math.pi: self.current_yaw -= 2.0 * math.pi
        while self.current_yaw < -math.pi: self.current_yaw += 2.0 * math.pi

        # -- 6. Desired speed with deceleration ramp near waypoint ---------
        motion_state = "CRUISE"
        if horiz_dist < self.decel_radius:
            ramp_frac = max(0.05, horiz_dist / self.decel_radius)
            desired_speed = self.cruise_velocity * ramp_frac
            motion_state = "DECEL"
        else:
            desired_speed = self.cruise_velocity
        desired_speed = min(desired_speed, self.max_velocity)

        # -- [FIX-4] Near-wall speed reduction --------------------------------
        # Scale down desired_speed when the sensor reports a close obstacle.
        # This gives local avoidance more frames to react before wall contact.
        # Thresholds + scales are YAML-configurable via la.slow_*
        # Does NOT interfere with decel ramp  -  applied multiplicatively after.
        if la.enabled:
            nearest_obs_m = self._last_la_info.get("nearest_dist_m", float("inf"))
            s_start = la.slow_start_distance   # default 8 m
            s_mid   = la.slow_mid_distance     # default 5 m
            s_near  = la.slow_near_distance    # default 3 m
            if nearest_obs_m < s_near:
                speed_scale = la.emergency_slow_scale   # default 0.45
            elif nearest_obs_m < s_mid:
                t = (nearest_obs_m - s_near) / max(s_mid - s_near, 1e-4)
                speed_scale = la.slow_near_scale + (la.slow_mid_scale - la.slow_near_scale) * t
            elif nearest_obs_m < s_start:
                t = (nearest_obs_m - s_mid) / max(s_start - s_mid, 1e-4)
                speed_scale = la.slow_mid_scale + (1.0 - la.slow_mid_scale) * t
            else:
                speed_scale = 1.0
            desired_speed *= speed_scale
            desired_speed = min(desired_speed, self.max_velocity)

        # -- 7. Velocity damping near waypoint to kill oscillation (Task 5) --
        if horiz_dist < self.wp_damping_radius:
            self.vx *= self.velocity_damping_factor
            self.vy *= self.velocity_damping_factor
            motion_state = "DAMP"

        self.last_motion_state = motion_state

        # -- 8. Desired velocity vector from heading + speed ---------------
        desired_vx = math.cos(self.current_yaw) * desired_speed
        desired_vy = math.sin(self.current_yaw) * desired_speed

        # Accelerate toward desired velocity (bounded by max_accel)
        prev_vx, prev_vy = self.vx, self.vy
        ax = desired_vx - self.vx
        ay = desired_vy - self.vy
        a_mag = math.hypot(ax, ay)
        if a_mag > self.max_acceleration:
            scale_a = self.max_acceleration / a_mag
            ax *= scale_a
            ay *= scale_a
        self.vx += ax
        self.vy += ay

        # Hard clamp to max_velocity
        v_mag = math.hypot(self.vx, self.vy)
        if v_mag > self.max_velocity:
            fac = self.max_velocity / v_mag
            self.vx *= fac
            self.vy *= fac

        # -- 9. Integrate XY position --------------------------------------
        new_x = cur[0] + self.vx
        new_y = cur[1] + self.vy

        # -- 10. Kinematic Z hold (no PID, no motor thrust) ----------------
        # Hard-clamp altitude to prevent runaway climb from propeller physics.
        # vz integrator drives toward mission altitude, but new_z is then
        # clamped to [0, self.altitude] as a safety guarantee.
        z_error = self.altitude - cur[2]
        desired_vz = max(-self.max_acceleration * 2,
                         min(self.max_acceleration * 2, z_error * 0.3))
        dz = desired_vz - self.vz
        dz_clamped = max(-self.max_acceleration * 0.5,
                         min(self.max_acceleration * 0.5, dz))
        self.vz += dz_clamped
        new_z = cur[2] + self.vz
        # HARD CLAMP: never allow altitude to exceed mission altitude.
        # This prevents propeller thrust accumulation from driving the drone skyward.
        new_z = max(0.5, min(new_z, self.altitude))
        # Anti-windup: if we hit the ceiling, zero out vz so we don't bounce.
        if new_z >= self.altitude:
            self.vz = 0.0
        elif new_z <= 0.5:
            self.vz = 0.0

        # -- Store HUD telemetry -------------------------------------------
        heading_unit = (math.cos(self.current_yaw), math.sin(self.current_yaw))
        self.last_steer_vec      = heading_unit
        self.last_velocity       = (self.vx, self.vy)
        self.last_speed          = math.hypot(self.vx, self.vy)
        self.last_accel          = (ax, ay)
        self.last_accel_mag      = math.hypot(ax, ay)
        self.last_desired_yaw    = raw_desired_yaw
        self.last_filtered_yaw   = desired_yaw
        # Heading error: signed angle between current heading and filtered target
        herr = desired_yaw - self.current_yaw
        while herr >  math.pi: herr -= 2.0 * math.pi
        while herr < -math.pi: herr += 2.0 * math.pi
        self.last_heading_error_deg = math.degrees(herr)

        return [new_x, new_y, new_z]

    def _orient_toward(self, prev_pos, new_pos):
        """
        Apply the kinematic model's already-smoothed yaw to the UAV rotation field.

        The kinematic engine in _move_toward_target() owns heading via
        self.current_yaw (bounded per step by max_turn_rate), so we simply
        write that value instead of deriving an instant angle from displacement.
        This guarantees orientation and velocity are always consistent and that
        no instant heading flips can occur regardless of prev/new positions.
        """
        # Use the smooth kinematic yaw  -  prev/new_pos kept as parameters for
        # API compatibility in case callers change in future.
        self.uav_rf.setSFRotation([0.0, 0.0, 1.0, self.current_yaw])

    # -- Per-step update ---------------------------------------------------

    # -- Per-step update ---------------------------------------------------

    def step(self):
        """
        Called once per simulation step.
        Returns True while the mission is running; False when ARRIVED
        (caller can then choose to keep the loop alive for visual inspection).
        """
        self.step_count += 1

        # Suppress UAV_1-4 every step (keeps propeller thrust from lifting them)
        self._suppress_inactive_uavs()

        cur = list(self.uav_tf.getSFVec3f())

        # Initialize metrics on step 1
        if self.step_count == 1:
            self.sim_start_time = self.supervisor.getTime()
            self._prev_pos = list(cur)

        # Accumulate distance travelled
        if self._prev_pos is not None:
            self.distance_travelled += self._dist3(cur, self._prev_pos)
        self._prev_pos = list(cur)

        # Record altitude
        self.altitude_readings.append(cur[2])

        # -- Calculate TRR (Target Recognition Rate) mathematically --
        crowd_positions = []
        for node in self.parent.crowd_nodes:
            try:
                crowd_positions.append(node.getPosition())
            except Exception:
                pass

        tracked = set()
        for ci, c_pos in enumerate(crowd_positions):
            dist = math.hypot(cur[0] - c_pos[0], cur[1] - c_pos[1])
            view_r = cur[2] * 0.7  # Z (altitude) based footprint
            if dist <= view_r:
                tracked.add(ci)

        self.last_trr = len(tracked) / len(crowd_positions) * 100.0 if crowd_positions else 0.0
        self.last_tracked_count = len(tracked)
        self.trr_readings.append(self.last_trr)

        # -- STATE: GROUND_IDLE -----------------------------------------
        if self.state == self.STATE_GROUND_IDLE:
            if self._spinup_counter == 0:
                # Place at start pos but on the ground
                self.uav_tf.setSFVec3f([self.start_pos[0], self.start_pos[1], self.ground_spawn_z])
                self.uav_rf.setSFRotation([0.0, 0.0, 1.0, 0.0])  # face East
                try:
                    self.uav_node.resetPhysics()
                except Exception:
                    pass
                print(f"[SingleDroneNav] STATE: GROUND_IDLE (spin-up {self.spinup_steps} steps)")
                print(f"  Target               : {self.target_pos}")
                total_dist = self._dist3(self.start_pos, self.target_pos)
                print(f"  Total mission dist   : {total_dist:.1f} m")
                
            self._spinup_counter += 1
            if self._spinup_counter >= self.spinup_steps:
                self.state = self.STATE_ASCEND
                print(f"[SingleDroneNav] STATE: ASCEND (climbing to {self.altitude}m)")
            return True

        # -- STATE: ASCEND ----------------------------------------------
        if self.state == self.STATE_ASCEND:
            # Climb smoothly from ground_spawn_z to mission_altitude
            new_z = min(cur[2] + self.takeoff_speed, self.altitude)
            
            self.uav_tf.setSFVec3f([cur[0], cur[1], new_z])
            try:
                self.uav_node.resetPhysics()
            except Exception:
                pass

            if cur[2] >= self.altitude - 0.5:
                print(f"[SingleDroneNav] Reached mission altitude. Transitioning to NAVIGATE.")
                self.state = self.STATE_NAVIGATE
                
            return True

        # -- STATE: NAVIGATE -------------------------------------------
        if self.state == self.STATE_NAVIGATE:

            # Lazy A* init  -  runs once on the very first NAVIGATE step
            if self.astar_enabled and not self.astar_initialized:
                self._init_astar()

            dist_to_target = self._dist3(cur, self.target_pos)

            if dist_to_target <= self.arrival_radius:
                # Mission complete  -  snap to current position, hold
                self.uav_tf.setSFVec3f(cur)
                self.state = self.STATE_ARRIVED
                return True

            # -- A* waypoint advance with hysteresis lock (Task 1) --------
            if self.astar_enabled and self.waypoints:
                wp = self.waypoints[self.current_wp_idx]
                dist_to_wp = self._dist2(cur, wp)

                # Decrement hysteresis lock counter each step
                if self._wp_lock_counter > 0:
                    self._wp_lock_counter -= 1
                    if self._wp_lock_counter == 0:
                        print(f"  [WP_LOCK] Released  -  WP[{self.current_wp_idx:02d}] "
                              f"now active  step={self.step_count}")

                # Release lock early if we are already very close to the waypoint to avoid orbiting/spinning
                if dist_to_wp <= self.waypoint_radius * 0.5:
                    self._wp_lock_counter = 0

                # Only advance when lock is not active
                if dist_to_wp <= self.waypoint_radius and self._wp_lock_counter == 0:
                    prev_idx = self.current_wp_idx
                    if self.current_wp_idx < len(self.waypoints) - 1:
                        candidate_idx = self.current_wp_idx + 1
                        candidate_wp  = self.waypoints[candidate_idx]

                        # -- Safety layer segment check before advancing ------
                        # Verify the segment cur -> candidate_wp is clear at the
                        # safety_inflation_radius BEFORE committing to the advance.
                        seg_clear = self.safety_layer.check_segment(cur, candidate_wp)
                        if not seg_clear:
                            fallback = self.safety_layer.nearest_safe_waypoint(
                                cur, candidate_wp
                            )
                            print(f"  [CollisionSafety] WP advance to WP[{candidate_idx:02d}] "
                                  f"BLOCKED  -  replacing with nearest-safe fallback "
                                  f"({fallback[0]:.1f}, {fallback[1]:.1f})  "
                                  f"step={self.step_count}")
                            # Replace candidate with fallback to avoid feedback loop insertion
                            self.waypoints[candidate_idx] = fallback
                            candidate_wp = fallback

                        self.current_wp_idx += 1
                        new_wp = self.waypoints[self.current_wp_idx]
                        self._wp_lock_counter = self.waypoint_reach_lock_steps
                        print(f"  [A*] WP[{prev_idx:02d}] reached -> advancing to "
                              f"WP[{self.current_wp_idx:02d}] "
                              f"({new_wp[0]:.1f}, {new_wp[1]:.1f})  "
                              f"step={self.step_count}")
                        print(f"  [WP_LOCK] Active for {self._wp_lock_counter} steps")
                    else:
                        # Reached the final waypoint in the list!
                        print(f"  [A*] Reached final waypoint WP[{self.current_wp_idx:02d}]  -  declaring mission complete. step={self.step_count}")
                        self.state = self.STATE_ARRIVED
                        return True

            prev_pos = list(cur)

            # -- Pre-movement sensor updates -------------------------------
            # Both sensors must update from the CURRENT position before
            # _move_toward_target() reads their cached state.

            # LocalAvoidanceSensor: cast 8 rays, compute blended avoidance vec
            if self.local_avoidance.enabled:
                self.local_avoidance.update(cur, self.current_yaw, self.vx, self.vy)

            # CollisionSafetyLayer: update building-distance state machine
            self.safety_layer.get_collision_state(cur)

            new_pos  = self._move_toward_target()
            self.uav_tf.setSFVec3f(new_pos)
            self._orient_toward(prev_pos, new_pos)
            # FIX-2: reset physics AFTER every position update.
            # This zeroes the ODE velocity buffer so propeller thrust from
            # uav_camera.py cannot accumulate altitude between steps.
            # The drone's Z will always be exactly self.altitude next read.
            try:
                self.uav_node.resetPhysics()
            except Exception:
                pass

            # -- DEBUG OBSERVABILITY UPDATES -------------------------------
            if self.debug_mode:
                # Task 1: move beacon above drone
                self._update_debug_beacon(new_pos)
                # Task 3: add trail dot
                self._update_path_trail(new_pos)
                # LocalAvoidanceSensor: move & recolor ray-tip spheres
                if self.local_avoidance.enabled and self.local_avoidance.debug_rays:
                    self.local_avoidance.update_ray_visuals(self.supervisor, new_pos)



            # Task 6: ASCII minimap every N steps
            if self.debug_mode and self.step_count % self._minimap_interval == 0:
                self._print_minimap(new_pos)
            # -------------------------------------------------------------

            # Update grid coverage tracking
            cell_x = int(new_pos[0] // self.grid_cell_size)
            cell_y = int(new_pos[1] // self.grid_cell_size)
            self.visited_grid_cells.add((cell_x, cell_y))

            # Direct physical collision check using Webots contact points (independent of map!)
            has_physics_contact = False
            try:
                contacts = self.uav_node.getContactPoints()
                if len(contacts) > 0:
                    has_physics_contact = True
                    self.physics_contact_count += 1
            except Exception:
                pass

            # -- Safety layer: update state + collision telemetry ----------
            # get_collision_state() caches result internally for HUD.
            safety_state = self.safety_layer.get_collision_state(new_pos)
            self._last_safety_info = self.safety_layer.get_debug_info()

            # Legacy lightweight collision check (kept for metrics logging)
            closest_name, min_dist, _, _ = self._check_collision_distance(cur)
            if min_dist < self.safety_radius:
                self.collision_count += 1  # Accumulate proximity warning steps
            if min_dist < 0.0:
                self.hard_collision_count += 1  # Accumulate actual building interpenetration steps

            if safety_state == CollisionSafetyLayer.EMERGENCY and \
               (self.step_count - self.last_warning_step) > 20:
                print(f"  [COLLISION EMERGENCY] Drone dangerously close to "
                      f"{self._last_safety_info['nearest_name']} "
                      f"(surface dist: {self._last_safety_info['surface_dist']:.1f}m)")
                self.last_warning_step = self.step_count
            elif safety_state == CollisionSafetyLayer.WARNING and \
                 (self.step_count - self.last_warning_step) > 50:
                print(f"  [COLLISION WARNING] Near {self._last_safety_info['nearest_name']} "
                      f"({self._last_safety_info['surface_dist']:.1f}m surface dist)")
                self.last_warning_step = self.step_count
            elif min_dist < 0.0 and (self.step_count - self.last_warning_step) > 20:
                print(f"  [CRITICAL] Drone is INSIDE building {closest_name}!")
                self.last_warning_step = self.step_count

            # Sparse step logging for metrics JSON
            if self.step_count % 10 == 0:
                self.step_log.append({
                    "step": self.step_count,
                    "x": round(new_pos[0], 2),
                    "y": round(new_pos[1], 2),
                    "z": round(new_pos[2], 2),
                    "state": self.state,
                    "trr_percent": round(self.last_trr, 2),
                    "tracked_count": self.last_tracked_count,
                    "proximity_warning": int(min_dist < self.safety_radius)
                })

            # Task 5: Periodic formatted debug HUD
            if self.step_count % self.log_interval == 0:
                cur_after = list(self.uav_tf.getSFVec3f())
                self._log_status(cur_after, dist_to_target, closest_name, min_dist)
                # TEMPORARY LOGGING FOR STUCK DEBUGGING
                with open("stuck_debug.log", "a", encoding="utf-8") as f:
                    f.write(f"Step {self.step_count}: Pos ({cur_after[0]:.1f}, {cur_after[1]:.1f}, {cur_after[2]:.1f}) "
                            f"TargetDist={dist_to_target:.1f} Nearest={closest_name} ({min_dist:.1f}m) "
                            f"AvoidMode={self.last_avoidance_mode} State={self.last_motion_state} "
                            f"Speed={self.last_speed:.3f}\n")

            return True

        # -- STATE: ARRIVED -------------------------------------------
        if self.state == self.STATE_ARRIVED:
            if not self.arrived_reported:
                self.reached_target = True
                cur = list(self.uav_tf.getSFVec3f())
                print("\n" + "=" * 64)
                print("  (OK) MISSION COMPLETE  -  UAV_0 ARRIVED AT TARGET")
                print(f"  Final position : ({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})")
                print(f"  Target         : {self.target_pos}")
                print(f"  Steps taken    : {self.step_count}")
                print(f"  State          : ARRIVED (drone holding position)")
                print("=" * 64 + "\n")
                
                # Save metrics to JSON file
                self._save_metrics()
                self.arrived_reported = True
                
                if os.environ.get("WEBOTS_HEADLESS", "false").lower() == "true":
                    print("[HEADLESS] Exiting Webots automatically.")
                    self.supervisor.simulationQuit(0)
                    return True
                
            # Hold position  -  reset physics every step so propellers
            # don't push the drone away from the target (FIX-2).
            self.uav_tf.setSFVec3f(cur)
            try:
                self.uav_node.resetPhysics()
            except Exception:
                pass
            return True

        return True  # fallback

    def _save_metrics(self):
        """Save metrics JSON to experiments/single_drone/<timestamp>/metrics.json."""
        import json
        import time as _time
        
        travel_time = self.supervisor.getTime() - (self.sim_start_time or 0.0)
        
        # Altitude std-dev
        alt_mean = sum(self.altitude_readings) / len(self.altitude_readings) if self.altitude_readings else self.altitude
        alt_var = sum((z - alt_mean)**2 for z in self.altitude_readings) / len(self.altitude_readings) if self.altitude_readings else 0.0
        alt_std = math.sqrt(alt_var)
        
        # Mean TRR
        mean_trr = sum(self.trr_readings) / len(self.trr_readings) if self.trr_readings else 0.0
        
        # Cumulative coverage
        cumulative_coverage = len(self.visited_grid_cells) / self.total_grid_cells * 100.0 if self.total_grid_cells else 0.0

        metrics = {
            "phase": "Phase 1  -  Single Drone Lawnmower Patrol & Surveillance Baseline",
            "world": "worlds/single_drone_downtown.wbt",
            "start_pos": self.start_pos,
            "target_pos": self.target_pos,
            "reached_target": self.reached_target,
            "travel_time_s": round(travel_time, 3),
            "distance_travelled_m": round(self.distance_travelled, 3),
            "proximity_events": self.collision_count,
            "hard_collisions": self.hard_collision_count,
            "physics_contacts": self.physics_contact_count,
            "altitude_stability_std_m": round(alt_std, 5),
            "mean_trr_percent": round(mean_trr, 2),
            "cumulative_coverage_percent": round(cumulative_coverage, 2),
            "total_steps": self.step_count,
            "step_log": self.step_log
        }
        
        # Output path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
        timestamp = _time.strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.join(root_dir, "experiments", "single_drone", timestamp)
        os.makedirs(out_dir, exist_ok=True)
        
        out_path = os.path.join(out_dir, "metrics.json")
        with open(out_path, "w") as fp:
            json.dump(metrics, fp, indent=4)
            
        print(f"[SingleDrone] Metrics saved successfully -> {out_path}")

    def _check_collision_distance(self, pos):
        """Returns (closest_building_name, surface_distance, b_x, b_y)."""
        min_dist = 999.0
        closest = None
        bx, by = 0.0, 0.0
        for b in self.test_buildings:
            dist = math.hypot(pos[0] - b["x"], pos[1] - b["y"])
            surface_dist = dist - b["r"]
            if surface_dist < min_dist:
                min_dist = surface_dist
                closest = b["name"]
                bx, by = b["x"], b["y"]
        return closest, min_dist, bx, by

    def _log_status(self, pos, dist_to_target, closest_bldg=None, bldg_dist=999.0):
        """
        Task 5/6: Formatted debug HUD printed every log_interval steps.
        Extended with stabilisation metrics: heading error, yaw target,
        filtered yaw, avoidance mode, motion state.
        """
        planner_str     = "A*" if self.astar_enabled else "Reactive-only"
        vel             = getattr(self, 'last_velocity',         (0.0, 0.0))
        spd             = getattr(self, 'last_speed',            0.0)
        acc             = getattr(self, 'last_accel',            (0.0, 0.0))
        acc_mag         = getattr(self, 'last_accel_mag',        0.0)
        turn            = getattr(self, 'last_turn_rate',        0.0)
        yaw_deg         = math.degrees(self.current_yaw)
        raw_yaw_deg     = math.degrees(getattr(self, 'last_desired_yaw',    0.0))
        filt_yaw_deg    = math.degrees(getattr(self, 'last_filtered_yaw',   0.0))
        herr_deg        = getattr(self, 'last_heading_error_deg', 0.0)
        avoid_mode      = getattr(self, 'last_avoidance_mode',    'DISABLED')
        motion_state    = getattr(self, 'last_motion_state',      'CRUISE')
        wp_locked       = self._wp_lock_counter > 0

        # Waypoint info
        if self.astar_enabled and self.waypoints:
            wp          = self.waypoints[self.current_wp_idx]
            wp_idx_str  = f"{self.current_wp_idx + 1} / {len(self.waypoints)}"
            wp_pos_str  = f"({wp[0]:.1f}, {wp[1]:.1f})"
            dist_to_wp  = self._dist2(pos, wp)
            lock_str    = f"LOCKED({self._wp_lock_counter})" if wp_locked else "OPEN"
        else:
            wp_idx_str  = "N/A"
            wp_pos_str  = "N/A"
            dist_to_wp  = dist_to_target
            lock_str    = "N/A"

        # Nearest obstacle (raw geometry  -  kept for legacy column)
        if closest_bldg and bldg_dist < 999.0:
            obstacle_str = f"{closest_bldg:<14} ({bldg_dist:.1f} m)"
        else:
            obstacle_str = "none detected"

        total_crowd = len(self.parent.crowd_nodes)
        trr_str = f"{self.last_trr:5.1f}% ({self.last_tracked_count}/{total_crowd})"

        # Collision safety layer debug info
        si           = self._last_safety_info
        coll_state   = si.get("state",            "N/A")
        safe_nearest = si.get("nearest_name",     "N/A") or "N/A"
        safe_dist    = si.get("surface_dist",     float("inf"))
        danger_r     = si.get("danger_radius",    0.0)
        emerg_r      = si.get("emergency_radius", 0.0)
        avd_vec      = si.get("repulsion_vec",    (0.0, 0.0))
        avd_mag      = si.get("repulsion_mag",    0.0)

        safe_dist_str = f"{safe_dist:.1f} m" if safe_dist < 9999 else "clear"
        avd_str       = f"({avd_vec[0]:+.3f}, {avd_vec[1]:+.3f}) |{avd_mag:.3f}|"
        # State string with inline label
        state_label   = f"{coll_state:<9}"  # SAFE / WARNING  / EMERGENCY

        line = "=" * 50
        print(f"\n+{line}+")
        print(f"|  DRONE DEBUG STATUS          Step: {self.step_count:>6}        |")
        print(f"+{line}+")
        print(f"|  Motion Model     : KINEMATIC + STABILISATION         |")
        print(f"|  Motion State     : {motion_state:<28} |")
        print(f"|  Planner          : {planner_str:<28} |")
        print(f"|  Mission State    : {self.state:<28} |")
        print(f"|  Avoidance Mode   : {avoid_mode:<28} |")
        print(f"|  Waypoint         : {wp_idx_str:<8}  -> {wp_pos_str:<16} |")
        print(f"|  WP Lock          : {lock_str:<28} |")
        print(f"+{line}+")
        print(f"|  Current Pos      : ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:5.2f})      |")
        print(f"|  Velocity (vx,vy) : ({vel[0]:+7.4f}, {vel[1]:+7.4f})           |")
        print(f"|  Speed            : {spd:7.4f} m/step  (max: {self.max_velocity:.3f})  |")
        print(f"|  Accel (ax,ay)    : ({acc[0]:+7.4f}, {acc[1]:+7.4f})           |")
        print(f"|  Accel magnitude  : {acc_mag:7.4f} m/step? (max: {self.max_acceleration:.3f})|")
        print(f"+{line}+")
        print(f"|  Yaw (current)    : {yaw_deg:+8.2f} deg                       |")
        print(f"|  Yaw target (raw) : {raw_yaw_deg:+8.2f} deg                       |")
        print(f"|  Yaw target (filt): {filt_yaw_deg:+8.2f} deg                       |")
        print(f"|  Heading error    : {herr_deg:+8.2f} deg                       |")
        print(f"|  Turn rate        : {math.degrees(turn):+7.3f} deg/step (max: {math.degrees(self.max_turn_rate):.1f})|")
        print(f"+{line}+")
        print(f"|  Altitude         : {pos[2]:6.2f} m   (target={self.altitude:.1f} m)       |")
        print(f"|  Dist to Target   : {dist_to_target:7.2f} m                         |")
        print(f"|  Dist to WP       : {dist_to_wp:7.2f} m                         |")
        print(f"|  Nearest Obs.     : {obstacle_str:<28} |")
        print(f"|  TRR (Crowd)      : {trr_str:<28} |")
        print(f"+{line}+")
        print(f"|  > COLLISION SAFETY LAYER (emergency fail-safe) <          |")
        print(f"|  Coll. State      : {state_label:<28} |")
        print(f"|  Nearest (safety) : {safe_nearest:<14} {safe_dist_str:<13} |")
        print(f"|  Danger  radius   : {danger_r:5.1f} m  (WARNING threshold)        |")
        print(f"|  Emerg.  radius   : {emerg_r:5.1f} m  (EMERGENCY threshold)      |")
        print(f"|  Avoidance vector : {avd_str:<28} |")
        print(f"+{line}+")

        # -- Local Avoidance Sensor HUD section ---------------------------
        la_info     = getattr(self, '_last_la_info', {})
        la_enabled  = self.local_avoidance.enabled
        la_active   = la_info.get("active",         False)
        la_near_ray = la_info.get("nearest_ray",    "none")
        la_near_d   = la_info.get("nearest_dist_m", float("inf"))
        la_avec     = la_info.get("avoidance_vec",  (0.0, 0.0))
        la_amag     = la_info.get("avoidance_mag",  0.0)
        la_rays     = la_info.get("ray_results",    [])

        la_near_d_str  = f"{la_near_d:.2f} m" if la_near_d < 9999 else "clear"
        la_avec_str    = f"({la_avec[0]:+.3f}, {la_avec[1]:+.3f}) |{la_amag:.3f}|"
        la_active_str  = "ACTIVE" if la_active else "IDLE"
        la_en_str      = "ENABLED" if la_enabled else "DISABLED"

        # Build compact ray status line: F=front, FR=front-right, etc.
        ray_abbr = {"front": "FW", "front-left": "FL", "left": "LT",
                    "rear-left": "RL", "rear": "RR", "rear-right": "RR",
                    "right": "RT", "front-right": "FR"}
        ray_parts = []
        for rr in la_rays:
            abbr = ray_abbr.get(rr["label"], rr["label"][:2].upper())
            frac = rr["hit_fraction"]
            if not rr["hit"]:
                tag = f"{abbr}:(OK)"   # checkmark = clear
            elif frac < self.local_avoidance.red_threshold:
                tag = f"{abbr}:!!!"      # red  -  very close
            elif frac < self.local_avoidance.yellow_threshold:
                tag = f"{abbr}:!"        # yellow  -  nearby
            else:
                tag = f"{abbr}:(OK)"   # green (hit but far)
            ray_parts.append(tag)
        ray_status_str = " ".join(ray_parts)

        print(f"|  > LOCAL AVOIDANCE SENSOR (Phase-1, primary) <          |")
        print(f"|  Sensor           : {la_en_str:<28} |")
        print(f"|  Sensor range     : {self.local_avoidance.sensor_distance:5.1f} m  ({self.local_avoidance.NUM_RAYS} rays)            |")
        print(f"|  Active           : {la_active_str:<28} |")
        print(f"|  Nearest hit ray  : {la_near_ray:<28} |")
        print(f"|  Nearest hit dist : {la_near_d_str:<28} |")
        print(f"|  Avoidance vector : {la_avec_str:<28} |")
        # Ray status  -  may be long; trim to fit box
        ray_disp = ray_status_str[:46] if len(ray_status_str) > 46 else ray_status_str
        print(f"|  Ray states       : {ray_disp:<28} |")
        print(f"+{line}+")




# ---------------------------------------------------------------------------------


# -- Camera mode presets --------------------------------------------------------
# position: [x, y, z]  |  orientation: [ax, ay, az, angle]
# In ENU (Z-up), default Viewpoint orientation looks along -Z (straight down).
# The existing oblique angle -0.16 0.22 0.96 1.32 gives a good city view.
_CAM_MODES = {
    1: {
        "label":       "Cinematic city (Pan & Tilt)",
        "position":    [80.0, -100.0, 75.0],
        "orientation": [-0.16, 0.22, 0.96, 1.32],
        "follow":      "UAV_0",
        "followType":  "Pan and Tilt Shot",
    },
    2: {
        "label":       "Chase-cam (Webots Tracking Shot)",
        "position":    [14.288285517172052, -82.68417751996504, 57.06941199587428],
        "orientation": [0.07663242203175878, 0.17066914565780517, 0.9823485121345423, 1.3436034177699318],
        "follow":      "UAV_0",
        "followType":  "Tracking Shot",
    },
    3: {
        "label":       "Top-down overview",
        "position":    [0.0, 0.0, 130.0],
        # NOTE: [0,0,1,0] is a degenerate axis-angle (zero angle) and causes Webots
        # to render a black screen after clock drift. Use a near-identity rotation
        # around a well-defined axis instead  -  visually identical but numerically safe.
        "orientation": [0.0, 1.0, 0.0, 0.0001],  # ~0 rad around Y = looks straight down along -Z
        "follow":      "",
        "followType":  "None",
    },
}


class MultiUAVSurveillance:

    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.num_uavs = 5
        self.patrol_radius = 45.0
        self.patrol_altitude = 15.0  # Z in ENU

        # Gather UAV node references
        self.uavs = []
        self.uav_trans = []
        self.uav_rot = []
        for i in range(self.num_uavs):
            node = self.supervisor.getFromDef(f"UAV_{i}")
            if node is not None:
                self.uavs.append(node)
                self.uav_trans.append(node.getField("translation"))
                self.uav_rot.append(node.getField("rotation"))
            else:
                print(f"[WARN] DEF UAV_{i} not found")

        self.prev_targets = [None] * self.num_uavs

        # Capture starting translations and rotations for soft resets
        self.initial_translations = []
        self.initial_rotations = []
        for tf, rf in zip(self.uav_trans, self.uav_rot):
            self.initial_translations.append(list(tf.getSFVec3f()))
            self.initial_rotations.append(list(rf.getSFRotation()))

        # Gather Bird node references (up to 8)
        self.birds = []
        self.bird_trans = []
        for i in range(8):
            node = self.supervisor.getFromDef(f"BIRD_{i}")
            if node is not None:
                self.birds.append(node)
                self.bird_trans.append(node.getField("translation"))

        # Gather crowd agent positions (read-only tracking)
        self.crowd_nodes = []
        self._collect_crowd()

        # Gather building positions
        self.building_nodes = []
        self._collect_buildings()

        self.step_count = 0
        self.coverage_log = []

        # -- Camera control -----------------------------------------------------
        self.viewpoint = self.supervisor.getFromDef("MAIN_VIEW")
        self.cam_mode = 1
        self._cam_health_counter = 0  # increments each step; triggers recovery check every 500 steps
        # Task 2: smooth chase-cam lerp state (reset by _set_camera_mode on mode 2 activation)
        self._chase_smooth_pos    = None   # [x, y, z] lerped viewpoint position
        self._chase_smooth_target = None   # [x, y, z] lerped look-at point
        try:
            self.keyboard = self.supervisor.getKeyboard()
            self.keyboard.enable(self.timestep)
            self._kb_available = True
            print("[UAV Controller] Keyboard successfully enabled and listening.")
        except Exception as e:
            print(f"[WARN] Keyboard failed to enable: {e}")
            self._kb_available = False

        print(f"[UAV Controller] UAVs={len(self.uavs)}, "
              f"Birds={len(self.birds)}, Crowd={len(self.crowd_nodes)}")
        self._print_camera_help()

        # -- RL Integration Hook ------------------------------------------------
        import sys
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
        if root_dir not in sys.path:
            sys.path.append(root_dir)

        self.rl_enabled = False
        self.baseline_nav_enabled = False
        self.baseline_nav_cfg = {}
        config_path = os.path.join(root_dir, "configs", "environment_config.yaml")
        if os.path.isfile(config_path):
            try:
                with open(config_path, "r") as f:
                    _cfg = yaml.safe_load(f)
                # Safely navigate rl.enabled  -  no manual text scanning
                self.rl_enabled = bool(_cfg.get("rl", {}).get("enabled", False))
                # Read single-drone baseline navigation config
                _bn_cfg = _cfg.get("baseline_navigation", {})
                self.baseline_nav_enabled = bool(_bn_cfg.get("enabled", False))
                self.baseline_nav_cfg = _bn_cfg
            except Exception as e:
                print(f"[WARN] Failed to parse config file: {e}")

        if self.rl_enabled:
            print("[UAV Controller] RL mode is ENABLED. Loading Gym wrapper...")
            try:
                from rl.gym_wrapper.uav_swarm_env import UAVSwarmEnv
                self.env = UAVSwarmEnv(self)
                print("[UAV Controller] Gymnasium environment successfully initialized in-process.")
            except Exception as e:
                print(f"[ERROR] Failed to load Gymnasium environment: {e}")
                self.rl_enabled = False

        if self.baseline_nav_enabled:
            print("[UAV Controller] Baseline navigation mode ENABLED "
                  "(RL and patrol are inactive).")
            self._set_camera_mode(2)

    # -- Camera control ---------------------------------------------------------

    def _print_camera_help(self):
        """Print camera mode instructions once at startup."""
        print("\n" + "=" * 52)
        print("  CAMERA MODES  (click Webots 3D view first)")
        print("  Key 1 : Cinematic city view  [default]")
        print("  Key 2 : Chase-cam  (tracks UAV_0 closely)")
        print("  Key 3 : Top-down overview  (all drones visible)")
        print("=" * 52 + "\n")

    def _set_camera_mode(self, mode: int):
        """Switch Viewpoint to the requested camera mode (1/2/3)."""
        if self.viewpoint is None:
            print(f"[Camera] MAIN_VIEW not found in scene  -  skipping")
            return
        cfg = _CAM_MODES.get(mode)
        if cfg is None:
            return
        try:
            self.viewpoint.getField("position").setSFVec3f(cfg["position"])
            self.viewpoint.getField("orientation").setSFRotation(cfg["orientation"])
            self.viewpoint.getField("follow").setSFString(cfg["follow"])
            self.viewpoint.getField("followType").setSFString(cfg["followType"])
            self.cam_mode = mode
            print(f"[Camera] Mode {mode} activated  -  {cfg['label']}")
            # Reset smooth cam state so mode 2 lerp starts fresh
            if mode == 2:
                self._chase_smooth_pos    = None
                self._chase_smooth_target = None
                if hasattr(self, '_cam_manual_override_ticks'):
                    self._cam_manual_override_ticks = 0
                print("[Camera] Chase-cam smoothing reset  -  "
                      "will begin interpolating from current drone position.")
        except Exception as e:
            print(f"[Camera] Could not switch mode: {e}")

    def _change_focus(self, uav_name: str):
        """Dynamically shift camera focus to a specific UAV (UAV_0 to UAV_4)."""
        if self.viewpoint is None:
            return
        try:
            self.viewpoint.getField("follow").setSFString(uav_name)
            print(f"[Camera] Focus shifted to: {uav_name}")
            
            # Dynamically update camera mode presets so mode switches follow this UAV
            _CAM_MODES[1]["follow"] = uav_name
            _CAM_MODES[2]["follow"] = uav_name
        except Exception as e:
            print(f"[Camera] Could not shift focus to {uav_name}: {e}")



    def _validate_and_recover_camera(self):
        """
        Periodic camera health check. Detects invalid/degenerate viewpoint state
        (black-screen conditions) and automatically resets to a safe cinematic view.
        Called every 500 simulation steps.
        """
        if self.viewpoint is None:
            return
        try:
            pos = self.viewpoint.getField("position").getSFVec3f()
            ori = self.viewpoint.getField("orientation").getSFRotation()

            # Check 1: position outside the world bounds (+/-500m radius is generous)
            pos_magnitude = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            if pos_magnitude > 500.0:
                print(f"[Camera Recovery] Position out of bounds ({pos_magnitude:.1f}m)  -  "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

            # Check 2: degenerate orientation (near-zero angle = undefined rotation axis)
            # ori = [ax, ay, az, angle]; if |angle| < 1e-6 and axis not normalized -> degenerate
            angle = abs(ori[3])
            axis_len = math.sqrt(ori[0]**2 + ori[1]**2 + ori[2]**2)
            if angle < 1e-6 and axis_len < 0.5:
                print(f"[Camera Recovery] Degenerate orientation detected "
                      f"(angle={angle:.2e}, axis_len={axis_len:.3f})  -  "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

            # Check 3: altitude below ground (camera inside the terrain)
            if pos[2] < 0.0:
                print(f"[Camera Recovery] Camera below ground (Z={pos[2]:.1f}m)  -  "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

        except Exception as e:
            print(f"[Camera Recovery] Health check failed ({e})  -  Resetting to cinematic view")
            try:
                self._set_camera_mode(1)
            except Exception:
                pass

    def _handle_keyboard(self):
        """Read keyboard and switch camera mode (keys 1-3) or camera focus (keys 4-8)."""
        if not self._kb_available:
            return
        try:
            key = self.keyboard.getKey()
            while key > 0:
                if key == ord('1'):
                    self._set_camera_mode(1)
                elif key == ord('2'):
                    self._set_camera_mode(2)
                elif key == ord('3'):
                    self._set_camera_mode(3)
                elif key == ord('4'):
                    self._change_focus("UAV_0")
                elif key == ord('5'):
                    self._change_focus("UAV_1")
                elif key == ord('6'):
                    self._change_focus("UAV_2")
                elif key == ord('7'):
                    self._change_focus("UAV_3")
                elif key == ord('8'):
                    self._change_focus("UAV_4")
                key = self.keyboard.getKey()
        except Exception:
            pass

    # -- Crowd collection -------------------------------------------------------

    def _collect_crowd(self):
        """Find all Pedestrian / CrowdAgent nodes in the scene."""
        root = self.supervisor.getRoot()
        children = root.getField("children")
        n = children.getCount()
        for i in range(n):
            node = children.getMFNode(i)
            if node is None:
                continue
            type_name = node.getTypeName()
            if type_name in ("Pedestrian", "CrowdAgent"):
                self.crowd_nodes.append(node)
            elif type_name == "Robot":
                def_name = node.getDef()
                if def_name and any(kw in def_name.lower() for kw in ("ped", "crowd", "worker", "agent")):
                    self.crowd_nodes.append(node)

    def _collect_buildings(self):
        """Find all Building-like nodes in the scene to measure proximity."""
        root = self.supervisor.getRoot()
        children = root.getField("children")
        n = children.getCount()
        for i in range(n):
            node = children.getMFNode(i)
            if node is None:
                continue
            type_name = node.getTypeName()
            if type_name in ("SimpleBuilding", "CommercialBuilding", "ResidentialBuilding", "LargeResidentialTower", "Hotel", "RandomBuilding"):
                self.building_nodes.append(node)
            else:
                def_name = node.getDef()
                if def_name and any(kw in def_name.lower() for kw in ("building", "tower", "office", "hotel", "commercial")):
                    self.building_nodes.append(node)

    # -- UAV patrol -------------------------------------------------------------

    def _patrol_target(self, uav_idx, t):
        """Circular formation with POMDP-style attention bias."""
        # Base circular patrol
        offset_angle = uav_idx * (2 * math.pi / self.num_uavs)
        r = self.patrol_radius + 5 * math.sin(t * 0.012 + uav_idx)
        angle = t * 0.006 + offset_angle

        x = r * math.cos(angle)
        y = r * math.sin(angle)
        z = self.patrol_altitude + 3 * math.sin(t * 0.009 + uav_idx * 0.8)

        # Attention: bias toward crowd density
        if self.crowd_nodes:
            crowd_x, crowd_y = self._crowd_centroid()
            # Soft pull toward crowd centroid (attention mechanism)
            alpha = 0.15
            x = x * (1 - alpha) + crowd_x * alpha
            y = y * (1 - alpha) + crowd_y * alpha

        return [x, y, z]

    def _crowd_centroid(self):
        cx, cy, count = 0.0, 0.0, 0
        for node in self.crowd_nodes:
            try:
                pos = node.getPosition()
                cx += pos[0]
                cy += pos[1]
                count += 1
            except Exception:
                pass
        if count > 0:
            return cx / count, cy / count
        return 0.0, 0.0

    def update_uavs(self):
        t = self.step_count
        for i, (tf, rf) in enumerate(zip(self.uav_trans, self.uav_rot)):
            target = self._patrol_target(i, t)
            tf.setSFVec3f(target)
            
            # Orient the drone in the direction of movement
            if self.prev_targets[i] is not None:
                prev = self.prev_targets[i]
                dx = target[0] - prev[0]
                dy = target[1] - prev[1]
                if math.hypot(dx, dy) > 1e-4:
                    yaw = math.atan2(dy, dx)
                    rf.setSFRotation([0.0, 0.0, 1.0, yaw])
            self.prev_targets[i] = target

    # -- Bird flock (fallback if bird_controller not running) ------------------

    def update_birds(self):
        t = self.step_count
        for i, (node, tf) in enumerate(zip(self.birds, self.bird_trans)):
            # Simple boids-lite circular orbit
            angle = t * (0.3 + i * 0.025) * 0.01 + i * 0.8
            radius = 12 + i * 2.5
            base_alt = 6.0 + (i % 3) * 1.5
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = base_alt + 1.8 * math.sin(t * 0.008 + i)
            try:
                tf.setSFVec3f([x, y, z])
            except Exception:
                pass

    # -- Surveillance metrics ---------------------------------------------------

    def compute_coverage(self):
        """Grid-cell coverage metric (percentage of 200x200m arena)."""
        covered = set()
        cell = 5.0
        half = 90.0
        for node in self.uavs:
            pos = node.getPosition()
            view_r = pos[2] * 0.7  # altitude-based footprint
            for dx in range(-int(view_r / cell), int(view_r / cell) + 1):
                for dy in range(-int(view_r / cell), int(view_r / cell) + 1):
                    cx = int((pos[0] + dx * cell) / cell)
                    cy = int((pos[1] + dy * cell) / cell)
                    if -int(half / cell) <= cx <= int(half / cell):
                        if -int(half / cell) <= cy <= int(half / cell):
                            covered.add((cx, cy))
        total = (2 * half / cell) ** 2
        return len(covered) / total * 100.0

    def log_metrics(self):
        if self.step_count % 125 == 0:
            cov = self.compute_coverage()
            self.coverage_log.append(cov)
            print(f"\n{'='*52}")
            print(f" Step {self.step_count:>6} | "
                  f"Coverage {cov:5.1f}% | "
                  f"Crowd {len(self.crowd_nodes):>3} | "
                  f"Cam Mode {self.cam_mode}")
            for i, node in enumerate(self.uavs):
                p = node.getPosition()
                print(f"   UAV_{i}: ({p[0]:6.1f}, {p[1]:6.1f}, {p[2]:5.1f})")
            print(f"{'='*52}")

    # -- Main loop --------------------------------------------------------------

    def run(self):
        print("[UAV Controller] Surveillance system online.")
        
        # Check for benchmark run env var
        run_benchmark = os.environ.get("RUN_BENCHMARK", "").lower() == "true"
        
        if run_benchmark:
            print("\n" + "=" * 58)
            print("  RUNNING RULE-BASED BASELINE BENCHMARK")
            print("=" * 58)
            
            import numpy as np
            import json
            
            num_episodes = 10
            steps_per_episode = 1000
            
            episode_coverages = []
            episode_trrs = []
            episode_collisions = []
            
            # Determine project root path
            script_dir = os.path.dirname(os.path.abspath(__file__))
            root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
            
            for ep in range(num_episodes):
                print(f"[Benchmark] Starting Episode {ep + 1}/{num_episodes}...")
                
                # Reset simulation positions (soft reset)
                self.step_count = 0
                self.prev_targets = [None] * self.num_uavs
                for i, (tf, rf) in enumerate(zip(self.uav_trans, self.uav_rot)):
                    tf.setSFVec3f(self.initial_translations[i])
                    rf.setSFRotation(self.initial_rotations[i])
                    try:
                        self.uavs[i].resetPhysics()
                    except Exception:
                        pass
                
                # Step once to apply changes in the physics engine
                self.supervisor.step(self.timestep)
                
                ep_covs = []
                ep_trrs = []
                ep_coll_events = 0
                
                for step in range(steps_per_episode):
                    self.step_count += 1
                    
                    # Update UAVs and birds rule-based patrol
                    self.update_uavs()
                    if self.birds:
                        self.update_birds()
                    
                    # Advance simulation
                    if self.supervisor.step(self.timestep) == -1:
                        break
                        
                    # Calculate coverage
                    cov = self.compute_coverage()
                    ep_covs.append(cov)
                    
                    # Calculate TRR
                    uav_positions = [tf.getSFVec3f() for tf in self.uav_trans]
                    crowd_positions = []
                    for node in self.crowd_nodes:
                        try:
                            crowd_positions.append(node.getPosition())
                        except Exception:
                            pass
                            
                    tracked = set()
                    for ci, c_pos in enumerate(crowd_positions):
                        for u_pos in uav_positions:
                            dist = math.hypot(u_pos[0] - c_pos[0], u_pos[1] - c_pos[1])
                            view_r = u_pos[2] * 0.7
                            if dist <= view_r:
                                tracked.add(ci)
                                break
                    trr = len(tracked) / len(crowd_positions) if crowd_positions else 0.0
                    ep_trrs.append(trr * 100.0)
                    
                    # Calculate safety violations (UAV-UAV collision threshold < 5m)
                    for i in range(self.num_uavs):
                        for j in range(i + 1, self.num_uavs):
                            p_i = uav_positions[i]
                            p_j = uav_positions[j]
                            d = math.sqrt(sum((a - b) ** 2 for a, b in zip(p_i, p_j)))
                            if d < 5.0:
                                ep_coll_events += 1
                
                mean_ep_cov = np.mean(ep_covs)
                mean_ep_trr = np.mean(ep_trrs)
                print(f"  Episode {ep + 1} Done | Mean Coverage: {mean_ep_cov:.2f}% | Mean TRR: {mean_ep_trr:.2f}% | Near-Misses: {ep_coll_events}")
                
                episode_coverages.append(float(mean_ep_cov))
                episode_trrs.append(float(mean_ep_trr))
                episode_collisions.append(ep_coll_events)
                
            # Compile final metrics
            final_metrics = {
                "mean_coverage": float(np.mean(episode_coverages)),
                "std_coverage": float(np.std(episode_coverages)),
                "mean_trr": float(np.mean(episode_trrs)),
                "std_trr": float(np.std(episode_trrs)),
                "total_near_misses": int(np.sum(episode_collisions)),
                "episode_coverages": episode_coverages,
                "episode_trrs": episode_trrs,
                "episode_near_misses": episode_collisions
            }
            
            # Save final benchmark JSON
            metrics_dir = os.path.join(root_dir, "experiments", "metrics")
            os.makedirs(metrics_dir, exist_ok=True)
            benchmark_path = os.path.join(metrics_dir, "baseline_benchmark.json")
            with open(benchmark_path, "w") as f:
                json.dump(final_metrics, f, indent=4)
                
            print(f"\n[Benchmark] Complete! Saved metrics to: {benchmark_path}")
            print(f"  Final Mean Coverage: {final_metrics['mean_coverage']:.2f}% (std: {final_metrics['std_coverage']:.2f}%)")
            print(f"  Final Mean TRR:      {final_metrics['mean_trr']:.2f}% (std: {final_metrics['std_trr']:.2f}%)")
            print(f"  Total Near-Misses:   {final_metrics['total_near_misses']}")
            
            # Exit Webots supervisor gracefully
            self.supervisor.simulationQuit(0)
            return

        if self.rl_enabled:
            # Use cinematic view (mode 1) as default for RL training.
            # Mode 3 top-down used to be forced here but had a degenerate [0,0,1,0]
            # axis-angle orientation that caused black screens after long runs.
            # Mode 1 uses proven-safe orientation [-0.16, 0.22, 0.96, 1.32].
            # Press key 3 during runtime for top-down if needed.
            self._set_camera_mode(1)
            
            # Load active PPO trainer in-process
            try:
                from rl.training.trainer import Trainer
                headless_run = os.environ.get("WEBOTS_HEADLESS", "false").lower() == "true"
                
                # Resolve root directory path
                script_dir = os.path.dirname(os.path.abspath(__file__))
                root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
                model_canonical_path = os.path.join(root_dir, "models", "ppo_swarm_final.zip")
                
                trainer = Trainer(
                    scenario="downtown",
                    episodes=500,
                    max_steps=5000,       # was 1000 -> matched to UAVSwarmEnv.MAX_STEPS
                    headless=headless_run,  # Dynamically resolved from launch script env var
                    log_interval=10,
                    save_interval=50000,  # Save model checkpoints every 50k steps
                    env=self.env
                )
                trainer.setup()
                
                # If a trained model is found and we are running in GUI mode, run demo mode!
                if not headless_run and os.path.exists(model_canonical_path):
                    trainer.evaluate(model_canonical_path)
                else:
                    # Otherwise, run PPO training
                    trainer.run()
            except Exception as e:
                print(f"[ERROR] Trainer execution failed: {e}")
                raise e
        else:
            # -- Check for single-drone baseline navigation mode -----------------
            if self.baseline_nav_enabled:
                print("[UAV Controller] Starting Single-Drone Navigation Baseline...")
                self._set_camera_mode(2)  # chase-cam follows UAV_0  -  best for baseline
                nav = SingleDroneNavigation(self, self.baseline_nav_cfg)
                while self.supervisor.step(self.timestep) != -1:
                    self.step_count += 1
                    self._handle_keyboard()         # camera mode switching (GUI only)
                    nav.step()                      # navigation state machine

                    # Periodic camera health check
                    self._cam_health_counter += 1
                    if self._cam_health_counter % 500 == 0:
                        self._validate_and_recover_camera()
            else:
                # Rule-based baseline patrol (original behaviour)
                while self.supervisor.step(self.timestep) != -1:
                    self.step_count += 1
                    self._handle_keyboard()      # camera mode switching (GUI only)
                    self.update_uavs()
                    if self.birds:
                        self.update_birds()
                    self.log_metrics()

                    # Periodic camera health check  -  recover from black-screen conditions
                    self._cam_health_counter += 1
                    if self._cam_health_counter % 500 == 0:
                        self._validate_and_recover_camera()


if __name__ == "__main__":
    controller = MultiUAVSurveillance()
    controller.run()
