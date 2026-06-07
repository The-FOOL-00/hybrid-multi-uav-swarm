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
import random
import os
import yaml


# ══════════════════════════════════════════════════════════════════════════════
# A* GLOBAL PATH PLANNER
# ══════════════════════════════════════════════════════════════════════════════
class AStarPlanner:
    """
    Lightweight 2-D A* path planner on a uniform occupancy grid.

    Coordinate mapping
    ------------------
    World XY → grid (col, row):
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
    redundant intermediate waypoints: if the segment (wA → wC) is free of
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

    # ── Grid construction ───────────────────────────────────────────────────

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

    # ── Coordinate helpers ──────────────────────────────────────────────────

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

    # ── A* core ────────────────────────────────────────────────────────────

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

        print("[A*] WARNING: No path found — falling back to straight line")
        return []

    # ── Path smoothing ──────────────────────────────────────────────────────

    def _los_clear(self, ax: float, ay: float, bx: float, by: float,
                   samples: int = 20) -> bool:
        """
        Check if the straight segment (ax,ay)→(bx,by) is free of obstacles.
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

    # ── Nearest-free helper ─────────────────────────────────────────────────

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
        return col, row   # give up — return original



# ══════════════════════════════════════════════════════════════════════════════
# SINGLE-DRONE NAVIGATION BASELINE
# Phase 1 of the hybrid-multi-UAV-swarm development pipeline.
# Deterministic start → target mission for engineering baseline validation.
# ────────────────────────────────────────────────────────────────────────────────
class SingleDroneNavigation:
    """
    Deterministic waypoint navigation for UAV_0 with optional A* global planner.

    Algorithm:
        1. TAKEOFF  — teleport UAV_0 to start_position.
        2. NAVIGATE — if A* enabled: follow planned waypoint list.
                      otherwise: steer straight toward target.
                      Each step applies local obstacle repulsion on top
                      of the waypoint steering vector.
        3. ARRIVED  — once dist_to_target < arrival_radius, stop moving
                      and announce mission complete.

    UAV_1-4 are parked at their initial positions and never updated.
    RL and random patrol are NOT used in this mode.
    """

    # State constants
    STATE_TAKEOFF  = "TAKEOFF"
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

        # ── Kinematic flight model state ──────────────────────────────────────
        # Velocity (m/step) in ENU frame — integrated each step.
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        # Current heading (radians, ENU yaw from East axis).
        self.current_yaw = 0.0

        # Kinematic parameters — read from YAML with safe defaults.
        self.max_velocity     = float(cfg.get("max_velocity",     0.5))
        self.cruise_velocity  = float(cfg.get("cruise_velocity",  self.cruise_speed))
        self.max_acceleration = float(cfg.get("max_acceleration", 0.05))
        self.max_turn_rate    = float(cfg.get("max_turn_rate",    0.08))  # rad/step
        self.decel_radius     = float(cfg.get("decel_radius",     8.0))   # m

        # ── HUD telemetry (populated each kinematic step) ─────────────────────
        self.last_avoiding    = False
        self.last_steer_vec   = (0.0, 0.0)
        self.last_velocity    = (0.0, 0.0)   # (vx, vy) for HUD
        self.last_speed       = 0.0          # |v_xy| m/step
        self.last_accel       = (0.0, 0.0)   # (ax, ay) applied this step
        self.last_accel_mag   = 0.0
        self.last_turn_rate   = 0.0          # rad/step applied this step

        # ── Stabilisation state ─────────────────────────────────────────────
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

        # UAV_0 node references (index 0 in parent lists)
        if len(parent.uav_trans) == 0:
            raise RuntimeError("[SingleDroneNav] UAV_0 not found in scene!")
        self.uav_tf = parent.uav_trans[0]   # translation field
        self.uav_rf = parent.uav_rot[0]     # rotation field
        self.uav_node = parent.uavs[0]      # Mavic2Pro node

        self.state       = self.STATE_TAKEOFF
        self.step_count  = 0
        self.arrived_reported = False

        # Dynamically load all buildings from the scene to prevent flying through any buildings
        self.test_buildings = []
        self._init_buildings()
        self.last_warning_step = -999

        # ── Metrics collection state ──────────────────────────────────────────
        self.sim_start_time      = None     # supervisor time at first step (s)
        self.distance_travelled  = 0.0      # cumulative 3-D distance (m)
        self.collision_count     = 0        # count of proximity/collision steps
        self.altitude_readings   = []       # Z values each step for stability calc
        self.trr_readings        = []       # TRR values each step for average calculation
        self.step_log            = []       # sparse log for JSON export
        self.reached_target      = False
        self._prev_pos           = None
        self.last_trr            = 0.0
        self.last_tracked_count  = 0

        # ── A* planner setup ──────────────────────────────────────────────
        self.astar_enabled   = bool(cfg.get("astar_enabled", True))
        self.grid_resolution = float(cfg.get("grid_resolution", 2.0))
        self.grid_margin     = float(cfg.get("grid_margin", 90.0))
        self.obstacle_inflation = float(cfg.get("obstacle_inflation", 2.0))
        self.waypoint_radius = float(cfg.get("waypoint_radius", 4.0))
        self.show_wp_markers = bool(cfg.get("show_waypoint_markers", True))

        # Waypoint list — populated during first NAVIGATE step via _init_astar()
        self.waypoints          = []   # list of [x, y, z]
        self.current_wp_idx     = 0
        self.astar_initialized  = False
        self._marker_nodes      = []   # Webots Solid nodes for visual debug

        # ══════════════════════════════════════════════════════════════════
        # DEBUG / OBSERVABILITY STATE  (Tasks 1–6)
        # ══════════════════════════════════════════════════════════════════
        self.debug_mode = True          # master switch for all debug visuals

        # Task 1 — debug beacon above UAV_0
        self._beacon_node = None        # Webots Solid node reference
        self._beacon_tf   = None        # translation field of beacon node
        self._beacon_offset_z = 4.0     # metres above drone centre

        # Task 2 — smooth chase camera (active when cam_mode == 2)
        # Offset vector in drone heading frame: behind=−ve heading, up=positive Z
        self._chase_behind_dist = 14.0  # m behind drone
        self._chase_above_dist  = 7.0   # m above drone
        self._chase_lookahead   = 4.0   # m ahead of drone for look-at target
        self._chase_lerp_alpha  = 0.07  # smoothing factor (0=frozen, 1=instant)
        self._cam_smooth_pos    = None  # smoothed viewpoint position [x,y,z]
        self._cam_smooth_target = None  # smoothed look-at point [x,y,z]

        # Task 3 — persistent path trail (blue dots)
        self._trail_positions   = []    # sampled drone positions
        self._trail_nodes       = []    # spawned Webots Solid nodes
        self._trail_counter     = 0     # steps since last sample
        self._trail_interval    = 10    # sample every N steps
        self._trail_max_nodes   = 600   # cap on spawned nodes
        self._trail_radius      = 0.35  # sphere radius (metres)

        # Task 6 — ASCII minimap
        self._minimap_interval  = 200   # print minimap every N steps

        # Park UAV_1-4 at their initial positions so they don't drift
        self._park_inactive_uavs()

        self._print_banner()

    # ── Setup helpers ─────────────────────────────────────────────────────

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
        print(f"  Max accel       : {self.max_acceleration} m/step\u00b2")
        print(f"  Max turn rate   : {math.degrees(self.max_turn_rate):.1f} deg/step")
        print(f"  Decel radius    : {self.decel_radius} m before WP")
        print(f"  Arrival zone    : {self.arrival_radius} m radius")
        print(f"  Planner         : {astar_mode}")
        if self.astar_enabled:
            print(f"  Grid res        : {self.grid_resolution} m/cell")
            print(f"  WP radius       : {self.waypoint_radius} m")
        print("  States       : TAKEOFF \u2192 NAVIGATE \u2192 ARRIVED")
        print("  -" * 32)
        print("  DEBUG FEATURES ACTIVE:")
        print("    [T1] Smooth chase-cam    (key 2 to activate)")
        print("    [T2] Blue path trail     (every 10 steps)")
        print("    [T3] Improved WP markers (larger, emissive)")
        print("    [T4] Full debug HUD      (every 100 steps, speed/accel/yaw)")
        print("    [T5] ASCII minimap       (every 200 steps)")
        print("=" * 64 + "\n")

    # ── Navigation logic ─────────────────────────────────────────────────

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
        
        # Define lawnmower sweep key waypoints aligned with road intersections
        sweep_targets = [
            [-40.0, -40.0],
            [0.0, -40.0],
            [0.0, 40.0],
            [40.0, 40.0],
            [40.0, -40.0],
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
                print(f"[A*] Warning: leg to {target} failed planning — using straight-line fallback")
                raw_wps.append(target)
                current_start = target

        # Convert (x, y) → [x, y, z] with mission altitude
        self.waypoints = [[wx, wy, self.altitude] for wx, wy in raw_wps]
        self.current_wp_idx = 0

        # ── Task 2: Startup angle sanity check ───────────────────────────────
        # Compare angle (start→WP0) vs (start→first sweep target).
        # If WP0 is more than 90° off the first sweep direction, skip it.
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
                          f"(angle to first sweep target: {angle_deg:.1f}° > 90°)")
                    self.waypoints.pop(0)
                else:
                    print(f"[A*] First waypoint OK (angle to first sweep target: {angle_deg:.1f}°)")

        print(f"[A*] Waypoint plan ({len(self.waypoints)} waypoints):")
        for i, wp in enumerate(self.waypoints):
            marker = " <- FINAL TARGET" if i == len(self.waypoints) - 1 else ""
            print(f"  WP[{i:02d}] ({wp[0]:7.2f}, {wp[1]:7.2f}, {wp[2]:.1f}){marker}")
        print()

        # Spawn visual markers if requested
        if self.show_wp_markers:
            self._spawn_waypoint_markers()

        # Spawn debug beacon above drone (Task 1)
        if self.debug_mode:
            self._spawn_debug_beacon()

        self.astar_initialized = True

    def _spawn_waypoint_markers(self):
        """
        Spawn improved waypoint markers (Task 4):
          - Bigger spheres (radius 1.5 vs 1.0)
          - Bright yellow intermediate, bright green final
          - Strong emissive glow so markers are visible from far away
          - Floated +1 m above the waypoint altitude
          - Numbered console printout per waypoint
        Uses supervisor importMFNodeFromString — gracefully skips on any error.
        """
        print("[WP Markers] Spawning improved waypoint markers:")
        try:
            root = self.supervisor.getRoot()
            children_field = root.getField("children")
            total = len(self.waypoints)
            for i, wp in enumerate(self.waypoints):
                is_final = (i == total - 1)
                if is_final:
                    # Bright green — final target
                    r, g, b  = 0.0, 1.0, 0.15
                    er, eg, eb = 0.0, 0.85, 0.1
                    label = "FINAL TARGET"
                else:
                    # Bright yellow — intermediate waypoint
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

    # ── Task 1: Debug Beacon ───────────────────────────────────────────────

    def _spawn_debug_beacon(self):
        """No-op: visual debug beacon disabled by request."""
        pass

    def _update_debug_beacon(self, drone_pos):
        """No-op: visual debug beacon disabled by request."""
        pass

    # ── Task 3: Persistent Path Trail ─────────────────────────────────────

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
            pass   # trail spawn failure is silent — never block simulation

    # ── Task 6: ASCII Minimap ─────────────────────────────────────────────

    def _print_minimap(self, drone_pos):
        """
        Lightweight ASCII minimap printed to console every _minimap_interval steps.
        Grid: 21 cols × 10 rows representing the ±90m world.
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

        grid = [['·'] * COLS for _ in range(ROWS)]

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
        sep = '─' * (COLS * 2 + 1)
        print(f"\n┌{sep}┐")
        for row in grid:
            print('│ ' + ' '.join(row) + ' │')
        print(f"└{sep}┘")
        wp_idx = self.current_wp_idx + 1 if self.waypoints else 0
        total_wp = len(self.waypoints)
        print(f"  ASCII Minimap │ D=drone  T=target  B=building  W=waypoint  "
              f"WP={wp_idx}/{total_wp}  "
              f"pos=({drone_pos[0]:.0f},{drone_pos[1]:.0f})")
        print()

    def _move_toward_target(self):
        """
        KINEMATIC FLIGHT MODEL with stabilisation layer.

        Algorithm (per step):
          1.  Select steering target (current A* waypoint or final target).
          2.  Compute raw desired heading toward target.
          3.  Obstacle repulsion — GATED: active only inside emergency_radius
              when A* is enabled (Task 3).  Full repulsion when A* disabled.
          4.  Low-pass filter desired_yaw (Task 4 — yaw_smoothing_alpha).
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

        # ── 1. Select steering target (A* waypoint or final target) ───────
        if self.astar_enabled and self.waypoints:
            wp = self.waypoints[self.current_wp_idx]
            tx, ty = wp[0], wp[1]
        else:
            tx, ty = self.target_pos[0], self.target_pos[1]

        dx, dy = tx - cur[0], ty - cur[1]
        horiz_dist = math.hypot(dx, dy)

        # Already on target — bleed off velocity and hold
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

        # ── 2. Raw desired heading toward target ─────────────────────────
        raw_desired_yaw = math.atan2(dy, dx)

        # ── 3. Obstacle repulsion — gated by emergency_radius (Task 3) ───
        closest_name, min_dist, bx, by = self._check_collision_distance(cur)
        self.last_avoiding = False

        if self.astar_enabled:
            # A* is in control — only intervene in true emergencies
            avoidance_active = closest_name and min_dist < self.emergency_radius
            avoidance_mode   = "EMERGENCY" if avoidance_active else "DISABLED"
        else:
            # No A*: full reactive avoidance at safety_radius
            avoidance_active = closest_name and min_dist < self.safety_radius
            avoidance_mode   = "FULL" if avoidance_active else "DISABLED"

        if avoidance_active:
            self.last_avoiding = True
            rx = cur[0] - bx
            ry = cur[1] - by
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

        # ── 4. Low-pass filter on desired_yaw (Task 4) ───────────────────
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

        # ── 5. Smooth yaw toward filtered desired_yaw (max_turn_rate per step) ─
        delta_yaw = desired_yaw - self.current_yaw
        while delta_yaw >  math.pi: delta_yaw -= 2.0 * math.pi
        while delta_yaw < -math.pi: delta_yaw += 2.0 * math.pi
        turn = max(-self.max_turn_rate, min(self.max_turn_rate, delta_yaw))
        self.current_yaw += turn
        self.last_turn_rate = turn
        while self.current_yaw >  math.pi: self.current_yaw -= 2.0 * math.pi
        while self.current_yaw < -math.pi: self.current_yaw += 2.0 * math.pi

        # ── 6. Desired speed with deceleration ramp near waypoint ─────────
        motion_state = "CRUISE"
        if horiz_dist < self.decel_radius:
            ramp_frac = max(0.05, horiz_dist / self.decel_radius)
            desired_speed = self.cruise_velocity * ramp_frac
            motion_state = "DECEL"
        else:
            desired_speed = self.cruise_velocity
        desired_speed = min(desired_speed, self.max_velocity)

        # ── 7. Velocity damping near waypoint to kill oscillation (Task 5) ──
        if horiz_dist < self.wp_damping_radius:
            self.vx *= self.velocity_damping_factor
            self.vy *= self.velocity_damping_factor
            motion_state = "DAMP"

        self.last_motion_state = motion_state

        # ── 8. Desired velocity vector from heading + speed ───────────────
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

        # ── 9. Integrate XY position ──────────────────────────────────────
        new_x = cur[0] + self.vx
        new_y = cur[1] + self.vy

        # ── 10. Kinematic Z hold (no PID, no motor thrust) ────────────────
        z_error = self.altitude - cur[2]
        desired_vz = max(-self.max_acceleration * 2,
                         min(self.max_acceleration * 2, z_error * 0.3))
        dz = desired_vz - self.vz
        dz_clamped = max(-self.max_acceleration * 0.5,
                         min(self.max_acceleration * 0.5, dz))
        self.vz += dz_clamped
        new_z = cur[2] + self.vz

        # ── Store HUD telemetry ───────────────────────────────────────────
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
        # Use the smooth kinematic yaw — prev/new_pos kept as parameters for
        # API compatibility in case callers change in future.
        self.uav_rf.setSFRotation([0.0, 0.0, 1.0, self.current_yaw])

    # ── Per-step update ───────────────────────────────────────────────────

    # ── Per-step update ───────────────────────────────────────────────────

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

        # ── Calculate TRR (Target Recognition Rate) mathematically ──
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

        # ── STATE: TAKEOFF ─────────────────────────────────────────────
        if self.state == self.STATE_TAKEOFF:
            # Teleport UAV_0 to the fixed start position and zero its physics.
            # FIX-2: resetPhysics() after setSFVec3f clears any propeller
            # momentum accumulated since last step, preventing altitude drift.
            self.uav_tf.setSFVec3f(self.start_pos)
            self.uav_rf.setSFRotation([0.0, 0.0, 1.0, 0.0])  # face East
            try:
                self.uav_node.resetPhysics()   # zero accumulated thrust
            except Exception:
                pass
            print(f"[SingleDroneNav] STATE: TAKEOFF")
            print(f"  UAV_0 placed at start: {self.start_pos}")
            print(f"  Target               : {self.target_pos}")
            total_dist = self._dist3(self.start_pos, self.target_pos)
            print(f"  Total mission dist   : {total_dist:.1f} m")
            print(f"  Estimated steps      : ~{int(total_dist / self.cruise_speed)} steps\n")
            self.state = self.STATE_NAVIGATE
            return True

        # ── STATE: NAVIGATE ───────────────────────────────────────────
        if self.state == self.STATE_NAVIGATE:

            # Lazy A* init — runs once on the very first NAVIGATE step
            if self.astar_enabled and not self.astar_initialized:
                self._init_astar()

            dist_to_target = self._dist3(cur, self.target_pos)

            if dist_to_target <= self.arrival_radius:
                # Mission complete — snap exactly to target, hold
                self.uav_tf.setSFVec3f(self.target_pos)
                self.state = self.STATE_ARRIVED
                return True

            # ── A* waypoint advance with hysteresis lock (Task 1) ────────
            if self.astar_enabled and self.waypoints:
                wp = self.waypoints[self.current_wp_idx]
                dist_to_wp = self._dist2(cur, wp)

                # Decrement hysteresis lock counter each step
                if self._wp_lock_counter > 0:
                    self._wp_lock_counter -= 1
                    if self._wp_lock_counter == 0:
                        print(f"  [WP_LOCK] Released — WP[{self.current_wp_idx:02d}] "
                              f"now active  step={self.step_count}")

                # Only advance when lock is not active
                if dist_to_wp <= self.waypoint_radius and self._wp_lock_counter == 0:
                    prev_idx = self.current_wp_idx
                    if self.current_wp_idx < len(self.waypoints) - 1:
                        self.current_wp_idx += 1
                        new_wp = self.waypoints[self.current_wp_idx]
                        self._wp_lock_counter = self.waypoint_reach_lock_steps
                        print(f"  [A*] WP[{prev_idx:02d}] reached -> advancing to "
                              f"WP[{self.current_wp_idx:02d}] "
                              f"({new_wp[0]:.1f}, {new_wp[1]:.1f})  "
                              f"step={self.step_count}")
                        print(f"  [WP_LOCK] Active for {self._wp_lock_counter} steps")

            prev_pos = list(cur)
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

            # ── DEBUG OBSERVABILITY UPDATES ───────────────────────────────
            if self.debug_mode:
                # Task 1: move beacon above drone
                self._update_debug_beacon(new_pos)
                # Task 3: add trail dot
                self._update_path_trail(new_pos)

            # Task 2: smooth chase cam override (only when cam_mode == 2)
            if self.parent.cam_mode == 2:
                self.parent._update_chase_cam(
                    new_pos, self.last_steer_vec, self._chase_lerp_alpha,
                    self._chase_behind_dist, self._chase_above_dist,
                    self._chase_lookahead
                )

            # Task 6: ASCII minimap every N steps
            if self.debug_mode and self.step_count % self._minimap_interval == 0:
                self._print_minimap(new_pos)
            # ─────────────────────────────────────────────────────────────

            # Lightweight Collision Check
            closest_name, min_dist, _, _ = self._check_collision_distance(cur)
            if min_dist < self.safety_radius:
                self.collision_count += 1  # Accumulate proximity warning steps

            if min_dist < 4.0 and (self.step_count - self.last_warning_step) > 20:
                print(f"  [WARNING] Potential collision likely with {closest_name}! (dist: {min_dist:.1f}m)")
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

            return True

        # ── STATE: ARRIVED ───────────────────────────────────────────
        if self.state == self.STATE_ARRIVED:
            if not self.arrived_reported:
                self.reached_target = True
                cur = list(self.uav_tf.getSFVec3f())
                print("\n" + "=" * 64)
                print("  ✓ MISSION COMPLETE — UAV_0 ARRIVED AT TARGET")
                print(f"  Final position : ({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})")
                print(f"  Target         : {self.target_pos}")
                print(f"  Steps taken    : {self.step_count}")
                print(f"  State          : ARRIVED (drone holding position)")
                print("=" * 64 + "\n")
                
                # Save metrics to JSON file
                self._save_metrics()
                self.arrived_reported = True
                
            # Hold position — reset physics every step so propellers
            # don't push the drone away from the target (FIX-2).
            self.uav_tf.setSFVec3f(self.target_pos)
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
        
        metrics = {
            "phase": "Phase 1 — Single Drone Lawnmower Patrol & Surveillance Baseline",
            "world": "worlds/single_drone_downtown.wbt",
            "start_pos": self.start_pos,
            "target_pos": self.target_pos,
            "reached_target": self.reached_target,
            "travel_time_s": round(travel_time, 3),
            "distance_travelled_m": round(self.distance_travelled, 3),
            "proximity_events": self.collision_count,
            "altitude_stability_std_m": round(alt_std, 5),
            "mean_trr_percent": round(mean_trr, 2),
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

        # Nearest obstacle
        if closest_bldg and bldg_dist < 999.0:
            obstacle_str = f"{closest_bldg:<14} ({bldg_dist:.1f} m)"
        else:
            obstacle_str = "none detected"

        total_crowd = len(self.parent.crowd_nodes)
        trr_str = f"{self.last_trr:5.1f}% ({self.last_tracked_count}/{total_crowd})"

        line = "\u2550" * 50
        print(f"\n\u2554{line}\u2557")
        print(f"\u2551  DRONE DEBUG STATUS          Step: {self.step_count:>6}        \u2551")
        print(f"\u2560{line}\u2563")
        print(f"\u2551  Motion Model     : KINEMATIC + STABILISATION         \u2551")
        print(f"\u2551  Motion State     : {motion_state:<28} \u2551")
        print(f"\u2551  Planner          : {planner_str:<28} \u2551")
        print(f"\u2551  Mission State    : {self.state:<28} \u2551")
        print(f"\u2551  Avoidance Mode   : {avoid_mode:<28} \u2551")
        print(f"\u2551  Waypoint         : {wp_idx_str:<8}  -> {wp_pos_str:<16} \u2551")
        print(f"\u2551  WP Lock          : {lock_str:<28} \u2551")
        print(f"\u2560{line}\u2563")
        print(f"\u2551  Current Pos      : ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:5.2f})      \u2551")
        print(f"\u2551  Velocity (vx,vy) : ({vel[0]:+7.4f}, {vel[1]:+7.4f})           \u2551")
        print(f"\u2551  Speed            : {spd:7.4f} m/step  (max: {self.max_velocity:.3f})  \u2551")
        print(f"\u2551  Accel (ax,ay)    : ({acc[0]:+7.4f}, {acc[1]:+7.4f})           \u2551")
        print(f"\u2551  Accel magnitude  : {acc_mag:7.4f} m/step\u00b2 (max: {self.max_acceleration:.3f})\u2551")
        print(f"\u2560{line}\u2563")
        print(f"\u2551  Yaw (current)    : {yaw_deg:+8.2f} deg                       \u2551")
        print(f"\u2551  Yaw target (raw) : {raw_yaw_deg:+8.2f} deg                       \u2551")
        print(f"\u2551  Yaw target (filt): {filt_yaw_deg:+8.2f} deg                       \u2551")
        print(f"\u2551  Heading error    : {herr_deg:+8.2f} deg                       \u2551")
        print(f"\u2551  Turn rate        : {math.degrees(turn):+7.3f} deg/step (max: {math.degrees(self.max_turn_rate):.1f})\u2551")
        print(f"\u2560{line}\u2563")
        print(f"\u2551  Altitude         : {pos[2]:6.2f} m   (target={self.altitude:.1f} m)       \u2551")
        print(f"\u2551  Dist to Target   : {dist_to_target:7.2f} m                         \u2551")
        print(f"\u2551  Dist to WP       : {dist_to_wp:7.2f} m                         \u2551")
        print(f"\u2551  Nearest Obs.     : {obstacle_str:<28} \u2551")
        print(f"\u2551  TRR (Crowd)      : {trr_str:<28} \u2551")
        print(f"\u255a{line}\u255d")



# ─────────────────────────────────────────────────────────────────────────────────


# ── Camera mode presets ────────────────────────────────────────────────────────
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
        "label":       "Smooth chase-cam (custom interpolated follow)",
        "position":    [80.0, -100.0, 75.0],   # initial seed — overridden each step
        "orientation": [-0.16, 0.22, 0.96, 1.32],
        # followType set to None so our manual _update_chase_cam() takes full control
        "follow":      "UAV_0",
        "followType":  "None",
    },
    3: {
        "label":       "Top-down overview",
        "position":    [0.0, 0.0, 130.0],
        # NOTE: [0,0,1,0] is a degenerate axis-angle (zero angle) and causes Webots
        # to render a black screen after clock drift. Use a near-identity rotation
        # around a well-defined axis instead — visually identical but numerically safe.
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

        # ── Camera control ─────────────────────────────────────────────────────
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

        # ── RL Integration Hook ────────────────────────────────────────────────
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
                # Safely navigate rl.enabled — no manual text scanning
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

    # ── Camera control ─────────────────────────────────────────────────────────

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
            print(f"[Camera] MAIN_VIEW not found in scene — skipping")
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
            print(f"[Camera] Mode {mode} activated — {cfg['label']}")
            # Reset smooth cam state so mode 2 lerp starts fresh
            if mode == 2:
                self._chase_smooth_pos    = None
                self._chase_smooth_target = None
                if hasattr(self, '_cam_manual_override_ticks'):
                    self._cam_manual_override_ticks = 0
                print("[Camera] Chase-cam smoothing reset — "
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

    # ── Task 2: Smooth Chase Camera ────────────────────────────────────────────

    def _update_chase_cam(self, drone_pos, drone_heading_vec,
                          lerp_alpha=0.07, behind_dist=14.0,
                          above_dist=7.0, lookahead=4.0):
        """
        Manually position the Viewpoint each step for a smooth third-person chase cam.
        Includes a manual override check: if the user clicks and drags the camera
        in the Webots GUI, the supervisor suspends auto-tracking for 250 steps (~5s),
        allowing full manual viewing and rotation. Auto-tracking then resumes smoothly.
        """
        if self.viewpoint is None:
            return

        # Initialize manual override state
        if not hasattr(self, '_cam_manual_override_ticks'):
            self._cam_manual_override_ticks = 0
            self._last_written_pos = None
            self._last_written_ori = None

        # Check if user manually dragged the camera in the GUI
        try:
            actual_pos = self.viewpoint.getField("position").getSFVec3f()
            actual_ori = self.viewpoint.getField("orientation").getSFRotation()
            
            if self._last_written_pos is not None and self._last_written_ori is not None:
                pos_diff = math.sqrt(sum((a - b)**2 for a, b in zip(actual_pos, self._last_written_pos)))
                ori_diff = math.sqrt(sum((a - b)**2 for a, b in zip(actual_ori, self._last_written_ori)))
                
                # If there's a significant deviation, the user is manual-dragging
                if pos_diff > 0.05 or ori_diff > 0.05:
                    self._cam_manual_override_ticks = 250  # pause auto-chase for 250 steps (~5 seconds)
                    self._chase_smooth_pos = list(actual_pos)  # sync lerp state to prevent sudden jump
        except Exception:
            pass

        # If manual override is active, count down and do not modify the camera viewpoint
        if self._cam_manual_override_ticks > 0:
            self._cam_manual_override_ticks -= 1
            try:
                # Keep updating written states to match actual user camera position
                self._last_written_pos = self.viewpoint.getField("position").getSFVec3f()
                self._last_written_ori = self.viewpoint.getField("orientation").getSFRotation()
            except Exception:
                pass
            return

        # Normalise heading vector (may be (0,0) on first step)
        hx, hy = drone_heading_vec
        h_len = math.hypot(hx, hy)
        if h_len < 1e-4:
            hx, hy = 1.0, 0.0   # default: face East
        else:
            hx, hy = hx / h_len, hy / h_len

        # Desired camera position: behind and above the drone
        desired_cx = drone_pos[0] - hx * behind_dist
        desired_cy = drone_pos[1] - hy * behind_dist
        desired_cz = drone_pos[2] + above_dist

        # Look-at point: slightly ahead of the drone
        look_x = drone_pos[0] + hx * lookahead
        look_y = drone_pos[1] + hy * lookahead
        look_z = drone_pos[2]

        # Initialise smooth state on first call
        if not hasattr(self, '_chase_smooth_pos') or self._chase_smooth_pos is None:
            self._chase_smooth_pos    = [desired_cx, desired_cy, desired_cz]
            self._chase_smooth_target = [look_x, look_y, look_z]

        # Lerp toward desired values
        sp = self._chase_smooth_pos
        st = self._chase_smooth_target
        sp[0] += lerp_alpha * (desired_cx - sp[0])
        sp[1] += lerp_alpha * (desired_cy - sp[1])
        sp[2] += lerp_alpha * (desired_cz - sp[2])
        st[0] += lerp_alpha * (look_x - st[0])
        st[1] += lerp_alpha * (look_y - st[1])
        st[2] += lerp_alpha * (look_z - st[2])

        # Derive orientation: axis-angle from camera → look-at direction
        dx = st[0] - sp[0]
        dy = st[1] - sp[1]
        dz = st[2] - sp[2]
        dist_look = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist_look < 1e-3:
            return   # degenerate — skip this frame

        # Yaw: angle around Z axis
        yaw = math.atan2(dy, dx)
        # Pitch: angle downward from horizontal
        pitch = math.atan2(-dz, math.hypot(dx, dy))

        # tilt axis is perpendicular to heading in horizontal plane: (-hy, hx, 0)
        tilt_angle = math.radians(25) + pitch   # extra tilt toward drone
        tilt_ax = -hy
        tilt_ay =  hx
        tilt_az =  0.0
        tilt_len = math.hypot(tilt_ax, tilt_ay)
        if tilt_len < 1e-4:
            tilt_ax, tilt_ay = 0.0, 1.0
        else:
            tilt_ax /= tilt_len
            tilt_ay /= tilt_len

        final_ori = [tilt_ax, tilt_ay, tilt_az, tilt_angle + math.pi * 0.05]

        try:
            self.viewpoint.getField("position").setSFVec3f(list(sp))
            self.viewpoint.getField("orientation").setSFRotation(final_ori)
            
            # Save written state to detect next manual user drag
            self._last_written_pos = list(sp)
            self._last_written_ori = list(final_ori)
        except Exception:
            pass

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

            # Check 1: position outside the world bounds (±500m radius is generous)
            pos_magnitude = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            if pos_magnitude > 500.0:
                print(f"[Camera Recovery] Position out of bounds ({pos_magnitude:.1f}m) — "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

            # Check 2: degenerate orientation (near-zero angle = undefined rotation axis)
            # ori = [ax, ay, az, angle]; if |angle| < 1e-6 and axis not normalized → degenerate
            angle = abs(ori[3])
            axis_len = math.sqrt(ori[0]**2 + ori[1]**2 + ori[2]**2)
            if angle < 1e-6 and axis_len < 0.5:
                print(f"[Camera Recovery] Degenerate orientation detected "
                      f"(angle={angle:.2e}, axis_len={axis_len:.3f}) — "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

            # Check 3: altitude below ground (camera inside the terrain)
            if pos[2] < 0.0:
                print(f"[Camera Recovery] Camera below ground (Z={pos[2]:.1f}m) — "
                      f"Resetting to cinematic view")
                self._set_camera_mode(1)
                return

        except Exception as e:
            print(f"[Camera Recovery] Health check failed ({e}) — Resetting to cinematic view")
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

    # ── Crowd collection ───────────────────────────────────────────────────────

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

    # ── UAV patrol ─────────────────────────────────────────────────────────────

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

    # ── Bird flock (fallback if bird_controller not running) ──────────────────

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

    # ── Surveillance metrics ───────────────────────────────────────────────────

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

    # ── Main loop ──────────────────────────────────────────────────────────────

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
                    max_steps=5000,       # was 1000 → matched to UAVSwarmEnv.MAX_STEPS
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
            # ── Check for single-drone baseline navigation mode ─────────────────
            if self.baseline_nav_enabled:
                print("[UAV Controller] Starting Single-Drone Navigation Baseline...")
                self._set_camera_mode(2)  # chase-cam follows UAV_0 — best for baseline
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

                    # Periodic camera health check — recover from black-screen conditions
                    self._cam_health_counter += 1
                    if self._cam_health_counter % 500 == 0:
                        self._validate_and_recover_camera()


if __name__ == "__main__":
    controller = MultiUAVSurveillance()
    controller.run()
