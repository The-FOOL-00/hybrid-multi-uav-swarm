"""
Single Drone Navigation Baseline Controller
============================================
Phase 1: Proven engineering baseline before swarm coordination and RL.

Route  : Start (-40, 15, 15)  →  Target (40, -15, 15)
World  : worlds/single_drone_downtown.wbt  (isolated, no swarm/crowd/birds)
Pattern: Supervisor teleport — same architecture as uav_swarm_controller.py
         The supervisor sets drone position via setSFVec3f() each step.
         The Mavic2Pro robot runs uav_camera.py (propellers / sensors only).

State machine (reactive, no timers):
    NAVIGATE  →  AVOID  →  NAVIGATE  →  ARRIVED

Features implemented (Phase 1):
    ✅  Altitude stabilization  — P-controller, target 15m
    ✅  Waypoint navigation     — direct vector to target
    ✅  Obstacle avoidance      — combined repulsion from all nearby buildings
    ✅  Metrics logging         — JSON saved to experiments/single_drone/

Placeholders for future phases:
    🔲  Path planning  (A* / RRT)   →  navigation/  and  path_planning/
    🔲  PID altitude control        →  altitude_control/
    🔲  Vector-field avoidance      →  collision_avoidance/
"""

from controller import Supervisor
import math
import os
import json
import time as _time

# ── Navigation parameters ──────────────────────────────────────────────────────
START_POS       = [-40.0,  15.0, 15.0]   # metres (ENU: X=East, Y=North, Z=Up)
TARGET_POS      = [ 40.0, -15.0, 15.0]
TARGET_ALT      = 15.0          # desired cruise altitude (m)
CRUISE_SPEED    = 0.25          # m per simulation step (cruise)
SLOW_SPEED      = 0.08          # m per step when within SLOW_RADIUS of target
AVOID_SPEED     = 0.15          # m per step during obstacle avoidance

ARRIVE_RADIUS   = 2.5           # stop when within this XY distance of target (m)
SLOW_RADIUS     = 8.0           # slow down within this radius (m)
SAFETY_RADIUS   = 10.0          # trigger avoidance when obstacle within this XY dist (m)

# ── Altitude P-controller ─────────────────────────────────────────────────────
ALT_KP          = 0.06          # proportional gain (0 < Kp ≤ 1.0)
MAX_ALT_CORRECT = 0.5           # hard clamp on per-step altitude correction (m)

# ── State identifiers ─────────────────────────────────────────────────────────
STATE_NAVIGATE  = "NAVIGATE"
STATE_AVOID     = "AVOID"
STATE_ARRIVED   = "ARRIVED"

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_INTERVAL    = 125           # print status every N steps (matches swarm controller)


# ─────────────────────────────────────────────────────────────────────────────
class SingleDroneController:
    """
    Supervisor-based single drone navigator.

    Architecture mirrors MultiUAVSurveillance in uav_swarm_controller.py:
      - supervisor.getFromDef() to get node references
      - setSFVec3f() / setSFRotation() to move drone each step
      - step(timestep) main loop
    """

    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep   = int(self.supervisor.getBasicTimeStep())

        # ── Locate drone node ──────────────────────────────────────────────────
        self.drone_node  = self.supervisor.getFromDef("SOLO_DRONE")
        if self.drone_node is None:
            print("[FATAL] DEF 'SOLO_DRONE' not found in scene. "
                  "Make sure worlds/single_drone_downtown.wbt is loaded.")
            raise RuntimeError("SOLO_DRONE not found")

        self.drone_trans = self.drone_node.getField("translation")
        self.drone_rot   = self.drone_node.getField("rotation")

        # ── Collect building obstacle positions ────────────────────────────────
        # Reads from the scene at startup — same approach as _collect_buildings()
        # in uav_swarm_controller.py. Stores [x, y] pairs only (avoidance is 2-D).
        self.obstacle_xy = []
        self._collect_buildings()

        # ── State machine ──────────────────────────────────────────────────────
        self.state = STATE_NAVIGATE

        # ── Runtime metrics ────────────────────────────────────────────────────
        self.step_count          = 0
        self.sim_start_time      = None     # supervisor time at first step (s)
        self.distance_travelled  = 0.0      # cumulative 3-D distance (m)
        self.collision_count     = 0        # times within SAFETY_RADIUS of any building
        self.altitude_readings   = []       # Z values each step for stability calc
        self.step_log            = []       # sparse log for JSON export
        self.reached_target      = False
        self._prev_pos           = list(START_POS)

        self._print_banner()

    # ── Initialisation helpers ─────────────────────────────────────────────────

    def _collect_buildings(self):
        """
        Iterate scene root children and record XY centre of every building node.
        Handles SimpleBuilding, CommercialBuilding, ResidentialBuilding,
        LargeResidentialTower, Hotel, RandomBuilding, and any DEF whose name
        contains 'building', 'tower', 'office', or 'hotel' (case-insensitive).
        """
        BUILDING_TYPES = {
            "SimpleBuilding", "CommercialBuilding", "ResidentialBuilding",
            "LargeResidentialTower", "Hotel", "RandomBuilding",
        }
        BUILDING_KEYWORDS = ("building", "tower", "office", "hotel", "commercial")

        root     = self.supervisor.getRoot()
        children = root.getField("children")
        n        = children.getCount()

        for i in range(n):
            node = children.getMFNode(i)
            if node is None:
                continue
            type_name = node.getTypeName()
            def_name  = node.getDef() or ""

            if type_name in BUILDING_TYPES or \
               any(kw in def_name.lower() for kw in BUILDING_KEYWORDS):
                try:
                    pos = node.getPosition()
                    self.obstacle_xy.append([pos[0], pos[1]])
                except Exception:
                    pass

        print(f"[SingleDrone] Obstacle buildings detected: {len(self.obstacle_xy)}")

    def _print_banner(self):
        print(f"\n{'='*60}")
        print(f"  Single Drone Navigation Baseline — Phase 1")
        print(f"{'='*60}")
        print(f"  Start        : {START_POS}")
        print(f"  Target       : {TARGET_POS}")
        print(f"  Altitude     : {TARGET_ALT} m")
        print(f"  Cruise speed : {CRUISE_SPEED} m/step")
        print(f"  Safety radius: {SAFETY_RADIUS} m")
        print(f"  Buildings    : {len(self.obstacle_xy)}")
        print(f"{'='*60}")
        print(f"  BLUE  pillar = start  marker at {START_POS[:2]}")
        print(f"  ORANGE pillar= target marker at {TARGET_POS[:2]}")
        print(f"{'='*60}\n")

    # ── Math utilities ─────────────────────────────────────────────────────────

    @staticmethod
    def _dist_2d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _dist_3d(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    @staticmethod
    def _normalize_2d(vec):
        mag = math.hypot(vec[0], vec[1])
        if mag < 1e-9:
            return [0.0, 0.0]
        return [vec[0] / mag, vec[1] / mag]

    # ── Obstacle avoidance ─────────────────────────────────────────────────────

    def _compute_avoidance(self, pos):
        """
        Combined repulsion from ALL buildings within SAFETY_RADIUS.

        Returns (is_close: bool, repulsion_vec: [dx, dy] normalised).
        Each building contributes a force proportional to (1 - dist/SAFETY_RADIUS).
        This gives smooth, graduated steering rather than a binary flip.
        """
        total = [0.0, 0.0]
        any_close = False

        for bx, by in self.obstacle_xy:
            dx   = pos[0] - bx
            dy   = pos[1] - by
            dist = math.hypot(dx, dy)

            if 0.1 < dist < SAFETY_RADIUS:
                any_close = True
                # Repulsion magnitude grows as drone gets closer
                weight = 1.0 - (dist / SAFETY_RADIUS)   # 0.0 → 1.0
                unit   = self._normalize_2d([dx, dy])
                total[0] += unit[0] * weight
                total[1] += unit[1] * weight
                self.collision_count += 1                # count every proximity event

        if any_close:
            return True, self._normalize_2d(total)
        return False, [0.0, 0.0]

    # ── Altitude control ───────────────────────────────────────────────────────

    def _altitude_correction(self, current_z):
        """
        Simple proportional correction.
        """
        if current_z > TARGET_ALT + 0.2:
            return -0.1  # move down slightly
        elif current_z < TARGET_ALT - 0.2:
            return 0.1   # move up slightly
        return 0.0

    # ── Main step ──────────────────────────────────────────────────────────────

    def update(self):
        """
        Execute one simulation step.
        Returns True to continue, False to stop the loop.
        """
        current_pos = list(self.drone_trans.getSFVec3f())

        # First-step initialisation
        if self.step_count == 0:
            self.sim_start_time = self.supervisor.getTime()
            self._prev_pos      = list(current_pos)

        self.step_count += 1

        # ── Stay put once arrived ──────────────────────────────────────────────
        if self.state == STATE_ARRIVED:
            return True

        # ── Arrival check ──────────────────────────────────────────────────────
        dist_2d = self._dist_2d(current_pos, TARGET_POS)
        if dist_2d < ARRIVE_RADIUS:
            self._on_arrived(current_pos)
            return True

        # ── Obstacle check  →  decide state ───────────────────────────────────
        is_close, repulsion = self._compute_avoidance(current_pos)

        new_xy = [current_pos[0], current_pos[1]]

        if is_close:
            # ── AVOID ─────────────────────────────────────────────────────────
            self.state = STATE_AVOID

            # To-target direction (keeps drone making progress)
            to_target = self._normalize_2d([
                TARGET_POS[0] - current_pos[0],
                TARGET_POS[1] - current_pos[1],
            ])

            # Temporary steering offset (tangent to obstacle)
            tangent = [-repulsion[1], repulsion[0]]
            if (tangent[0]*to_target[0] + tangent[1]*to_target[1]) < 0:
                tangent = [repulsion[1], -repulsion[0]]

            # Goal attraction dominates, avoidance temporarily steers around
            blend = self._normalize_2d([
                (to_target[0] * 0.7) + (tangent[0] * 0.3),
                (to_target[1] * 0.7) + (tangent[1] * 0.3),
            ])

            new_xy[0] += blend[0] * AVOID_SPEED
            new_xy[1] += blend[1] * AVOID_SPEED

        else:
            # ── NAVIGATE ──────────────────────────────────────────────────────
            self.state  = STATE_NAVIGATE
            speed       = SLOW_SPEED if dist_2d < SLOW_RADIUS else CRUISE_SPEED
            to_target   = self._normalize_2d([
                TARGET_POS[0] - current_pos[0],
                TARGET_POS[1] - current_pos[1],
            ])

            new_xy[0] += to_target[0] * speed
            new_xy[1] += to_target[1] * speed

        # ── Altitude stabilisation (Proportional) ─────────────────────────────
        new_z = current_pos[2] + self._altitude_correction(current_pos[2])
        
        # Reset physics to kill accumulated velocity (stops upward drift)
        self.drone_node.resetPhysics()

        new_pos = [new_xy[0], new_xy[1], new_z]

        # ── Orientation — face movement direction ──────────────────────────────
        dx = new_pos[0] - current_pos[0]
        dy = new_pos[1] - current_pos[1]
        if math.hypot(dx, dy) > 1e-5:
            yaw = math.atan2(dy, dx)
            self.drone_rot.setSFRotation([0.0, 0.0, 1.0, yaw])

        # ── Apply translation ──────────────────────────────────────────────────
        self.drone_trans.setSFVec3f(new_pos)

        # ── Accumulate distance & altitude history ─────────────────────────────
        self.distance_travelled += self._dist_3d(new_pos, self._prev_pos)
        self._prev_pos           = list(new_pos)
        self.altitude_readings.append(new_pos[2])

        # ── Periodic console status ────────────────────────────────────────────
        if self.step_count % LOG_INTERVAL == 0:
            self._print_status(new_pos, dist_2d)

        # ── Sparse step log (every 10 steps) ──────────────────────────────────
        if self.step_count % 10 == 0:
            self.step_log.append({
                "step":           self.step_count,
                "x":              round(new_pos[0], 2),
                "y":              round(new_pos[1], 2),
                "z":              round(new_pos[2], 2),
                "state":          self.state,
                "dist_to_target": round(dist_2d, 2),
            })

        return True

    # ── Arrival handler ────────────────────────────────────────────────────────

    def _on_arrived(self, pos):
        """Called exactly once when the drone reaches the target radius."""
        self.state          = STATE_ARRIVED
        self.reached_target = True

        elapsed = self.supervisor.getTime() - (self.sim_start_time or 0.0)
        alt_std = self._altitude_std()

        print(f"\n{'='*60}")
        print(f"  ✓  TARGET REACHED!")
        print(f"{'='*60}")
        print(f"  Final position  : ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        print(f"  Travel time     : {elapsed:.2f} s  (simulation time)")
        print(f"  Distance        : {self.distance_travelled:.2f} m")
        print(f"  Proximity events: {self.collision_count}")
        print(f"  Altitude std-dev: {alt_std:.4f} m")
        print(f"{'='*60}\n")

        self._save_metrics(elapsed, alt_std)

    # ── Metrics utilities ──────────────────────────────────────────────────────

    def _altitude_std(self):
        readings = self.altitude_readings
        if len(readings) < 2:
            return 0.0
        mean     = sum(readings) / len(readings)
        variance = sum((a - mean) ** 2 for a in readings) / len(readings)
        return math.sqrt(variance)

    def _print_status(self, pos, dist_2d):
        print(
            f"[Step {self.step_count:>7}]  "
            f"State={self.state:<9}  "
            f"Pos=({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:5.2f})  "
            f"DistToTarget={dist_2d:6.2f} m  "
            f"Proximity={self.collision_count}"
        )

    def _save_metrics(self, travel_time=None, alt_std=None):
        """Save metrics JSON to experiments/single_drone/<timestamp>/metrics.json."""
        if travel_time is None:
            travel_time = (
                self.supervisor.getTime() - self.sim_start_time
                if self.sim_start_time is not None else 0.0
            )
        if alt_std is None:
            alt_std = self._altitude_std()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir   = os.path.abspath(os.path.join(script_dir, "..", ".."))
        timestamp  = _time.strftime("%Y%m%d_%H%M%S")
        out_dir    = os.path.join(root_dir, "experiments", "single_drone", timestamp)
        os.makedirs(out_dir, exist_ok=True)

        metrics = {
            "phase":                "Phase 1 — Single Drone Navigation Baseline",
            "world":                "worlds/single_drone_downtown.wbt",
            "start":                START_POS,
            "target":               TARGET_POS,
            "target_altitude_m":    TARGET_ALT,
            "safety_radius_m":      SAFETY_RADIUS,
            # ── Results ──────────────────────────────────────────────────────
            "reached_target":       self.reached_target,
            "travel_time_s":        round(travel_time, 3),
            "distance_travelled_m": round(self.distance_travelled, 3),
            "collision_count":      self.collision_count,
            "altitude_stability_std_m": round(alt_std, 5),
            "total_steps":          self.step_count,
            # ── Sparse trajectory log ─────────────────────────────────────────
            "step_log":             self.step_log,
        }

        out_path = os.path.join(out_dir, "metrics.json")
        with open(out_path, "w") as fp:
            json.dump(metrics, fp, indent=4)

        print(f"[SingleDrone] Metrics saved → {out_path}")

    # ── Main loop ──────────────────────────────────────────────────────────────

    def run(self):
        """Blocking simulation loop. Saves metrics on exit regardless of outcome."""
        print("[SingleDrone] Supervisor online. Starting navigation...\n")
        try:
            while self.supervisor.step(self.timestep) != -1:
                self.update()
        except KeyboardInterrupt:
            print("\n[SingleDrone] Interrupted by user.")
        finally:
            # Save metrics even if simulation was stopped before arrival
            if not self.reached_target and self.step_count > 0:
                print("[SingleDrone] Saving partial run metrics...")
                self._save_metrics()


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    controller = SingleDroneController()
    controller.run()
