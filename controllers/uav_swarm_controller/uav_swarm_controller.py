"""
Multi-UAV Supervisor Controller
POMDP-based decentralized surveillance for smart-city crowd monitoring.
Coordinate system: ENU (East=X, North=Y, Up=Z)

Camera modes (keyboard keys 1 / 2 / 3 in GUI mode):
  1 - Cinematic city view  : fixed SE-corner position, pans to track UAV_0
  2 - Tracking shot        : chase-cam follows UAV_0 from behind
  3 - Top-down overview    : bird's-eye view, all 5 drones visible
"""
from controller import Supervisor
import math
import random
import os
import yaml


# ══════════════════════════════════════════════════════════════════════════════
# SINGLE-DRONE NAVIGATION BASELINE
# Phase 1 of the hybrid-multi-UAV-swarm development pipeline.
# Deterministic start → target mission for engineering baseline validation.
# ────────────────────────────────────────────────────────────────────────────────
class SingleDroneNavigation:
    """
    Simple deterministic waypoint navigation for UAV_0.

    Algorithm:
        1. TAKEOFF  — teleport UAV_0 to start_position.
        2. NAVIGATE — each step compute:
               direction = normalize(target - current)
               new_pos   = current + direction * speed
               new_pos.z = altitude  # lock altitude
           Move drone there via setSFVec3f (supervisor teleport, same as
           the existing patrol mode — no physics engine involvement).
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
        # FIX-1: speed reduced to 0.3 m/step for smooth visual motion.
        # At 8ms/step this equals 37.5 m/s supervisor-teleport velocity.
        # Mission distance ≈85.4m → ~285 visible steps across the city.
        self.cruise_speed  = float(cfg.get("cruise_speed", 0.3))
        self.arrival_radius= float(cfg.get("arrival_radius", 3.0))
        self.altitude      = float(cfg.get("altitude", 15.0))
        self.log_interval  = int(cfg.get("log_interval_steps", 100))

        # UAV_0 node references (index 0 in parent lists)
        if len(parent.uav_trans) == 0:
            raise RuntimeError("[SingleDroneNav] UAV_0 not found in scene!")
        self.uav_tf = parent.uav_trans[0]   # translation field
        self.uav_rf = parent.uav_rot[0]     # rotation field
        self.uav_node = parent.uavs[0]      # Mavic2Pro node

        self.state       = self.STATE_TAKEOFF
        self.step_count  = 0
        self.arrived_reported = False

        # Lightweight obstacle list for collision testing (radius roughly approximates footprint)
        self.test_buildings = [
            {"name": "office_nw2", "x": -36.0, "y": 48.0, "r": 8.0},
            {"name": "tower_c",    "x": 24.0,  "y": 46.0, "r": 7.0},
            {"name": "tower_b",    "x": 36.0,  "y": 48.0, "r": 8.5},
            {"name": "tower_a",    "x": 24.0,  "y": 58.0, "r": 8.0},
            {"name": "office_nw1", "x": -26.0, "y": 60.0, "r": 11.0}
        ]
        self.last_warning_step = -999

        # Park UAV_1-4 at their initial positions so they don't drift
        self._park_inactive_uavs()

        self._print_banner()

    # ── Setup helpers ─────────────────────────────────────────────────────

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
        print("\n" + "=" * 64)
        print("  SINGLE DRONE NAVIGATION BASELINE  (Phase 1)")
        print("  NO RL  |  NO RANDOM PATROL  |  NO SWARM")
        print("  -" * 32)
        print(f"  Active drone : UAV_0")
        print(f"  Start        : {self.start_pos}")
        print(f"  Target       : {self.target_pos}")
        print(f"  Altitude     : {self.altitude} m (constant)")
        print(f"  Speed        : {self.cruise_speed} m/step")
        print(f"  Arrival zone : {self.arrival_radius} m radius")
        print("  States       : TAKEOFF → NAVIGATE → ARRIVED")
        print("=" * 64 + "\n")

    # ── Navigation logic ─────────────────────────────────────────────────

    @staticmethod
    def _dist3(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    @staticmethod
    def _dist2(a, b):
        """Horizontal distance (XY plane only)."""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _move_toward_target(self):
        """
        Compute one step of vector steering.

        direction = normalize(target_xy - current_xy)
        new_xy    = current_xy + direction * cruise_speed
        new_z     = altitude  (altitude lock — no climbing/falling)

        Returns the new [x, y, z] position.
        """
        cur = list(self.uav_tf.getSFVec3f())
        tx, ty = self.target_pos[0], self.target_pos[1]
        dx, dy = tx - cur[0], ty - cur[1]
        horiz_dist = math.hypot(dx, dy)

        if horiz_dist < 1e-4:
            # Already aligned horizontally — hold position
            return [tx, ty, self.altitude]

        # Normalise and scale
        scale = min(self.cruise_speed, horiz_dist)  # don’t overshoot
        ux, uy = dx / horiz_dist, dy / horiz_dist
        new_x = cur[0] + ux * scale
        new_y = cur[1] + uy * scale
        new_z = self.altitude  # altitude lock (simple rule-based, no PID)

        return [new_x, new_y, new_z]

    def _orient_toward(self, prev_pos, new_pos):
        """
        Rotate UAV_0 in the direction of travel (yaw only, Z-up ENU).
        Uses the same axis-angle [0,0,1,yaw] convention as the patrol code.
        """
        dx = new_pos[0] - prev_pos[0]
        dy = new_pos[1] - prev_pos[1]
        if math.hypot(dx, dy) > 1e-4:
            yaw = math.atan2(dy, dx)
            self.uav_rf.setSFRotation([0.0, 0.0, 1.0, yaw])

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
            cur = list(self.uav_tf.getSFVec3f())
            dist_to_target = self._dist3(cur, self.target_pos)

            if dist_to_target <= self.arrival_radius:
                # Mission complete — snap exactly to target, hold
                self.uav_tf.setSFVec3f(self.target_pos)
                self.state = self.STATE_ARRIVED
                return True

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

            # Lightweight Collision Check
            closest_name, min_dist = self._check_collision_distance(cur)
            if min_dist < 4.0 and (self.step_count - self.last_warning_step) > 20:
                print(f"  [WARNING] Potential collision likely with {closest_name}! (dist: {min_dist:.1f}m)")
                self.last_warning_step = self.step_count
            elif min_dist < 0.0 and (self.step_count - self.last_warning_step) > 20:
                print(f"  [CRITICAL] Drone is INSIDE building {closest_name}!")
                self.last_warning_step = self.step_count

            # Periodic console debug log
            if self.step_count % self.log_interval == 0:
                cur_after = list(self.uav_tf.getSFVec3f())
                self._log_status(cur_after, dist_to_target, closest_name, min_dist)

            return True

        # ── STATE: ARRIVED ───────────────────────────────────────────
        if self.state == self.STATE_ARRIVED:
            if not self.arrived_reported:
                cur = list(self.uav_tf.getSFVec3f())
                print("\n" + "=" * 64)
                print("  ✓ MISSION COMPLETE — UAV_0 ARRIVED AT TARGET")
                print(f"  Final position : ({cur[0]:.2f}, {cur[1]:.2f}, {cur[2]:.2f})")
                print(f"  Target         : {self.target_pos}")
                print(f"  Steps taken    : {self.step_count}")
                print(f"  State          : ARRIVED (drone holding position)")
                print("=" * 64 + "\n")
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

    def _check_collision_distance(self, pos):
        """Returns (closest_building_name, surface_distance)."""
        min_dist = 999.0
        closest = None
        for b in self.test_buildings:
            dist = math.hypot(pos[0] - b["x"], pos[1] - b["y"])
            surface_dist = dist - b["r"]
            if surface_dist < min_dist:
                min_dist = surface_dist
                closest = b["name"]
        return closest, min_dist

    def _log_status(self, pos, dist_to_target, closest_bldg=None, bldg_dist=999.0):
        """Console debug log shown every log_interval steps."""
        print(f"\n[► Step {self.step_count:>6}]  State: {self.state}")
        print(f"  UAV_0 position : ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:5.2f}) m")
        print(f"  Dist to target : {dist_to_target:7.2f} m")
        print(f"  Altitude       : {pos[2]:5.2f} m  (target={self.altitude:.1f} m)")
        if closest_bldg:
            print(f"  [CollisionTest] Nearest obstacle ({closest_bldg}): {bldg_dist:.1f}m")
        print(f"  Target         : ({self.target_pos[0]:.1f}, {self.target_pos[1]:.1f}, {self.target_pos[2]:.1f})")


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
        "label":       "Tracking shot (chase UAV_0)",
        "position":    [80.0, -100.0, 75.0],   # irrelevant for Tracking Shot
        "orientation": [-0.16, 0.22, 0.96, 1.32],
        "follow":      "UAV_0",
        "followType":  "Tracking Shot",
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
