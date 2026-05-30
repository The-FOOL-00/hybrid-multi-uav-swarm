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
        "orientation": [0.0, 0.0, 1.0, 0.0],  # identity = look along -Z = straight down
        "follow":      "",
        "followType":  "None",
    },
}


class MultiUAVSurveillance:

    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.num_uavs = 5
        self.patrol_radius = 20.0
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

        self.step_count = 0
        self.coverage_log = []

        # ── Camera control ─────────────────────────────────────────────────────
        self.viewpoint = self.supervisor.getFromDef("MAIN_VIEW")
        self.cam_mode = 1
        try:
            self.keyboard = self.supervisor.getKeyboard()
            self.keyboard.enable(self.timestep)
            self._kb_available = True
        except Exception:
            self._kb_available = False

        print(f"[UAV Controller] UAVs={len(self.uavs)}, "
              f"Birds={len(self.birds)}, Crowd={len(self.crowd_nodes)}")
        self._print_camera_help()

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
            print(f"[Camera] Mode {mode}: {cfg['label']}")
        except Exception as e:
            print(f"[Camera] Could not switch mode: {e}")

    def _handle_keyboard(self):
        """Read keyboard and switch camera mode on 1/2/3 keys."""
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
            if type_name in ("Pedestrian", "CrowdAgent", "Robot"):
                def_name = node.getDef()
                if def_name and ("ped" in def_name.lower() or
                                 "pedestrian" in def_name.lower() or
                                 "crowd" in def_name.lower() or
                                 "worker" in def_name.lower()):
                    self.crowd_nodes.append(node)

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
        while self.supervisor.step(self.timestep) != -1:
            self.step_count += 1
            self._handle_keyboard()      # camera mode switching (GUI only)
            self.update_uavs()
            if self.birds:
                self.update_birds()
            self.log_metrics()


if __name__ == "__main__":
    controller = MultiUAVSurveillance()
    controller.run()
