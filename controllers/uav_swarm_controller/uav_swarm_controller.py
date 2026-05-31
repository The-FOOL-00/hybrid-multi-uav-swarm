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

        # Gather building positions
        self.building_nodes = []
        self._collect_buildings()

        self.step_count = 0
        self.coverage_log = []

        # ── Camera control ─────────────────────────────────────────────────────
        self.viewpoint = self.supervisor.getFromDef("MAIN_VIEW")
        self.cam_mode = 1
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
        config_path = os.path.join(root_dir, "configs", "environment_config.yaml")
        if os.path.isfile(config_path):
            try:
                with open(config_path, "r") as f:
                    for line in f:
                        if "enabled:" in line and "true" in line.lower():
                            self.rl_enabled = True
                            break
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
            if type_name in ("Pedestrian", "CrowdAgent", "Robot"):
                def_name = node.getDef()
                if def_name and ("ped" in def_name.lower() or
                                 "pedestrian" in def_name.lower() or
                                 "crowd" in def_name.lower() or
                                 "worker" in def_name.lower()):
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
        
        if self.rl_enabled:
            print("\n" + "=" * 58)
            print("  NATIVE GYM TRAINING LOOP ACTIVE (Phase 1 Milestone)")
            print("=" * 58)
            # Force top-down static overview at startup to prevent follow-camera jitter during random actions!
            self._set_camera_mode(3)
            obs, _ = self.env.reset()
            step_count = 0
            while True:
                # Process keyboard inputs (keys 1-3 for camera modes, keys 4-8 for drone focus)
                self._handle_keyboard()
                
                # Sample random continuous actions: 15 floats in [-1.0, 1.0]
                import numpy as np
                action = np.random.uniform(-1.0, 1.0, size=15).astype(np.float32)
                
                obs, reward, terminated, truncated, info = self.env.step(action)
                step_count += 1
                
                if step_count % 100 == 0:
                    print(f"  [Gym Step] {step_count:>4}/1000 completed successfully.")
                    
                    # Phase 2 Observation Space Audit (Print first 8 features for UAV_0)
                    u0 = obs[:8]
                    print(f"    [UAV_0 Observation Audit]:")
                    print(f"      - Pos (x, y, z):  ({u0[0]:.2f}, {u0[1]:.2f}, {u0[2]:.2f})")
                    print(f"      - Nearest UAV:     {u0[3]:.2f} (~{u0[3]*50:.1f}m)")
                    print(f"      - Nearest Bldg:    {u0[4]:.2f} (~{u0[4]*50:.1f}m)")
                    print(f"      - Crowd Vector:   ({u0[5]:.2f}, {u0[6]:.2f})")
                    print(f"      - Grid Coverage:   {u0[7]:.2f} ({u0[7]*100:.1f}%)")
                    
                if terminated or truncated:
                    print(f"\n🎉 Milestone 1 Achieved: {step_count} steps completed without error!")
                    print("Soft-resetting and restarting loop...")
                    obs, _ = self.env.reset()
                    step_count = 0
        else:
            # Rule-based baseline patrol
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
