"""
UAVSwarmEnv — OpenAI Gym-compatible environment stub for Multi-UAV Swarm.

Status: SKELETON — NOT YET FUNCTIONAL.

Architecture Overview
---------------------
This environment will wrap the Webots simulation via the Supervisor API.
The RL agent will observe UAV/crowd states and issue action commands that
the uav_swarm_controller translates into position updates.

Coordinate System
-----------------
ENU (East=X, North=Y, Up=Z). Arena: ±90 m in X/Y. UAV altitude: 10–15 m (Z).

Observation Space (planned, per UAV)
-------------------------------------
    [x, y, z,                    # UAV position (normalized)
     crowd_centroid_x, y,        # Crowd center of mass
     coverage_percent,           # Current coverage %
     nearest_uav_dist,           # Min distance to other UAVs
     nearest_obstacle_dist]      # Min distance to buildings

Action Space (planned, per UAV)
---------------------------------
    Continuous: [delta_x, delta_y, delta_z]   # position delta, clipped ±2m

Reward
------
    See rl/rewards/reward_functions.py

Integration Point
-----------------
    The Webots supervisor (controllers/uav_swarm_controller/) will be
    extended to accept action commands via a shared IPC mechanism (e.g.,
    file socket or stdin) when RL mode is enabled. GUI and headless modes
    both supported via run_simulation.py --headless.

Usage (future):
    env = UAVSwarmEnv(scenario="downtown", headless=True)
    obs = env.reset()
    obs, reward, done, info = env.step(action)
    env.close()
"""

# ─── Dependencies (install when RL training begins) ──────────────────────────
# pip install gymnasium numpy pyyaml
# gymnasium replaces the old openai/gym package

try:
    import gymnasium as gym
    from gymnasium import spaces
    GYM_AVAILABLE = True
except ImportError:
    GYM_AVAILABLE = False

import os
import math
import subprocess
import numpy as np


class UAVSwarmEnv:
    """
    Stub Gym environment for the Hybrid Multi-UAV Swarm simulation.

    This class defines the full interface; internal logic is NOT implemented
    yet. Raises NotImplementedError on step/reset until the Webots IPC
    bridge is complete.
    """

    # ── Constants ─────────────────────────────────────────────────────────────
    NUM_UAVS = 5
    ARENA_HALF = 90.0          # metres
    ALT_MIN = 10.0
    ALT_MAX = 15.0
    MAX_DELTA = 2.0            # metres per step

    SCENARIOS = ["downtown", "event", "residential", "mixed", "industrial"]

    def __init__(self, supervisor_controller):
        """
        Args:
            supervisor_controller: The active MultiUAVSurveillance instance.
        """
        self.controller = supervisor_controller
        self.supervisor = supervisor_controller.supervisor
        self.timestep = supervisor_controller.timestep
        self.scenario = "downtown"
        
        self._step_count = 0

        # Capture starting translations and rotations for soft reset
        self.initial_translations = []
        self.initial_rotations = []
        for tf, rf in zip(self.controller.uav_trans, self.controller.uav_rot):
            self.initial_translations.append(list(tf.getSFVec3f()))
            self.initial_rotations.append(list(rf.getSFRotation()))

        # ── Observation space (flat, per-UAV concatenated) ────────────────
        # [x, y, z, nn_dist, obs_dist, crowd_dx, crowd_dy, coverage] × NUM_UAVS
        obs_dim = 8 * self.NUM_UAVS
        if GYM_AVAILABLE:
            self.observation_space = spaces.Box(
                low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32
            )
            # Continuous action: delta_x, delta_y, delta_z per UAV
            self.action_space = spaces.Box(
                low=-1.0, high=1.0,
                shape=(3 * self.NUM_UAVS,),
                dtype=np.float32
            )

    # ── Core Gym Interface ─────────────────────────────────────────────────

    def reset(self):
        """
        Soft-resets all UAVs to their initial positions and orientations.
        Returns the initial observation and info dict.
        """
        self._step_count = 0
        self.controller.prev_targets = [None] * self.NUM_UAVS

        # Teleport all UAVs back to start and reset physics
        for i, (tf, rf) in enumerate(zip(self.controller.uav_trans, self.controller.uav_rot)):
            tf.setSFVec3f(self.initial_translations[i])
            rf.setSFRotation(self.initial_rotations[i])
            try:
                self.controller.uavs[i].resetPhysics()
            except Exception:
                pass

        # Step once to apply changes in the physics engine
        self.supervisor.step(self.timestep)
        
        # Read keyboard input immediately after step advances to refresh the queue
        if hasattr(self.controller, '_handle_keyboard'):
            self.controller._handle_keyboard()

        # Return initial observation and empty info dict
        obs = self._get_observation()
        return obs, {}

    def step(self, action):
        """
        Apply delta actions, advance simulation physics by one step, and return feedback.

        Args:
            action: np.ndarray of shape (15,) — normalized delta_x, delta_y, delta_z for 5 UAVs.

        Returns:
            obs, reward, terminated, truncated, info
        """
        # De-normalize actions: scale from [-1.0, 1.0] to [-2.0, 2.0] meters displacement
        action = np.clip(action, -1.0, 1.0)
        
        for i in range(self.NUM_UAVS):
            tf = self.controller.uav_trans[i]
            rf = self.controller.uav_rot[i]
            
            # Get current 3D position
            curr_pos = tf.getSFVec3f()
            
            # Map index to action slice
            idx = i * 3
            dx = action[idx] * self.MAX_DELTA
            dy = action[idx + 1] * self.MAX_DELTA
            dz = action[idx + 2] * self.MAX_DELTA
            
            # Compute new target position
            new_x = curr_pos[0] + dx
            new_y = curr_pos[1] + dy
            new_z = curr_pos[2] + dz
            
            # Clamp to safety limits
            new_x = max(-self.ARENA_HALF, min(self.ARENA_HALF, new_x))
            new_y = max(-self.ARENA_HALF, min(self.ARENA_HALF, new_y))
            new_z = max(self.ALT_MIN, min(self.ALT_MAX, new_z))
            
            # Teleport UAV
            target = [new_x, new_y, new_z]
            tf.setSFVec3f(target)
            
            # Orient the drone smoothly in the direction of displacement using a Low-Pass Filter
            if math.hypot(dx, dy) > 1e-4:
                curr_rot = rf.getSFRotation()
                curr_yaw = curr_rot[3]
                target_yaw = math.atan2(dy, dx)
                
                # Compute shortest angular distance to prevent wild spins
                diff = target_yaw - curr_yaw
                diff = math.atan2(math.sin(diff), math.cos(diff))
                
                # Smooth interpolation (alpha = 0.1) to eliminate jitter/stutter
                new_yaw = curr_yaw + 0.1 * diff
                rf.setSFRotation([0.0, 0.0, 1.0, new_yaw])
                
            # Reset physics to completely zero out linear/angular momentum from teleportation
            try:
                self.controller.uavs[i].resetPhysics()
            except Exception:
                pass
                
        # Advance simulation physics by exactly one timestep
        self.supervisor.step(self.timestep)
        
        # Read keyboard input immediately after step advances to refresh the queue
        if hasattr(self.controller, '_handle_keyboard'):
            self.controller._handle_keyboard()
            
        self._step_count += 1
        
        # Check termination (Phase 1 milestone: 1,000 steps)
        terminated = self._step_count >= 1000
        truncated = False
        
        # Return outputs
        obs = self._get_observation()
        reward = 1.0
        info = {}
        
        return obs, reward, terminated, truncated, info

    def render(self):
        """Not needed — Webots handles its own rendering."""
        pass

    def close(self):
        """Terminate the Webots subprocess if running."""
        if self._webots_proc is not None:
            self._webots_proc.terminate()
            self._webots_proc = None

    # ── Internal helpers (stubs) ───────────────────────────────────────────

    def _launch_webots(self):
        """Launch Webots in headless or GUI mode via run_simulation.py."""
        script = os.path.join(
            os.path.dirname(__file__), "..", "..", "run_simulation.py"
        )
        cmd = ["python", script, "--scenario", self.scenario]
        if self.headless:
            cmd.append("--headless")
        self._webots_proc = subprocess.Popen(cmd)

    def _get_observation(self):
        """
        Gathers normalized coordinates, neighbor distances, building proximity,
        relative crowd centroid vector, and coverage percentage for all 5 UAVs.
        Returns a flat numpy array of shape (40,).
        """
        obs = []
        
        # Calculate crowd centroid
        crowd_cx, crowd_cy = self.controller._crowd_centroid()
        
        # Compute global coverage percentage (normalized to [0.0, 1.0])
        coverage = self.controller.compute_coverage() / 100.0
        
        for i in range(self.NUM_UAVS):
            tf = self.controller.uav_trans[i]
            pos = tf.getSFVec3f()
            
            # 1-3. Normalized absolute coordinates
            nx = pos[0] / self.ARENA_HALF
            ny = pos[1] / self.ARENA_HALF
            nz = (pos[2] - self.ALT_MIN) / (self.ALT_MAX - self.ALT_MIN) * 2.0 - 1.0
            
            # 4. Normalized distance to nearest neighbor UAV
            min_nn_dist = float("inf")
            for j in range(self.NUM_UAVS):
                if i == j:
                    continue
                tf_other = self.controller.uav_trans[j]
                pos_other = tf_other.getSFVec3f()
                dist = math.sqrt(sum((pi - pj) ** 2 for pi, pj in zip(pos, pos_other)))
                if dist < min_nn_dist:
                    min_nn_dist = dist
            norm_nn_dist = min(min_nn_dist / 50.0, 1.0)  # capped at 50m
            
            # 5. Normalized distance to nearest building
            min_obs_dist = float("inf")
            if hasattr(self.controller, 'building_nodes'):
                for b_node in self.controller.building_nodes:
                    try:
                        b_pos = b_node.getPosition()
                        dist = math.hypot(pos[0] - b_pos[0], pos[1] - b_pos[1])
                        if dist < min_obs_dist:
                            min_obs_dist = dist
                    except Exception:
                        pass
            if min_obs_dist == float("inf"):
                norm_obs_dist = 1.0
            else:
                norm_obs_dist = min(min_obs_dist / 50.0, 1.0)  # capped at 50m
                
            # 6-7. Relative crowd centroid vector
            relative_cx = (crowd_cx - pos[0]) / self.ARENA_HALF
            relative_cy = (crowd_cy - pos[1]) / self.ARENA_HALF
            relative_cx = max(-1.0, min(1.0, relative_cx))
            relative_cy = max(-1.0, min(1.0, relative_cy))
            
            # 8. Global coverage
            # obs vector layout: [x, y, z, nn_dist, obs_dist, crowd_dx, crowd_dy, coverage]
            obs.extend([nx, ny, nz, norm_nn_dist, norm_obs_dist, relative_cx, relative_cy, coverage])
            
        return np.array(obs, dtype=np.float32)

    def _normalize_position(self, x, y, z):
        """Normalize ENU position to [-1, 1] range."""
        nx = x / self.ARENA_HALF
        ny = y / self.ARENA_HALF
        nz = (z - self.ALT_MIN) / (self.ALT_MAX - self.ALT_MIN) * 2 - 1
        return nx, ny, nz

    def _denormalize_action(self, action_vec):
        """Convert normalized action [-1,1] to actual metre deltas."""
        return [a * self.MAX_DELTA for a in action_vec]

    def __repr__(self):
        return (
            f"UAVSwarmEnv(scenario={self.scenario!r}, "
            f"headless={self.headless}, "
            f"uavs={self.NUM_UAVS})"
        )
