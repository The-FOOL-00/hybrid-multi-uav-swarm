"""
trainer.py — Training loop orchestrator for Multi-UAV RL experiments using SB3 PPO.

This script implements:
1. SwarmTrainingCallback: Logs episode metrics (rewards, coverage, TRR, safety collisions) to standard out, tensorboard, and metrics JSON files.
2. Trainer: Orchestrates the SB3 PPO training loop for 500,000 steps, supporting visual GUI checks first, then switching to headless production.
"""

import os
import json
import time
import argparse
from datetime import datetime
import numpy as np

try:
    import stable_baselines3 as sb3
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False


class SwarmTrainingCallback(BaseCallback):
    """
    Custom callback to log episode-level metrics (Coverage %, TRR %, safety collisions)
    to tensorboard and metrics files.
    """
    def __init__(self, check_freq: int, save_path: str, metrics_dir: str, log_interval: int = 10, verbose=1, headless: bool = True):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.save_path = save_path
        self.metrics_dir = metrics_dir
        self.log_interval = log_interval
        self.headless = headless
        self.episode_count = 0
        self.episode_rewards = []
        self.episode_coverages = []
        self.episode_trrs = []
        self.episode_collisions = []
        self.episode_oobs = []
        self.current_ep_reward = 0.0
        self.metrics_history = []
        # Timing for steps/sec calculation
        self._training_start_time = time.time()
        self._ep_start_time = time.time()

    def _on_step(self) -> bool:
        # 1. Accumulate step reward for current episode
        rewards = self.locals.get("rewards")
        if rewards is not None:
            self.current_ep_reward += rewards[0]

        # Live step-by-step progress logging (prints every 100 steps)
        infos = self.locals.get("infos")
        if infos is not None and len(infos) > 0:
            step_info = infos[0]
            current_step = step_info.get("step", 0)
            if current_step > 0 and current_step % 100 == 0:
                current_cov = step_info.get("coverage", 0.0)
                # Dynamically retrieve max horizon from the wrapped env
                try:
                    max_h = self.training_env.envs[0].env.MAX_STEPS
                except Exception:
                    max_h = 5000
                print(f" -> Episode Step {current_step:>4}/{max_h} | Current Coverage: {current_cov:5.2f}%")
                
                # Retrieve raw 3D coordinates (X, Y, Z) from environment for visual verification
                try:
                    raw_env = self.training_env.envs[0].env
                    coords = [raw_env.controller.uav_trans[i].getSFVec3f() for i in range(5)]
                    coord_strs = ", ".join(f"UAV_{i}:({p[0]:5.1f},{p[1]:5.1f},{p[2]:4.1f})" for i, p in enumerate(coords))
                    print(f"    Coordinates: {coord_strs}")
                except Exception:
                    pass

        # 2. Check if the episode ended
        dones = self.locals.get("dones")
        infos = self.locals.get("infos")
        
        if dones is not None and dones[0] and infos is not None:
            self.episode_count += 1
            info = infos[0]
            
            cov = info.get("coverage", 0.0)
            trr = info.get("trr", 0.0)
            coll = info.get("is_collision", 0)
            oob = info.get("is_out_of_bounds", 0)
            
            self.episode_rewards.append(self.current_ep_reward)
            self.episode_coverages.append(cov)
            self.episode_trrs.append(trr)
            self.episode_collisions.append(coll)
            self.episode_oobs.append(oob)
            
            # Compute episode wall-clock duration and steps/sec
            ep_end_time = time.time()
            ep_duration_sec = ep_end_time - self._ep_start_time
            elapsed_total = ep_end_time - self._training_start_time
            steps_per_sec = self.n_calls / elapsed_total if elapsed_total > 0 else 0.0
            
            # Print episodic progress at intervals
            if self.episode_count % self.log_interval == 0:
                avg_r = np.mean(self.episode_rewards[-self.log_interval:])
                avg_c = np.mean(self.episode_coverages[-self.log_interval:])
                avg_t = np.mean(self.episode_trrs[-self.log_interval:])
                avg_coll = np.sum(self.episode_collisions[-self.log_interval:])
                print(
                    f"[Trainer] Ep {self.episode_count:>4} | "
                    f"AvgReward: {avg_r:>7.2f} | "
                    f"AvgCoverage: {avg_c:>5.1f}% | "
                    f"AvgTRR: {avg_t:>5.1f}% | "
                    f"Collisions: {avg_coll:>2.0f} | "
                    f"Steps/sec: {steps_per_sec:>5.1f}"
                )
                
            # Log custom metrics to TensorBoard logger
            self.logger.record("episode/reward", self.current_ep_reward)
            self.logger.record("episode/coverage", cov)
            self.logger.record("episode/trr", trr)
            self.logger.record("episode/is_collision", coll)
            self.logger.record("episode/is_out_of_bounds", oob)
            self.logger.record("episode/steps_per_sec", steps_per_sec)
            
            # Save historical metrics record to JSON file
            record = {
                "episode": self.episode_count,
                "reward": float(self.current_ep_reward),
                "coverage": float(cov),
                "trr": float(trr),
                "is_collision": int(coll),
                "is_out_of_bounds": int(oob),
                "episode_duration_sec": round(ep_duration_sec, 3),
                "steps_per_sec": round(steps_per_sec, 2),
                "timestamp": ep_end_time
            }
            
            # Append per-component reward breakdown fields if available.
            # Uses .get() throughout — safe if reward_breakdown is absent (backward compat).
            breakdown = info.get("reward_breakdown", {})
            if breakdown:
                # Log each component to TensorBoard
                for comp_key, comp_val in breakdown.items():
                    self.logger.record(f"reward/{comp_key}", float(comp_val))
                # Append to JSON record (prefixed with reward_ to avoid collisions)
                for comp_key, comp_val in breakdown.items():
                    record[f"reward_{comp_key}"] = round(float(comp_val), 5)
            self.metrics_history.append(record)
            metrics_path = os.path.join(self.metrics_dir, "metrics.json")
            try:
                with open(metrics_path, "w") as f:
                    json.dump(self.metrics_history, f, indent=4)
                print(f"[Benchmark] Episode {self.episode_count} saved — "
                      f"reward={record['reward']:.1f}, "
                      f"coverage={record['coverage']:.2f}%, "
                      f"duration={record['episode_duration_sec']:.1f}s")
            except Exception as e:
                print(f"[WARN] Failed to write metrics JSON: {e}")
                
            # Reset episode reward accumulator and episode timer
            self.current_ep_reward = 0.0
            self._ep_start_time = time.time()

        # 3. Save periodic model weights
        if self.n_calls % self.check_freq == 0:
            path = os.path.join(self.save_path, f"model_step_{self.n_calls}")
            try:
                self.model.save(path)
                print(f"[Trainer] Saved model checkpoint to: {path}.zip")
            except Exception as e:
                print(f"[ERROR] Failed to save checkpoint: {e}")

        return True


class Trainer:
    """
    Manages the SB3 PPO training loop for the Multi-UAV Swarm simulation.
    """
    SUPPORTED_SCENARIOS = ["downtown", "event", "residential", "mixed", "industrial"]

    def __init__(
        self,
        scenario: str = "downtown",
        episodes: int = 500,
        max_steps: int = 5000,       # was 1000 → increased for meaningful exploration
        headless: bool = True,
        log_interval: int = 10,
        save_interval: int = 50000,  # step-level saving frequency
        run_id: str = None,
        env = None,                  # passed Gym environment instance
    ):
        if scenario not in self.SUPPORTED_SCENARIOS:
            raise ValueError(f"Unknown scenario: {scenario!r}")

        self.scenario = scenario
        self.episodes = episodes
        self.max_steps = max_steps
        self.headless = headless
        self.log_interval = log_interval
        self.save_interval = save_interval
        self.run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")

        # ── Output paths ──────────────────────────────────────────────────
        root = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(root, "..", ".."))
        self.metrics_dir = os.path.join(project_root, "experiments", "metrics", self.run_id)
        self.logs_dir = os.path.join(project_root, "experiments", "logs", self.run_id)
        self.models_dir = os.path.join(project_root, "models", self.run_id)

        # ── State ─────────────────────────────────────────────────────────
        self.env = env
        self.model = None

    def setup(self):
        """Initialize environment, directory structures, and PPO agent."""
        os.makedirs(self.metrics_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        os.makedirs(self.models_dir, exist_ok=True)

        if not SB3_AVAILABLE:
            raise ImportError("Stable-Baselines3 is not installed in the current Python environment.")

        # Wrap self.env in a standard Monitor to ensure reward tracking
        if self.env is not None:
            self.env = Monitor(self.env)
        else:
            raise ValueError("[Trainer] Gym environment instance was not provided.")

        print(f"[Trainer] Run ID: {self.run_id}")
        print(f"[Trainer] Scenario: {self.scenario}")
        print(f"[Trainer] Headless: {self.headless}")
        print(f"[Trainer] Models  -> {self.models_dir}")
        print(f"[Trainer] Metrics -> {self.metrics_dir}")

        # Instantiate PPO model with standard hyper-parameters
        self.model = PPO(
            "MlpPolicy",
            self.env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,           # Regulate exploration
            tensorboard_log=self.logs_dir,
            verbose=1,
        )
        print("[Trainer] Stable-Baselines3 PPO policy successfully compiled.")

    def run(self):
        """Execute the 500,000-step training pipeline."""
        if self.model is None:
            raise ValueError("[Trainer] Trainer not set up. Call setup() first.")

        print("\n" + "=" * 58)
        print("  LAUNCHING PRODUCTION DRL PPO TRAINING RUN")
        print(f"  Step Horizon Limit: 500,000 steps")
        print("=" * 58 + "\n")

        # Define custom callback
        callback = SwarmTrainingCallback(
            check_freq=self.save_interval,
            save_path=self.models_dir,
            metrics_dir=self.metrics_dir,
            log_interval=self.log_interval,
            verbose=1,
            headless=self.headless
        )

        try:
            # Train the PPO agent
            self.model.learn(
                total_timesteps=500000,
                callback=callback,
                tb_log_name="PPO_swarm",
                reset_num_timesteps=True
            )
            
            # Save final trained model weights in timestamped folder
            final_path = os.path.join(self.models_dir, "ppo_swarm_final")
            self.model.save(final_path)
            
            # ALSO save to canonical models/ppo_swarm_final path for instant GUI load!
            parent_models_dir = os.path.dirname(self.models_dir)
            canonical_path = os.path.join(parent_models_dir, "ppo_swarm_final")
            self.model.save(canonical_path)
            
            print(f"\n[Trainer] Training complete! Final model saved to: {final_path}.zip and {canonical_path}.zip")
            
        except KeyboardInterrupt:
            print("\n[Trainer] KeyboardInterrupt received. Saving current model weights before exiting...")
            interrupt_path = os.path.join(self.models_dir, "ppo_swarm_interrupt")
            self.model.save(interrupt_path)
            
            # ALSO save to canonical models/ppo_swarm_final path for instant GUI load!
            parent_models_dir = os.path.dirname(self.models_dir)
            canonical_path = os.path.join(parent_models_dir, "ppo_swarm_final")
            self.model.save(canonical_path)
            
            print(f"[Trainer] Interrupted model weights successfully saved to: {interrupt_path}.zip and {canonical_path}.zip")
            
        except Exception as e:
            print(f"\n[Trainer] Critical error during training run: {e}")
            raise e

    def evaluate(self, model_path: str):
        """
        Run the pre-trained PPO model in visual evaluation (demo) mode.
        Drones will navigate autonomously using the learned policy without training.
        """
        print("\n" + "=" * 65)
        print("  LAUNCHING HIGH-PERFORMANCE AUTONOMOUS SWARM DEMO")
        print(f"  Loading Pre-Trained Model: {model_path}")
        print("=" * 65 + "\n")
        
        # Load the saved model weights
        self.model = PPO.load(model_path, env=self.env)
        print("[Trainer] Pre-trained PPO policy successfully loaded!")
        
        obs = self.env.reset()
        if isinstance(obs, tuple):
            obs = obs[0]
            
        step_count = 0
        while True:
            # Predict actions deterministically using the trained policy network
            action, _ = self.model.predict(obs, deterministic=True)
            
            # Step the Gymnasium environment
            step_result = self.env.step(action)
            
            # Unpack Gymnasium's 5-element return
            if len(step_result) == 5:
                obs, reward, terminated, truncated, info = step_result
                done = terminated or truncated
            else:
                obs, reward, done, info = step_result
                
            if isinstance(obs, tuple):
                obs = obs[0]
                
            step_count += 1
            if step_count % 100 == 0:
                print(f" -> Demo Step {step_count:>4} | Coverage: {info.get('coverage', 0.0):5.2f}%")
                
            if done:
                print("\n🔄 [Demo Episode Completed] resetting environment positions...\n")
                obs = self.env.reset()
                if isinstance(obs, tuple):
                    obs = obs[0]
                step_count = 0

    def close(self):
        """Clean up environment resources."""
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass


def _parse_args():
    p = argparse.ArgumentParser(
        description="Train RL agent on Multi-UAV Swarm simulation"
    )
    p.add_argument("--scenario", default="downtown",
                   choices=Trainer.SUPPORTED_SCENARIOS)
    p.add_argument("--episodes", type=int, default=500)
    p.add_argument("--max-steps", type=int, default=5000,
                   help="Max steps per episode (default: 5000)")
    p.add_argument("--headless", action="store_true", default=True)
    p.add_argument("--log-interval", type=int, default=10)
    p.add_argument("--save-interval", type=int, default=50000)
    p.add_argument("--run-id", default=None)
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    trainer = Trainer(
        scenario=args.scenario,
        episodes=args.episodes,
        max_steps=args.max_steps,
        headless=args.headless,
        log_interval=args.log_interval,
        save_interval=args.save_interval,
        run_id=args.run_id,
    )
    try:
        trainer.setup()
        trainer.run()
    except Exception as e:
        print(f"\n[Trainer] Trainer initialization failed: {e}")
    finally:
        trainer.close()
