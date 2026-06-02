"""
benchmark_logger.py — Lightweight RL benchmark metrics logger.

Records per-episode training statistics to a JSON file in experiments/metrics/.
Designed to be used alongside the existing SwarmTrainingCallback in trainer.py.

Logged fields (per episode):
    episode             — episode number (1-indexed)
    episode_duration_sec — wall-clock time for this episode (seconds)
    avg_reward          — mean reward over this episode
    avg_coverage        — mean area coverage % over this episode
    collision_count     — number of collision events in this episode
    steps_per_sec       — training throughput (simulation steps / wall-clock second)
    total_steps         — cumulative training steps at end of this episode
    timestamp           — Unix timestamp at episode end

Usage:
    from experiments.metrics.benchmark_logger import BenchmarkLogger

    logger = BenchmarkLogger(run_id="my_run")
    logger.log_episode(
        episode=1,
        episode_duration_sec=42.3,
        avg_reward=-12.5,
        avg_coverage=6.1,
        collision_count=0,
        steps_per_sec=8.4,
        total_steps=5000,
    )
    logger.save()   # also called automatically on each log_episode

Output file:
    experiments/metrics/rl_benchmark_<run_id>.json
"""

import os
import json
import time
from datetime import datetime
from typing import Optional


class BenchmarkLogger:
    """
    Accumulates and persists RL episode-level benchmark metrics.

    Thread-safety: NOT thread-safe (single-process RL training only).
    """

    def __init__(self, run_id: Optional[str] = None, output_dir: Optional[str] = None):
        """
        Args:
            run_id:     Unique identifier for this training run.
                        Defaults to a timestamp string.
            output_dir: Directory to write the JSON file.
                        Defaults to experiments/metrics/ relative to project root.
        """
        self.run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")

        if output_dir is None:
            # Resolve project root: this file lives at experiments/metrics/benchmark_logger.py
            _here = os.path.dirname(os.path.abspath(__file__))
            output_dir = _here  # already inside experiments/metrics/

        os.makedirs(output_dir, exist_ok=True)
        self.output_path = os.path.join(output_dir, f"rl_benchmark_{self.run_id}.json")

        self._records: list = []
        self._run_start_time: float = time.time()

        # Write a header record so the file exists even before the first episode
        self._meta = {
            "run_id": self.run_id,
            "started_at": datetime.now().isoformat(),
            "description": "RL training benchmark — episode-level metrics",
            "fields": [
                "episode",
                "episode_duration_sec",
                "avg_reward",
                "avg_coverage",
                "collision_count",
                "steps_per_sec",
                "total_steps",
                "timestamp",
            ],
        }
        self._flush()
        print(f"[BenchmarkLogger] Logging to: {self.output_path}")

    # ── Public API ─────────────────────────────────────────────────────────────

    def log_episode(
        self,
        episode: int,
        episode_duration_sec: float,
        avg_reward: float,
        avg_coverage: float,
        collision_count: int,
        steps_per_sec: float,
        total_steps: int,
        **extra_fields,
    ) -> None:
        """
        Record one episode's benchmark metrics and immediately persist to disk.

        Args:
            episode:              Episode number (1-indexed).
            episode_duration_sec: Wall-clock time for this episode in seconds.
            avg_reward:           Mean reward accumulated during this episode.
            avg_coverage:         Mean area coverage % during this episode.
            collision_count:      Number of collision/proximity events in this episode.
            steps_per_sec:        Training throughput in simulation steps per second.
            total_steps:          Cumulative simulation steps at the end of this episode.
            **extra_fields:       Any additional key-value pairs to include in the record.
        """
        record = {
            "episode": episode,
            "episode_duration_sec": round(episode_duration_sec, 3),
            "avg_reward": round(float(avg_reward), 4),
            "avg_coverage": round(float(avg_coverage), 4),
            "collision_count": int(collision_count),
            "steps_per_sec": round(float(steps_per_sec), 2),
            "total_steps": int(total_steps),
            "timestamp": time.time(),
        }
        record.update(extra_fields)
        self._records.append(record)
        self._flush()

    def summary(self) -> dict:
        """
        Return a dict of aggregate statistics over all logged episodes.
        Safe to call at any time (returns empty dict if no episodes logged yet).
        """
        if not self._records:
            return {}

        rewards = [r["avg_reward"] for r in self._records]
        coverages = [r["avg_coverage"] for r in self._records]
        collisions = [r["collision_count"] for r in self._records]
        durations = [r["episode_duration_sec"] for r in self._records]
        sps = [r["steps_per_sec"] for r in self._records]

        def _mean(lst):
            return sum(lst) / len(lst) if lst else 0.0

        def _std(lst):
            m = _mean(lst)
            return (sum((x - m) ** 2 for x in lst) / len(lst)) ** 0.5 if len(lst) > 1 else 0.0

        return {
            "run_id": self.run_id,
            "total_episodes": len(self._records),
            "total_steps": self._records[-1]["total_steps"] if self._records else 0,
            "mean_reward": round(_mean(rewards), 4),
            "std_reward": round(_std(rewards), 4),
            "mean_coverage": round(_mean(coverages), 4),
            "std_coverage": round(_std(coverages), 4),
            "total_collisions": int(sum(collisions)),
            "mean_episode_duration_sec": round(_mean(durations), 3),
            "mean_steps_per_sec": round(_mean(sps), 2),
        }

    def save(self) -> str:
        """Force-flush the current records to disk. Returns the output path."""
        self._flush()
        return self.output_path

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _flush(self) -> None:
        """Serialize the full log (meta + records + summary) to JSON."""
        payload = {
            "meta": self._meta,
            "episodes": self._records,
            "summary": self.summary(),
        }
        try:
            with open(self.output_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=4)
        except Exception as e:
            print(f"[BenchmarkLogger][WARN] Failed to write benchmark file: {e}")

    def __repr__(self) -> str:
        return (
            f"BenchmarkLogger(run_id={self.run_id!r}, "
            f"episodes={len(self._records)}, "
            f"output={self.output_path!r})"
        )
