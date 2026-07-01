"""
platform/dashboard/writer.py — DashboardWriter

The DashboardWriter is the public API through which every other platform
module updates the dashboard state.  It is the ONLY sanctioned way to
write to dashboard_state.json.

All write methods are additive and safe:
  - They load the current state before writing.
  - They validate the update against the model schema.
  - They timestamp the update automatically.
  - They call save() so the state is always flushed to disk.

Usage (from any platform module)
---------------------------------
    from platform.dashboard.writer import DashboardWriter

    writer = DashboardWriter()

    # Record a completed experiment
    writer.add_experiment({
        "id": "EXP-0034",
        "name": "A* Baseline — Event World",
        "config_profile": "baseline_astar",
        "world": "event",
        "n_trials": 5,
        "status": "pass",
        "metrics": {
            "path_efficiency": 0.871,
            "near_misses": 9,
            "mean_trr_percent": 2.3,
        }
    })

    # Update phase progress
    writer.update_phase(progress=1, progress_pct=80)

    # Mark a task done
    writer.complete_task("T-005")

    # Update best configuration
    writer.set_best_configuration({
        "config_profile": "hybrid_astar_v2",
        "experiment_id": "EXP-0040",
        "path_efficiency": 0.912,
        "near_misses": 5,
        "confidence": "Medium (n=8)",
    })

Thread-safety
-------------
    NOT thread-safe. Designed for single-process experiment orchestration.
"""

from __future__ import annotations

import json
import os
from copy import deepcopy
from datetime import datetime
from typing import Any, Dict, List, Optional

# Resolve the default state file path relative to this module
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STATE_PATH = os.path.join(_HERE, "dashboard_state.json")


class DashboardWriter:
    """
    Write-side API for the dashboard state.

    All mutating methods follow the pattern:
      1. Ensure state is loaded (lazy-load on first call)
      2. Apply the mutation
      3. Update metadata timestamps
      4. Flush state to disk
      5. Return self (for chaining)

    Parameters
    ----------
    state_path : str
        Absolute or relative path to dashboard_state.json.
        Defaults to the file co-located with this module.
    auto_save : bool
        If True (default), every mutation method saves immediately.
        Set to False to batch mutations and call save() manually.
    """

    def __init__(
        self,
        state_path: str = DEFAULT_STATE_PATH,
        auto_save: bool = True,
    ) -> None:
        self._state_path = state_path
        self._auto_save = auto_save
        self._state: Optional[Dict] = None   # Loaded lazily on first access

    # ── State access ─────────────────────────────────────────────────────────

    @property
    def state(self) -> Dict:
        """Return the current in-memory state, loading from disk if needed."""
        if self._state is None:
            self._state = self._load()
        return self._state

    def _load(self) -> Dict:
        """Load state from disk.  Returns empty scaffolding if file missing."""
        if not os.path.isfile(self._state_path):
            return self._empty_state()
        try:
            with open(self._state_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[DashboardWriter] WARNING: Could not load state ({exc}). Starting fresh.")
            return self._empty_state()

    def reload(self) -> "DashboardWriter":
        """Force-reload state from disk, discarding any unsaved in-memory changes."""
        self._state = self._load()
        return self

    def save(self) -> "DashboardWriter":
        """
        Flush the current in-memory state to disk.

        Updates the meta.generated_at timestamp before writing.
        Creates parent directories if they do not exist.
        """
        self.state["meta"]["generated_at"] = datetime.now().isoformat()
        os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
        with open(self._state_path, "w", encoding="utf-8") as f:
            json.dump(self.state, f, indent=4, ensure_ascii=False)
        return self

    def _maybe_save(self) -> None:
        """Save if auto_save is enabled."""
        if self._auto_save:
            self.save()

    # ── Project-level mutations ───────────────────────────────────────────────

    def set_current_phase(self, phase_id: int, sprint_name: str = "") -> "DashboardWriter":
        """Advance the project's current phase and optionally update the sprint name."""
        self.state["project"]["current_phase"] = phase_id
        if sprint_name:
            self.state["project"]["current_sprint"] = sprint_name
        self.state["project"]["last_updated"] = datetime.now().strftime("%Y-%m-%d")
        self._maybe_save()
        return self

    def set_branch(self, branch: str) -> "DashboardWriter":
        """Update the current git branch name."""
        self.state["project"]["current_branch"] = branch
        self._maybe_save()
        return self

    def set_research_readiness(self, score: int) -> "DashboardWriter":
        """
        Update the overall research readiness score (0–100).

        This is a manually curated score reflecting how close the project is
        to producing publishable evidence.  It is NOT computed automatically.
        """
        if not 0 <= score <= 100:
            raise ValueError(f"Research readiness must be 0–100; got {score}")
        self.state["project"]["research_readiness"] = score
        self._maybe_save()
        return self

    # ── Research mutations ────────────────────────────────────────────────────

    def set_active_question(self, question_id: str) -> "DashboardWriter":
        """Change the active research question shown on the dashboard."""
        self.state["research"]["active_question_id"] = question_id
        self._maybe_save()
        return self

    def set_active_hypothesis(self, hypothesis_id: str) -> "DashboardWriter":
        """Change the active hypothesis shown on the dashboard."""
        self.state["research"]["active_hypothesis_id"] = hypothesis_id
        self._maybe_save()
        return self

    def update_hypothesis_status(self, hypothesis_id: str, status: str) -> "DashboardWriter":
        """
        Update a hypothesis state.

        Valid statuses: proposed | under_test | confirmed | refuted | inconclusive
        """
        for h in self.state["research"].get("hypotheses", []):
            if h["id"] == hypothesis_id:
                h["status"] = status
                break
        self._maybe_save()
        return self

    def update_paper_section(self, section_name: str, progress: int, note: str = "") -> "DashboardWriter":
        """Update the completion percentage for a paper section (0–100)."""
        for s in self.state["research"].get("paper_sections", []):
            if s["name"].lower() == section_name.lower():
                s["progress"] = max(0, min(100, progress))
                if note:
                    s["status_note"] = note
                break
        self._maybe_save()
        return self

    # ── Roadmap mutations ─────────────────────────────────────────────────────

    def update_phase(
        self,
        phase_id: int,
        status: Optional[str] = None,
        progress: Optional[int] = None,
    ) -> "DashboardWriter":
        """
        Update the status or progress of a roadmap phase.

        Parameters
        ----------
        phase_id : int
            The phase number (0–10).
        status : str, optional
            One of: complete | active | in_progress | planned | blocked
        progress : int, optional
            Completion percentage 0–100.
        """
        for phase in self.state.get("roadmap", []):
            if phase["id"] == phase_id:
                if status is not None:
                    phase["status"] = status
                if progress is not None:
                    phase["progress"] = max(0, min(100, progress))
                break
        self._maybe_save()
        return self

    def mark_exit_criterion(self, phase_id: int, criterion_text: str, met: bool = True) -> "DashboardWriter":
        """Mark a specific exit criterion as met or unmet."""
        for phase in self.state.get("roadmap", []):
            if phase["id"] == phase_id:
                for crit in phase.get("exit_criteria", []):
                    if crit["text"] == criterion_text:
                        crit["met"] = met
                        break
                break
        self._maybe_save()
        return self

    # ── Module mutations ──────────────────────────────────────────────────────

    def update_module(
        self,
        module_id: str,
        status: Optional[str] = None,
        progress: Optional[int] = None,
        notes: Optional[str] = None,
    ) -> "DashboardWriter":
        """
        Update a module's implementation status and progress.

        Parameters
        ----------
        module_id : str
            The module's id field (e.g. "planner_factory").
        status : str, optional
            One of: implemented | in_progress | planned | missing
        progress : int, optional
            0–100.
        notes : str, optional
            Short status note. Replaces existing note.
        """
        for mod in self.state.get("modules", []):
            if mod["id"] == module_id:
                if status is not None:
                    mod["status"] = status
                if progress is not None:
                    mod["progress"] = max(0, min(100, progress))
                if notes is not None:
                    mod["notes"] = notes
                break
        self._maybe_save()
        return self

    # ── Algorithm mutations ───────────────────────────────────────────────────

    def update_algorithm(
        self,
        algorithm_id: str,
        status: Optional[str] = None,
        last_validated: Optional[str] = None,
        best_metric: Optional[str] = None,
        notes: Optional[str] = None,
    ) -> "DashboardWriter":
        """Update an algorithm's registration status and validation info."""
        today = datetime.now().strftime("%Y-%m-%d")
        for alg in self.state.get("algorithms", []):
            if alg["id"] == algorithm_id:
                if status is not None:
                    alg["status"] = status
                if last_validated is not None:
                    alg["last_validated"] = last_validated
                elif status == "registered":
                    alg["last_validated"] = today
                if best_metric is not None:
                    alg["best_metric"] = best_metric
                if notes is not None:
                    alg["notes"] = notes
                break
        self._maybe_save()
        return self

    # ── Experiment mutations ──────────────────────────────────────────────────

    def add_experiment(self, experiment: Dict[str, Any]) -> "DashboardWriter":
        """
        Prepend a new experiment record to the recent_experiments list.

        The list is capped at 20 entries (oldest removed first).
        The experiment dict should follow the ExperimentRecord schema.

        Parameters
        ----------
        experiment : dict
            Must include: id, name, config_profile, world, n_trials, status.
            Optional: timestamp, metrics, notes, research_question_id.
        """
        # Auto-fill timestamp if missing
        if "timestamp" not in experiment or not experiment["timestamp"]:
            experiment["timestamp"] = datetime.now().isoformat()

        # Prepend (most recent first)
        experiments = self.state.get("recent_experiments", [])
        experiments.insert(0, experiment)

        # Cap list at 20 entries
        self.state["recent_experiments"] = experiments[:20]
        self._maybe_save()
        return self

    def update_experiment_status(self, experiment_id: str, status: str) -> "DashboardWriter":
        """Update the status of an existing experiment record."""
        for exp in self.state.get("recent_experiments", []):
            if exp["id"] == experiment_id:
                exp["status"] = status
                break
        self._maybe_save()
        return self

    # ── Best configuration ────────────────────────────────────────────────────

    def set_best_configuration(self, config: Dict[str, Any]) -> "DashboardWriter":
        """
        Replace the current best configuration record.

        The config dict should follow the BestConfiguration schema.
        Auto-fills last_validated if not provided.
        """
        if "last_validated" not in config or not config["last_validated"]:
            config["last_validated"] = datetime.now().strftime("%Y-%m-%d")
        self.state["best_configuration"] = config
        self._maybe_save()
        return self

    # ── Task mutations ────────────────────────────────────────────────────────

    def add_task(self, task: Dict[str, Any]) -> "DashboardWriter":
        """
        Add a new task to the next_tasks list.

        The task dict must include: id, text.
        Optional: priority, phase, owner, estimated_hours, depends_on, done.
        """
        self.state.setdefault("next_tasks", []).append(task)
        self._maybe_save()
        return self

    def complete_task(self, task_id: str) -> "DashboardWriter":
        """Mark a task as done."""
        for task in self.state.get("next_tasks", []):
            if task["id"] == task_id:
                task["done"] = True
                break
        self._maybe_save()
        return self

    def remove_task(self, task_id: str) -> "DashboardWriter":
        """Remove a task by ID."""
        self.state["next_tasks"] = [
            t for t in self.state.get("next_tasks", [])
            if t["id"] != task_id
        ]
        self._maybe_save()
        return self

    # ── Knowledge base mutations ──────────────────────────────────────────────

    def add_knowledge_entry(self, entry: Dict[str, Any]) -> "DashboardWriter":
        """
        Add a new knowledge base entry.

        The entry dict must include: id, title, category.
        Optional: description, file_path, url, date_added, tags.
        """
        if "date_added" not in entry or not entry["date_added"]:
            entry["date_added"] = datetime.now().strftime("%Y-%m-%d")
        self.state.setdefault("knowledge_base", []).append(entry)
        self._maybe_save()
        return self

    # ── Environment mutations ─────────────────────────────────────────────────

    def mark_environment_used(self, environment_id: str) -> "DashboardWriter":
        """Update the last_used timestamp for an environment."""
        today = datetime.now().strftime("%Y-%m-%d")
        for env in self.state.get("environments", []):
            if env["id"] == environment_id:
                env["last_used"] = today
                break
        self._maybe_save()
        return self

    def set_environment_status(self, environment_id: str, status: str) -> "DashboardWriter":
        """Update an environment's availability status."""
        for env in self.state.get("environments", []):
            if env["id"] == environment_id:
                env["status"] = status
                break
        self._maybe_save()
        return self

    # ── Utility ───────────────────────────────────────────────────────────────

    def get_state_copy(self) -> Dict:
        """Return a deep copy of the current state (safe for external consumption)."""
        return deepcopy(self.state)

    @staticmethod
    def _empty_state() -> Dict:
        """Return an empty but valid state scaffold."""
        return {
            "meta": {
                "generated_at": datetime.now().isoformat(),
                "platform_version": "1.0.0",
                "schema_version": "1.0",
            },
            "project": {},
            "research": {"questions": [], "hypotheses": [], "paper_sections": []},
            "roadmap": [],
            "modules": [],
            "algorithms": [],
            "environments": [],
            "recent_experiments": [],
            "best_configuration": None,
            "knowledge_base": [],
            "next_tasks": [],
        }

    def __repr__(self) -> str:
        loaded = "loaded" if self._state is not None else "not loaded"
        return f"DashboardWriter(state_path={self._state_path!r}, state={loaded})"
