"""
platform/dashboard/models.py — Dashboard Data Models

Defines all dataclasses that represent the dashboard state.  These models
are the canonical schema for dashboard_state.json.  Every field has a type
annotation and a default so that partial state files are valid.

Design principles
-----------------
  - All models use @dataclass for zero-boilerplate serialisation
  - Every model has a from_dict() classmethod for safe deserialisation
  - Every model has a to_dict() method for JSON serialisation
  - No model imports anything from Webots or the navigation stack

Schema version
--------------
    SCHEMA_VERSION = "1.0" — bump when any model field changes
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Any, Dict, List, Optional


# ── Schema version ─────────────────────────────────────────────────────────────

SCHEMA_VERSION = "1.0"
PLATFORM_VERSION = "1.0.0"


# ── Enumerations (as string literals for JSON compatibility) ────────────────────

class PhaseStatus:
    COMPLETE    = "complete"
    ACTIVE      = "active"
    IN_PROGRESS = "in_progress"
    PLANNED     = "planned"
    BLOCKED     = "blocked"

class ModuleStatus:
    IMPLEMENTED = "implemented"
    IN_PROGRESS = "in_progress"
    PLANNED     = "planned"
    MISSING     = "missing"

class AlgorithmStatus:
    REGISTERED  = "registered"
    IN_PROGRESS = "in_progress"
    PLANNED     = "planned"
    MISSING     = "missing"

class HypothesisStatus:
    PROPOSED    = "proposed"
    UNDER_TEST  = "under_test"
    CONFIRMED   = "confirmed"
    REFUTED     = "refuted"
    INCONCLUSIVE = "inconclusive"

class QuestionStatus:
    OPEN                = "open"
    UNDER_INVESTIGATION = "under_investigation"
    ANSWERED            = "answered"
    REJECTED            = "rejected"

class ExperimentResult:
    PASS    = "pass"
    FAIL    = "fail"
    RUNNING = "running"
    QUEUED  = "queued"

class EnvironmentStatus:
    ACTIVE   = "active"
    AVAILABLE = "available"
    PLANNED  = "planned"


# ── Core models ─────────────────────────────────────────────────────────────────

@dataclass
class MetaInfo:
    """Dashboard metadata — updated on every write."""
    generated_at: str     = field(default_factory=lambda: datetime.now().isoformat())
    platform_version: str = PLATFORM_VERSION
    schema_version: str   = SCHEMA_VERSION

    @classmethod
    def from_dict(cls, d: Dict) -> MetaInfo:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class ProjectInfo:
    """Top-level project identity and sprint information."""
    name: str           = "Hybrid Multi-UAV Navigation Research Platform"
    current_phase: int  = 1
    current_branch: str = "single-drone-navigation"
    last_updated: str   = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d"))
    current_sprint: str = "Phase 1 — Platform Architecture"
    repository: str     = "The-FOOL-00/hybrid-multi-uav-swarm"
    institution: str    = "IFSP"
    research_readiness: int = 34   # 0–100 percent
    total_phases: int   = 11

    @classmethod
    def from_dict(cls, d: Dict) -> ProjectInfo:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class PaperSection:
    """Completion percentage for a single paper section."""
    name: str
    progress: int       # 0–100
    status_note: str = ""

    @classmethod
    def from_dict(cls, d: Dict) -> PaperSection:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class ResearchQuestion:
    """A single research question and its investigation state."""
    id: str
    text: str
    status: str            = QuestionStatus.OPEN
    since: str             = ""
    supporting_experiments: List[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: Dict) -> ResearchQuestion:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Hypothesis:
    """A falsifiable hypothesis linked to a research question."""
    id: str
    question_id: str
    text: str
    status: str       = HypothesisStatus.PROPOSED
    prediction: str   = ""
    since: str        = ""
    evidence_ids: List[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: Dict) -> Hypothesis:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class ResearchState:
    """All active research tracking — questions, hypotheses, paper progress."""
    active_question_id: str        = "Q-001"
    active_hypothesis_id: str      = "H-001"
    questions: List[ResearchQuestion] = field(default_factory=list)
    hypotheses: List[Hypothesis]   = field(default_factory=list)
    paper_sections: List[PaperSection] = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: Dict) -> ResearchState:
        obj = cls()
        obj.active_question_id  = d.get("active_question_id", obj.active_question_id)
        obj.active_hypothesis_id = d.get("active_hypothesis_id", obj.active_hypothesis_id)
        obj.questions   = [ResearchQuestion.from_dict(q) for q in d.get("questions", [])]
        obj.hypotheses  = [Hypothesis.from_dict(h) for h in d.get("hypotheses", [])]
        obj.paper_sections = [PaperSection.from_dict(s) for s in d.get("paper_sections", [])]
        return obj

    def to_dict(self) -> Dict:
        return {
            "active_question_id":  self.active_question_id,
            "active_hypothesis_id": self.active_hypothesis_id,
            "questions":    [q.to_dict() for q in self.questions],
            "hypotheses":   [h.to_dict() for h in self.hypotheses],
            "paper_sections": [s.to_dict() for s in self.paper_sections],
        }

    def get_active_question(self) -> Optional[ResearchQuestion]:
        for q in self.questions:
            if q.id == self.active_question_id:
                return q
        return None

    def get_active_hypothesis(self) -> Optional[Hypothesis]:
        for h in self.hypotheses:
            if h.id == self.active_hypothesis_id:
                return h
        return None


@dataclass
class ExitCriterion:
    """A single measurable exit criterion for a roadmap phase."""
    text: str
    met: bool = False

    @classmethod
    def from_dict(cls, d: Dict) -> ExitCriterion:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class RoadmapPhase:
    """A single phase in the development roadmap."""
    id: int
    name: str
    short_name: str
    description: str
    status: str                   = PhaseStatus.PLANNED
    progress: int                 = 0         # 0–100
    paper_section: str            = ""
    research_value: str           = ""        # High / Very High / Critical
    exit_criteria: List[ExitCriterion] = field(default_factory=list)
    deliverables: List[str]       = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: Dict) -> RoadmapPhase:
        obj = cls(
            id          = d["id"],
            name        = d["name"],
            short_name  = d.get("short_name", d["name"]),
            description = d.get("description", ""),
            status      = d.get("status", PhaseStatus.PLANNED),
            progress    = d.get("progress", 0),
            paper_section = d.get("paper_section", ""),
            research_value = d.get("research_value", ""),
        )
        obj.exit_criteria = [ExitCriterion.from_dict(e) for e in d.get("exit_criteria", [])]
        obj.deliverables  = d.get("deliverables", [])
        return obj

    def to_dict(self) -> Dict:
        return {
            "id":            self.id,
            "name":          self.name,
            "short_name":    self.short_name,
            "description":   self.description,
            "status":        self.status,
            "progress":      self.progress,
            "paper_section": self.paper_section,
            "research_value": self.research_value,
            "exit_criteria": [e.to_dict() for e in self.exit_criteria],
            "deliverables":  self.deliverables,
        }


@dataclass
class Module:
    """A platform or navigation module and its implementation status."""
    id: str
    name: str
    description: str
    status: str        = ModuleStatus.PLANNED
    progress: int      = 0
    owner: str         = ""
    file_path: str     = ""
    notes: str         = ""
    phase_introduced: int = 0

    @classmethod
    def from_dict(cls, d: Dict) -> Module:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Algorithm:
    """A registered algorithm in the planner or avoidance factory."""
    id: str
    name: str
    category: str      # "planner" | "avoidance" | "hybrid" | "coordination"
    status: str        = AlgorithmStatus.PLANNED
    file_path: str     = ""
    last_validated: str = ""
    best_metric: str   = ""
    notes: str         = ""

    @classmethod
    def from_dict(cls, d: Dict) -> Algorithm:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Environment:
    """A simulation world / scenario."""
    id: str
    name: str
    world_file: str
    status: str           = EnvironmentStatus.AVAILABLE
    crowd_count: int      = 0
    uav_altitude_m: float = 15.0
    area_m2: int          = 0
    description: str      = ""
    last_used: str        = ""

    @classmethod
    def from_dict(cls, d: Dict) -> Environment:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class ExperimentRecord:
    """Summary record for a completed or queued experiment."""
    id: str                          # EXP-XXXX
    name: str
    config_profile: str
    world: str
    n_trials: int
    status: str                      # ExperimentResult.*
    timestamp: str                   = ""
    metrics: Dict[str, Any]          = field(default_factory=dict)
    notes: str                       = ""
    research_question_id: str        = ""

    @classmethod
    def from_dict(cls, d: Dict) -> ExperimentRecord:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class BestConfiguration:
    """The current validated best algorithm configuration."""
    config_profile: str       = ""
    experiment_id: str        = ""
    path_efficiency: float    = 0.0
    near_misses: int          = 0
    mean_trr_percent: float   = 0.0
    mean_coverage_percent: float = 0.0
    confidence: str           = ""    # "Low (n<5)" / "Medium" / "High"
    last_validated: str       = ""
    notes: str                = ""

    @classmethod
    def from_dict(cls, d: Dict) -> BestConfiguration:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class KnowledgeEntry:
    """A link or reference in the project knowledge base."""
    id: str
    title: str
    category: str      # "research_question" | "architecture" | "audit" | "paper" | "decision"
    description: str   = ""
    file_path: str     = ""
    url: str           = ""
    date_added: str    = ""
    tags: List[str]    = field(default_factory=list)

    @classmethod
    def from_dict(cls, d: Dict) -> KnowledgeEntry:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class NextTask:
    """A prioritised next action item."""
    id: str
    text: str
    priority: str      = "medium"   # "critical" | "high" | "medium" | "low"
    phase: int         = 0
    owner: str         = ""
    estimated_hours: float = 0.0
    depends_on: List[str] = field(default_factory=list)
    done: bool         = False

    @classmethod
    def from_dict(cls, d: Dict) -> NextTask:
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

    def to_dict(self) -> Dict:
        return asdict(self)


# ── Root dashboard state ────────────────────────────────────────────────────────

@dataclass
class DashboardState:
    """
    Root model for the complete dashboard state.

    This is the canonical in-memory representation of dashboard_state.json.
    All platform modules interact with the state through this model.
    """
    meta: MetaInfo                        = field(default_factory=MetaInfo)
    project: ProjectInfo                  = field(default_factory=ProjectInfo)
    research: ResearchState               = field(default_factory=ResearchState)
    roadmap: List[RoadmapPhase]           = field(default_factory=list)
    modules: List[Module]                 = field(default_factory=list)
    algorithms: List[Algorithm]           = field(default_factory=list)
    environments: List[Environment]       = field(default_factory=list)
    recent_experiments: List[ExperimentRecord] = field(default_factory=list)
    best_configuration: Optional[BestConfiguration] = None
    knowledge_base: List[KnowledgeEntry]  = field(default_factory=list)
    next_tasks: List[NextTask]            = field(default_factory=list)

    # ── Serialisation ────────────────────────────────────────────────────────

    def to_dict(self) -> Dict:
        return {
            "meta":                self.meta.to_dict(),
            "project":             self.project.to_dict(),
            "research":            self.research.to_dict(),
            "roadmap":             [p.to_dict() for p in self.roadmap],
            "modules":             [m.to_dict() for m in self.modules],
            "algorithms":          [a.to_dict() for a in self.algorithms],
            "environments":        [e.to_dict() for e in self.environments],
            "recent_experiments":  [e.to_dict() for e in self.recent_experiments],
            "best_configuration":  self.best_configuration.to_dict() if self.best_configuration else None,
            "knowledge_base":      [k.to_dict() for k in self.knowledge_base],
            "next_tasks":          [t.to_dict() for t in self.next_tasks],
        }

    def to_json(self, indent: int = 4) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    @classmethod
    def from_dict(cls, d: Dict) -> DashboardState:
        obj = cls()
        obj.meta    = MetaInfo.from_dict(d.get("meta", {}))
        obj.project = ProjectInfo.from_dict(d.get("project", {}))
        obj.research = ResearchState.from_dict(d.get("research", {}))
        obj.roadmap  = [RoadmapPhase.from_dict(p) for p in d.get("roadmap", [])]
        obj.modules  = [Module.from_dict(m) for m in d.get("modules", [])]
        obj.algorithms = [Algorithm.from_dict(a) for a in d.get("algorithms", [])]
        obj.environments = [Environment.from_dict(e) for e in d.get("environments", [])]
        obj.recent_experiments = [ExperimentRecord.from_dict(e) for e in d.get("recent_experiments", [])]
        bc = d.get("best_configuration")
        obj.best_configuration = BestConfiguration.from_dict(bc) if bc else None
        obj.knowledge_base = [KnowledgeEntry.from_dict(k) for k in d.get("knowledge_base", [])]
        obj.next_tasks = [NextTask.from_dict(t) for t in d.get("next_tasks", [])]
        return obj

    @classmethod
    def from_json(cls, json_str: str) -> DashboardState:
        return cls.from_dict(json.loads(json_str))

    @classmethod
    def from_file(cls, path: str) -> DashboardState:
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    # ── Convenience accessors ─────────────────────────────────────────────────

    def get_active_question(self) -> Optional[ResearchQuestion]:
        return self.research.get_active_question()

    def get_active_hypothesis(self) -> Optional[Hypothesis]:
        return self.research.get_active_hypothesis()

    def get_current_phase(self) -> Optional[RoadmapPhase]:
        for p in self.roadmap:
            if p.id == self.project.current_phase:
                return p
        return None

    def get_algorithms_by_category(self, category: str) -> List[Algorithm]:
        return [a for a in self.algorithms if a.category == category]

    def get_pending_tasks(self) -> List[NextTask]:
        return [t for t in self.next_tasks if not t.done]

    def overall_paper_progress(self) -> int:
        if not self.research.paper_sections:
            return 0
        return int(sum(s.progress for s in self.research.paper_sections) / len(self.research.paper_sections))
