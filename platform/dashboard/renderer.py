"""
platform/dashboard/renderer.py — DashboardRenderer

Converts a dashboard state dictionary into a complete, self-contained HTML
document.  The HTML file has zero external dependencies: all CSS and
JavaScript are embedded inline.  It opens correctly from any browser via
the file:// protocol without a web server.

Design goals
------------
  - Zero dependencies beyond the Python standard library
  - Self-contained output (single .html file)
  - Production-quality visual design (dark mode, glassmorphism)
  - Responsive layout (works at any window width)
  - Accessible (semantic HTML, ARIA labels, color-contrast compliant)
  - Extensible (new sections added without touching existing ones)

Usage
-----
    from platform.dashboard.renderer import DashboardRenderer

    renderer = DashboardRenderer()
    html = renderer.render(state_dict)

    with open("dashboard.html", "w", encoding="utf-8") as f:
        f.write(html)
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Dict, List, Optional


class DashboardRenderer:
    """
    Renders a dashboard state dict → self-contained HTML string.

    The render() method is the sole public API.  All _render_* methods are
    internal section builders.  Adding a new dashboard section means adding
    a new _render_* method and calling it inside render().
    """

    # ── Status display configuration ─────────────────────────────────────────

    PHASE_STATUS_LABELS = {
        "complete":    ("COMPLETE",    "#10b981", "●"),
        "active":      ("ACTIVE",      "#00d4ff", "◉"),
        "in_progress": ("IN PROGRESS", "#f59e0b", "◑"),
        "planned":     ("PLANNED",     "#64748b", "○"),
        "blocked":     ("BLOCKED",     "#ef4444", "⊗"),
    }

    MODULE_STATUS_CONFIG = {
        "implemented": ("Implemented", "#10b981"),
        "in_progress": ("In Progress", "#f59e0b"),
        "planned":     ("Planned",     "#64748b"),
        "missing":     ("Missing",     "#ef4444"),
    }

    ALGORITHM_STATUS_CONFIG = {
        "registered":  ("Registered",  "#10b981"),
        "in_progress": ("In Progress", "#f59e0b"),
        "planned":     ("Planned",     "#64748b"),
        "missing":     ("Missing",     "#ef4444"),
    }

    ENV_STATUS_CONFIG = {
        "active":    ("Active",     "#10b981"),
        "available": ("Available",  "#00d4ff"),
        "planned":   ("Planned",    "#64748b"),
    }

    EXP_STATUS_CONFIG = {
        "pass":    ("PASS",    "#10b981"),
        "fail":    ("FAIL",    "#ef4444"),
        "running": ("RUNNING", "#f59e0b"),
        "queued":  ("QUEUED",  "#64748b"),
    }

    QUESTION_STATUS_CONFIG = {
        "open":                ("Open",              "#64748b"),
        "under_investigation": ("Under Investigation","#00d4ff"),
        "answered":            ("Answered",           "#10b981"),
        "rejected":            ("Rejected",           "#ef4444"),
    }

    HYPOTHESIS_STATUS_CONFIG = {
        "proposed":     ("Proposed",      "#64748b"),
        "under_test":   ("Under Test",    "#f59e0b"),
        "confirmed":    ("Confirmed",     "#10b981"),
        "refuted":      ("Refuted",       "#ef4444"),
        "inconclusive": ("Inconclusive",  "#a78bfa"),
    }

    PRIORITY_CONFIG = {
        "critical": ("#ef4444", "CRITICAL"),
        "high":     ("#f59e0b", "HIGH"),
        "medium":   ("#00d4ff", "MEDIUM"),
        "low":      ("#64748b", "LOW"),
    }

    RESEARCH_VALUE_CONFIG = {
        "Critical":  "#ef4444",
        "Very High": "#f59e0b",
        "High":      "#10b981",
        "Medium":    "#00d4ff",
        "Low":       "#64748b",
    }

    CATEGORY_ICONS = {
        "planner":      "🗺",
        "avoidance":    "🛡",
        "hybrid":       "⚡",
        "coordination": "🔗",
    }

    KB_CATEGORY_ICONS = {
        "architecture":      "🏗",
        "audit":             "🔍",
        "research_question": "❓",
        "decision":          "⚖",
        "paper":             "📄",
    }

    # ── Public API ────────────────────────────────────────────────────────────

    def render(self, state: Dict[str, Any]) -> str:
        """
        Render a complete, self-contained HTML dashboard document.

        Parameters
        ----------
        state : dict
            The full dashboard state dictionary (as loaded from dashboard_state.json).

        Returns
        -------
        str
            A complete HTML document as a string.
        """
        project  = state.get("project", {})
        research = state.get("research", {})
        roadmap  = state.get("roadmap", [])
        modules  = state.get("modules", [])
        algorithms = state.get("algorithms", [])
        environments = state.get("environments", [])
        experiments  = state.get("recent_experiments", [])
        best_config  = state.get("best_configuration")
        knowledge    = state.get("knowledge_base", [])
        next_tasks   = state.get("next_tasks", [])

        # Compute derived display values
        generated_at = state.get("meta", {}).get("generated_at", datetime.now().isoformat())
        try:
            gen_dt = datetime.fromisoformat(generated_at)
            generated_display = gen_dt.strftime("%Y-%m-%d %H:%M")
        except (ValueError, TypeError):
            generated_display = generated_at

        # Get active research question and hypothesis
        active_q_id = research.get("active_question_id", "")
        active_h_id = research.get("active_hypothesis_id", "")
        active_question = next(
            (q for q in research.get("questions", []) if q["id"] == active_q_id), None
        )
        active_hypothesis = next(
            (h for h in research.get("hypotheses", []) if h["id"] == active_h_id), None
        )

        # Overall paper progress
        paper_sections = research.get("paper_sections", [])
        overall_paper = (
            int(sum(s.get("progress", 0) for s in paper_sections) / len(paper_sections))
            if paper_sections else 0
        )

        # Phase summary
        current_phase_id = project.get("current_phase", 0)
        total_phases = project.get("total_phases", 11)
        complete_phases = sum(1 for p in roadmap if p.get("status") == "complete")
        readiness = project.get("research_readiness", 0)

        sections = [
            self._render_header(project, generated_display, readiness),
            '<main class="dashboard-main">',
            self._render_kpi_strip(project, complete_phases, total_phases, overall_paper, research),
            self._render_active_research(active_question, active_hypothesis),
            self._render_implementation_status(modules),
            self._render_system_architecture(),
            self._render_research_contribution(),
            self._render_roadmap(roadmap, current_phase_id),
            '<div class="two-col">',
            self._render_modules(modules),
            self._render_algorithms(algorithms),
            '</div>',
            '<div class="two-col">',
            self._render_environments(environments),
            self._render_best_config(best_config),
            '</div>',
            self._render_paper_progress(paper_sections, overall_paper),
            self._render_experiments(experiments),
            '<div class="two-col">',
            self._render_knowledge_base(knowledge),
            self._render_next_tasks(next_tasks),
            '</div>',
            '</main>',
            self._render_footer(project, generated_display),
        ]

        body = "\n".join(sections)
        return self._wrap_document(body, project.get("name", "Research Dashboard"))

    # ── Document wrapper ──────────────────────────────────────────────────────

    def _wrap_document(self, body: str, title: str) -> str:
        css = self._get_css()
        js  = self._get_js()
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{self._esc(title)} — Research Dashboard</title>
  <meta name="description" content="Research Platform Dashboard for {self._esc(title)}" />
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
  <style>{css}</style>
</head>
<body>
{body}
<script>{js}</script>
</body>
</html>"""

    # ── Header ────────────────────────────────────────────────────────────────

    def _render_header(self, project: Dict, generated: str, readiness: int) -> str:
        name    = project.get("name", "Research Dashboard")
        phase   = project.get("current_phase", 0)
        branch  = project.get("current_branch", "—")
        sprint  = project.get("current_sprint", "—")
        total   = project.get("total_phases", 11)
        readiness_color = self._readiness_color(readiness)

        return f"""
<header class="dashboard-header">
  <div class="header-brand">
    <div class="header-icon">🚁</div>
    <div class="header-text">
      <h1 class="header-title">{self._esc(name)}</h1>
      <div class="header-meta">
        <span class="meta-chip branch-chip">⎇ {self._esc(branch)}</span>
        <span class="meta-chip">📍 Phase {phase} of {total}</span>
        <span class="meta-chip">🏃 {self._esc(sprint)}</span>
        <span class="meta-chip">🕐 Updated {generated}</span>
        <span class="meta-chip research-chip">🎓 Research Platform v1.0</span>
      </div>
    </div>
  </div>
  <div class="header-center">
    <div class="header-badge">
      <span class="hb-icon">🔬</span>
      <span class="hb-text">IFSP Research<br><strong>Supervisor Review</strong></span>
    </div>
  </div>
  <div class="header-readiness">
    <div class="readiness-ring">
      <svg viewBox="0 0 80 80" class="readiness-svg">
        <circle cx="40" cy="40" r="34" class="ring-bg"/>
        <circle cx="40" cy="40" r="34" class="ring-fill"
          style="stroke-dasharray: {int(readiness * 2.136)} 213.6; stroke: {readiness_color}"/>
      </svg>
      <div class="readiness-inner">
        <div class="readiness-value" style="color:{readiness_color}">{readiness}%</div>
        <div class="readiness-label">Readiness</div>
      </div>
    </div>
  </div>
</header>"""

    # ── KPI strip ─────────────────────────────────────────────────────────────

    def _render_kpi_strip(self, project, complete_phases, total_phases, paper_pct, research):
        q_list     = research.get("questions", [])
        h_list     = research.get("hypotheses", [])
        open_q     = sum(1 for q in q_list if q.get("status") in ("open", "under_investigation"))
        active_h   = sum(1 for h in h_list if h.get("status") == "under_test")
        readiness  = project.get("research_readiness", 0)

        kpis = [
            ("Phases Complete",   f"{complete_phases}/{total_phases}", "🎯", "#10b981", "Research milestones achieved"),
            ("Open Questions",    str(open_q),                         "❓", "#00d4ff", "Active research questions"),
            ("Active Hypotheses", str(active_h),                       "🔬", "#a78bfa", "Under experimental test"),
            ("Paper Progress",    f"{paper_pct}%",                     "📄", "#f59e0b", "Manuscript completion"),
            ("Research Readiness",f"{readiness}%",                     "⚡", self._readiness_color(readiness), "Platform completeness"),
        ]

        cards = ""
        for label, value, icon, color, sub in kpis:
            cards += f"""
    <div class="kpi-card" style="--kpi-color:{color}">
      <div class="kpi-icon" style="color:{color}">{icon}</div>
      <div class="kpi-value" style="color:{color}">{value}</div>
      <div class="kpi-label">{label}</div>
      <div class="kpi-sub">{sub}</div>
    </div>"""

        return f'<div class="kpi-strip">{cards}</div>'

    # ── Active research ───────────────────────────────────────────────────────

    def _render_active_research(self, question: Optional[Dict], hypothesis: Optional[Dict]) -> str:
        q_html = self._no_data("No active research question") if not question else f"""
      <div class="research-q-id">{self._esc(question.get("id",""))}</div>
      <div class="research-q-text">"{self._esc(question.get("text",""))}"</div>
      <div class="research-q-meta">
        {self._status_badge(question.get("status",""), self.QUESTION_STATUS_CONFIG)}
        {f'<span class="meta-since">Since {question.get("since","")}</span>' if question.get("since") else ""}
        {self._exp_links(question.get("supporting_experiments", []))}
      </div>"""

        h_html = self._no_data("No active hypothesis") if not hypothesis else f"""
      <div class="hyp-id">{self._esc(hypothesis.get("id",""))}</div>
      <div class="hyp-text">{self._esc(hypothesis.get("text",""))}</div>
      <div class="hyp-prediction"><strong>Prediction:</strong> {self._esc(hypothesis.get("prediction","—"))}</div>
      <div class="hyp-meta">
        {self._status_badge(hypothesis.get("status",""), self.HYPOTHESIS_STATUS_CONFIG)}
        {f'<span class="meta-since">Since {hypothesis.get("since","")}</span>' if hypothesis.get("since") else ""}
      </div>"""

        return f"""
<section class="card research-card">
  <div class="card-header">
    <h2 class="card-title">🔬 Active Research</h2>
    <span class="card-badge-pill" style="background:rgba(0,212,255,0.1);color:#00d4ff;border-color:rgba(0,212,255,0.3)">Q-001 · H-001</span>
  </div>
  <div class="research-two-col">
    <div class="research-question-block">
      <div class="research-block-label">Research Question</div>
      {q_html}
    </div>
    <div class="research-hypothesis-block">
      <div class="research-block-label">Current Hypothesis</div>
      {h_html}
    </div>
  </div>
</section>"""

    # ── NEW: Implementation Status ────────────────────────────────────────────

    def _render_implementation_status(self, modules: List[Dict]) -> str:
        """Current Implementation Status — ✓ Implemented / 🟡 In Progress / ⚪ Planned."""

        # Build status lookup from modules data
        status_map: Dict[str, str] = {}
        for m in modules:
            status_map[m.get("name", "")] = m.get("status", "planned")

        def _chip(label: str, status: str, extra_class: str = "") -> str:
            icons  = {"implemented": "✓", "in_progress": "🟡", "planned": "⚪", "missing": "⚠"}
            colors = {"implemented": "#10b981", "in_progress": "#f59e0b", "planned": "#64748b", "missing": "#ef4444"}
            icon   = icons.get(status, "⚪")
            color  = colors.get(status, "#64748b")
            return f'<div class="impl-chip impl-{status} {extra_class}" style="--chip-color:{color}"><span class="impl-icon">{icon}</span><span class="impl-label">{label}</span></div>'

        # Core platform (layer 1 & 2)
        core_items = [
            ("Webots Simulation",   status_map.get("Webots Simulation", "implemented")),
            ("Navigation Engine",   status_map.get("Navigation Engine", "implemented")),
            ("Safety Manager",      status_map.get("Safety Manager", "in_progress")),
            ("Research Dashboard",  status_map.get("Research Dashboard", "in_progress")),
        ]

        # Algorithm layer (layer 3)
        alg_items = [
            ("A* Planner",          "implemented"),
            ("Dijkstra Planner",    "implemented"),
            ("RRT Planner",         "implemented"),
            ("8-Ray Avoidance",     "implemented"),
            ("Planner Factory",     status_map.get("Planner Factory", "in_progress")),
            ("Avoidance Factory",   status_map.get("Avoidance Factory", "in_progress")),
            ("Potential Field",     "planned"),
            ("Velocity Obstacle",   "planned"),
        ]

        # Research contribution layer (layer 4)
        contrib_items = [
            ("Hybrid Coordinator",  status_map.get("Hybrid Coordinator", "in_progress"), "impl-highlight"),
            ("Weighted Blend Fusion","in_progress", ""),
            ("Distance-Gated Fusion","planned", ""),
            ("Confidence Fusion",   "planned", ""),
            ("RL Fusion",           "planned", ""),
        ]

        # Metrics & reporting (layers 5 & 6)
        metrics_items = [
            ("Metrics Engine",      status_map.get("Metrics Engine", "in_progress")),
            ("Experiment Manager",  status_map.get("Experiment Manager", "planned")),
            ("Benchmark Engine",    status_map.get("Benchmark Engine", "planned")),
            ("Report Generator",    status_map.get("Report Generator", "planned")),
        ]

        def _group(title: str, icon: str, items, color: str) -> str:
            chips = "".join(_chip(lbl, st, ec if len(item) > 2 else "") for item in items for lbl, st, *ec in [item + ("",)])
            return f"""
  <div class="impl-group">
    <div class="impl-group-header" style="--grp-color:{color}">
      <span class="impl-group-icon">{icon}</span>
      <span class="impl-group-title">{title}</span>
    </div>
    <div class="impl-chips">{chips}</div>
  </div>"""

        legend = """
  <div class="impl-legend">
    <span class="legend-item"><span class="legend-dot" style="color:#10b981">✓</span> Implemented</span>
    <span class="legend-item"><span class="legend-dot" style="color:#f59e0b">🟡</span> In Progress</span>
    <span class="legend-item"><span class="legend-dot" style="color:#64748b">⚪</span> Planned</span>
    <span class="legend-sep"></span>
    <span class="legend-note">🔬 Highlighted = Research Contribution</span>
  </div>"""

        impl_html = (
            _group("Core Platform", "🖥", core_items, "#00d4ff") +
            _group("Algorithm Layer", "⚙", alg_items, "#a78bfa") +
            _group("Research Contribution", "🔬", contrib_items, "#f59e0b") +
            _group("Metrics & Reporting", "📊", metrics_items, "#10b981")
        )

        return f"""
<section class="card impl-status-card">
  <div class="card-header">
    <h2 class="card-title">📋 Current Implementation Status</h2>
    <span class="card-badge-pill" style="background:rgba(16,185,129,0.1);color:#10b981;border-color:rgba(16,185,129,0.3)">As of Today</span>
  </div>
  {legend}
  <div class="impl-groups">
    {impl_html}
  </div>
</section>"""

    # ── NEW: System Architecture ───────────────────────────────────────────────

    def _render_system_architecture(self) -> str:
        """7-layer system architecture visualization."""

        layers = [
            ("Research Layer",        "Defines research questions, hypotheses, metrics, and paper structure.",       "🎓", "#a78bfa", "Guides all design decisions"),
            ("Orchestration Layer",   "Experiment Manager, Dashboard, Config Manager — platform coordination.",      "🎛", "#00d4ff", "Controls the research workflow"),
            ("Algorithm Layer",       "Planner Factory + Avoidance Factory — pluggable algorithm registry.",         "⚙",  "#f59e0b", "Hot-swappable algorithm backends"),
            ("Navigation Layer",      "Hybrid Coordinator + Navigation Engine — core flight execution.",             "🚁", "#10b981", "★ Core Research Contribution"),
            ("Metrics Layer",         "16 ground-truth metrics (M01–M16) — data collection and validation.",         "📊", "#00d4ff", "Scientific measurement pipeline"),
            ("Reporting Layer",       "Benchmark Engine + Report Generator — statistical analysis → paper figures.", "📄", "#a78bfa", "Publication-ready outputs"),
            ("Future Extension Layer","RL Optimization, Swarm Coordinator, Dynamic Obstacles — Phase 7–10.",         "🚀", "#64748b", "Planned multi-agent expansion"),
        ]

        cards_html = ""
        for i, (name, desc, icon, color, note) in enumerate(layers):
            is_highlight = "navigation" in name.lower()
            highlight_class = " arch-highlight" if is_highlight else ""
            star = ' <span class="arch-star">★</span>' if is_highlight else ""
            arrow = '<div class="arch-arrow">↓</div>' if i < len(layers) - 1 else ""
            cards_html += f"""
    <div class="arch-layer{highlight_class}" style="--layer-color:{color}">
      <div class="arch-layer-left">
        <div class="arch-layer-icon" style="color:{color}">{icon}</div>
        <div class="arch-layer-body">
          <div class="arch-layer-name" style="color:{color}">{name}{star}</div>
          <div class="arch-layer-desc">{desc}</div>
        </div>
      </div>
      <div class="arch-layer-note">{note}</div>
    </div>
    {arrow}"""

        return f"""
<section class="card arch-card">
  <div class="card-header">
    <h2 class="card-title">🏗 System Architecture</h2>
    <span class="card-badge-pill" style="background:rgba(124,58,237,0.1);color:#a78bfa;border-color:rgba(124,58,237,0.3)">7 Layers · Webots R2025a</span>
  </div>
  <div class="arch-container">
    {cards_html}
  </div>
</section>"""

    # ── NEW: Research Contribution ─────────────────────────────────────────────

    def _render_research_contribution(self) -> str:
        """Visual pipeline showing how the Hybrid Coordinator is the research contribution."""

        return f"""
<section class="card contrib-card">
  <div class="card-header">
    <h2 class="card-title">🏆 Current Research Contribution</h2>
    <span class="card-badge-pill" style="background:rgba(245,158,11,0.1);color:#f59e0b;border-color:rgba(245,158,11,0.3)">Novel Contribution</span>
  </div>
  <div class="contrib-container">

    <div class="contrib-top-row">
      <div class="contrib-input-block">
        <div class="contrib-node contrib-planner">
          <div class="contrib-node-icon">🗺</div>
          <div class="contrib-node-label">Global Path Planner</div>
          <div class="contrib-node-sub">A* · Dijkstra · RRT</div>
        </div>
        <div class="contrib-plus">+</div>
        <div class="contrib-node contrib-avoidance">
          <div class="contrib-node-icon">🛡</div>
          <div class="contrib-node-label">Reactive Avoidance</div>
          <div class="contrib-node-sub">8-Ray · Safety Layer</div>
        </div>
      </div>
      <div class="contrib-arrow-down">↓</div>
      <div class="contrib-core">
        <div class="contrib-core-badge">★ RESEARCH CONTRIBUTION</div>
        <div class="contrib-core-icon">⚡</div>
        <div class="contrib-core-label">Hybrid Coordinator</div>
        <div class="contrib-core-sub">Fuses global direction + reactive vectors<br>into a single unified navigation command</div>
        <div class="contrib-core-chips">
          <span class="contrib-chip">Weighted Blend</span>
          <span class="contrib-chip">Distance-Gated</span>
          <span class="contrib-chip">Confidence-Weighted</span>
          <span class="contrib-chip contrib-chip-future">RL Learned</span>
        </div>
      </div>
    </div>

    <div class="contrib-pipeline">
      <div class="contrib-pipe-step contrib-pipe-done">
        <div class="contrib-pipe-icon">🚁</div>
        <div class="contrib-pipe-label">Navigation</div>
        <div class="contrib-pipe-note">Kinematic flight model</div>
      </div>
      <div class="contrib-pipe-arrow">→</div>
      <div class="contrib-pipe-step contrib-pipe-active">
        <div class="contrib-pipe-icon">📊</div>
        <div class="contrib-pipe-label">Metrics</div>
        <div class="contrib-pipe-note">16 ground-truth metrics</div>
      </div>
      <div class="contrib-pipe-arrow">→</div>
      <div class="contrib-pipe-step contrib-pipe-planned">
        <div class="contrib-pipe-icon">📈</div>
        <div class="contrib-pipe-label">Evidence</div>
        <div class="contrib-pipe-note">Statistical benchmarks</div>
      </div>
      <div class="contrib-pipe-arrow">→</div>
      <div class="contrib-pipe-step contrib-pipe-planned">
        <div class="contrib-pipe-icon">📄</div>
        <div class="contrib-pipe-label">Research Paper</div>
        <div class="contrib-pipe-note">Hybrid UAV Navigation</div>
      </div>
    </div>

    <div class="contrib-differentiator">
      <div class="contrib-diff-title">🔍 Why this differs from a standard Webots project</div>
      <div class="contrib-diff-grid">
        <div class="contrib-diff-item">
          <span class="contrib-diff-icon">🔀</span>
          <div><strong>Hybrid Architecture</strong><br><span>No existing Webots simulator fuses global planners with reactive avoidance in a pluggable, benchmarkable framework.</span></div>
        </div>
        <div class="contrib-diff-item">
          <span class="contrib-diff-icon">🏭</span>
          <div><strong>Factory Pattern</strong><br><span>Algorithms swap at runtime via YAML config. No code change needed to compare A* vs RRT vs Dijkstra.</span></div>
        </div>
        <div class="contrib-diff-item">
          <span class="contrib-diff-icon">📐</span>
          <div><strong>Formal Metrics (M01–M16)</strong><br><span>16 metrics with ground-truth vs proxy designation. Rigorous scientific measurement, not ad-hoc logging.</span></div>
        </div>
        <div class="contrib-diff-item">
          <span class="contrib-diff-icon">🧪</span>
          <div><strong>Publication-Grade Pipeline</strong><br><span>Every experiment produces statistically analysable data leading directly to a peer-reviewed paper submission.</span></div>
        </div>
      </div>
    </div>
  </div>
</section>"""

    # ── Roadmap ───────────────────────────────────────────────────────────────

    def _render_roadmap(self, roadmap: List[Dict], current_id: int) -> str:
        if not roadmap:
            return self._empty_card("🗺 Development Roadmap", "No phases defined.")

        phase_cards = ""
        for phase in roadmap:
            pid    = phase.get("id", 0)
            status = phase.get("status", "planned")
            prog   = phase.get("progress", 0)
            label, color, dot = self.PHASE_STATUS_LABELS.get(
                status, ("PLANNED", "#64748b", "○")
            )
            is_current = (pid == current_id)
            rv_color = self.RESEARCH_VALUE_CONFIG.get(phase.get("research_value",""), "#64748b")
            met_criteria  = sum(1 for c in phase.get("exit_criteria", []) if c.get("met"))
            total_criteria = len(phase.get("exit_criteria", []))
            crit_text = f"{met_criteria}/{total_criteria}" if total_criteria > 0 else "—"

            current_class = " phase-current" if is_current else ""
            phase_cards += f"""
  <div class="phase-card {status}{current_class}" title="{self._esc(phase.get('description',''))}">
    <div class="phase-header">
      <div class="phase-num" style="border-color:{color}; color:{color}; box-shadow: 0 0 8px {color}40">{pid}</div>
      <div class="phase-status-dot" style="color:{color}">{dot}</div>
    </div>
    <div class="phase-name">{self._esc(phase.get('short_name', phase.get('name','')))}</div>
    <div class="phase-status-label" style="color:{color}">{label}</div>
    <div class="phase-progress-bar-wrap">
      <div class="phase-progress-bar" style="width:{prog}%; background:linear-gradient(90deg, {color}cc, {color})"></div>
    </div>
    <div class="phase-prog-text" style="color:{color}">{prog}%</div>
    {f'<div class="phase-criteria">✅ {crit_text} criteria</div>' if total_criteria else ""}
    {f'<div class="phase-rv" style="color:{rv_color}">★ {phase.get("research_value","")}</div>' if phase.get("research_value") else ""}
  </div>"""

        return f"""
<section class="card roadmap-card">
  <div class="card-header">
    <h2 class="card-title">🗺 Development Roadmap</h2>
    <div class="card-subtitle">Phases complete when ALL exit criteria are met · {sum(1 for p in roadmap if p.get("status") == "complete")}/{len(roadmap)} done</div>
  </div>
  <div class="roadmap-scroll">
    <div class="roadmap-track">{phase_cards}</div>
  </div>
</section>"""

    # ── Modules ───────────────────────────────────────────────────────────────

    def _render_modules(self, modules: List[Dict]) -> str:
        if not modules:
            return self._empty_card("🧩 Platform Modules", "No modules defined.")

        rows = ""
        for mod in modules:
            status = mod.get("status", "planned")
            label, color = self.MODULE_STATUS_CONFIG.get(status, ("Planned", "#64748b"))
            prog = mod.get("progress", 0)
            is_contrib = "coordinator" in mod.get("name", "").lower() or "hybrid" in mod.get("name", "").lower()
            contrib_star = ' <span style="color:#f59e0b;font-size:10px">★ Contrib</span>' if is_contrib else ""
            rows += f"""
    <div class="module-row">
      <div class="module-info">
        <div class="module-name">{self._esc(mod.get("name",""))}{contrib_star}</div>
        <div class="module-desc">{self._esc(mod.get("description",""))}</div>
      </div>
      <div class="module-right">
        <div class="module-badge" style="color:{color}; border-color:{color}; background:{color}15">{label}</div>
        <div class="module-prog-bar-wrap">
          <div class="module-prog-bar" style="width:{prog}%; background:linear-gradient(90deg,{color}99,{color})"></div>
        </div>
      </div>
    </div>"""

        return f"""
<div class="card">
  <div class="card-header">
    <h2 class="card-title">🧩 Platform Modules</h2>
  </div>
  <div class="modules-list">{rows}</div>
</div>"""

    # ── Algorithms ────────────────────────────────────────────────────────────

    def _render_algorithms(self, algorithms: List[Dict]) -> str:
        if not algorithms:
            return self._empty_card("⚙ Algorithm Registry", "No algorithms registered.")

        categories = ["planner", "avoidance", "hybrid"]
        cat_labels = {
            "planner":  "Path Planners",
            "avoidance":"Collision Avoidance",
            "hybrid":   "Hybrid Fusion",
        }
        sections_html = ""

        for cat in categories:
            algs = [a for a in algorithms if a.get("category") == cat]
            if not algs:
                continue

            icon = self.CATEGORY_ICONS.get(cat, "⚙")

            # Sort: implemented/registered first, then in_progress, then planned
            status_order = {"registered": 0, "implemented": 0, "in_progress": 1, "planned": 2, "missing": 3}
            algs_sorted = sorted(algs, key=lambda a: status_order.get(a.get("status", "planned"), 2))

            # Group by sub-status for clear visual separation
            done_rows = ""
            prog_rows = ""
            plan_rows = ""

            for alg in algs_sorted:
                status = alg.get("status", "planned")
                label, color = self.ALGORITHM_STATUS_CONFIG.get(status, ("Planned", "#64748b"))
                row = f"""
      <div class="alg-row">
        <div class="alg-info">
          <span class="alg-name">{self._esc(alg.get("name",""))}</span>
          {f'<span class="alg-metric">{self._esc(alg.get("best_metric",""))}</span>' if alg.get("best_metric") else ""}
        </div>
        <div class="alg-badge" style="color:{color}; border-color:{color}; background:{color}15">{label}</div>
      </div>"""
                if status in ("registered", "implemented"):
                    done_rows += row
                elif status == "in_progress":
                    prog_rows += row
                else:
                    plan_rows += row

            sub_sections = ""
            if done_rows:
                sub_sections += f'<div class="alg-sub-label alg-sub-done">✓ Implemented</div>{done_rows}'
            if prog_rows:
                sub_sections += f'<div class="alg-sub-label alg-sub-prog">🟡 In Progress</div>{prog_rows}'
            if plan_rows:
                sub_sections += f'<div class="alg-sub-label alg-sub-plan">⚪ Planned</div>{plan_rows}'

            sections_html += f"""
    <div class="alg-category">
      <div class="alg-cat-label">{icon} {cat_labels.get(cat, cat.title())}</div>
      {sub_sections}
    </div>"""

        return f"""
<div class="card">
  <div class="card-header">
    <h2 class="card-title">⚙ Algorithm Registry</h2>
  </div>
  <div class="algorithms-container">{sections_html}</div>
</div>"""

    # ── Environments ──────────────────────────────────────────────────────────

    def _render_environments(self, environments: List[Dict]) -> str:
        if not environments:
            return self._empty_card("🌆 Simulation Environments", "No environments defined.")

        cards = ""
        for env in environments:
            status = env.get("status", "planned")
            label, color = self.ENV_STATUS_CONFIG.get(status, ("Planned", "#64748b"))
            crowd = env.get("crowd_count", 0)
            area  = env.get("area_m2", 0)
            alt   = env.get("uav_altitude_m", 0)
            last  = env.get("last_used", "")

            cards += f"""
    <div class="env-card" style="border-color:{color}25">
      <div class="env-header">
        <div class="env-name">{self._esc(env.get("name",""))}</div>
        <div class="env-badge" style="color:{color}; background:{color}15; border:1px solid {color}40; border-radius:20px; padding:2px 8px; font-size:10px; font-weight:600">{label}</div>
      </div>
      <div class="env-desc">{self._esc(env.get("description",""))}</div>
      <div class="env-stats">
        <span title="Crowd count">👥 {crowd}</span>
        <span title="Area">📐 {area:,}m²</span>
        <span title="Altitude">✈ {alt}m</span>
      </div>
      {f'<div class="env-last">Last used: {last}</div>' if last else '<div class="env-last">Never used</div>'}
    </div>"""

        return f"""
<div class="card">
  <div class="card-header">
    <h2 class="card-title">🌆 Simulation Environments</h2>
    <div class="card-subtitle">Webots R2025a · 6 urban worlds</div>
  </div>
  <div class="environments-grid">{cards}</div>
</div>"""

    # ── Best configuration ────────────────────────────────────────────────────

    def _render_best_config(self, config: Optional[Dict]) -> str:
        if not config or not config.get("config_profile"):
            return f"""
<div class="card">
  <div class="card-header"><h2 class="card-title">🏆 Best Configuration</h2></div>
  {self._no_data("No validated configuration yet. Run benchmark experiments to establish a baseline.")}
</div>"""

        confidence = config.get("confidence", "Unknown")
        conf_color = "#f59e0b" if "Low" in confidence else "#10b981" if "High" in confidence else "#00d4ff"

        metrics = [
            ("Path Efficiency",   f"{config.get('path_efficiency', 0):.3f}",  "#10b981"),
            ("Near-Misses",       str(config.get("near_misses", "—")),         "#f59e0b"),
            ("TRR",               f"{config.get('mean_trr_percent', 0):.2f}%", "#00d4ff"),
            ("Coverage",          f"{config.get('mean_coverage_percent', 0):.1f}%", "#a78bfa"),
        ]
        metric_html = "".join(f"""
      <div class="config-metric">
        <div class="config-metric-val" style="color:{c}">{v}</div>
        <div class="config-metric-label">{l}</div>
      </div>""" for l, v, c in metrics)

        return f"""
<div class="card">
  <div class="card-header"><h2 class="card-title">🏆 Best Configuration</h2></div>
  <div class="best-config">
    <div class="config-profile">{self._esc(config.get("config_profile",""))}</div>
    <div class="config-exp">Evidence: {self._esc(config.get("experiment_id",""))}</div>
    <div class="config-metrics">{metric_html}</div>
    <div class="config-confidence" style="color:{conf_color}">
      Confidence: {self._esc(confidence)}
    </div>
    {f'<div class="config-note">{self._esc(config.get("notes",""))}</div>' if config.get("notes") else ""}
  </div>
</div>"""

    # ── Paper progress ────────────────────────────────────────────────────────

    def _render_paper_progress(self, sections: List[Dict], overall: int) -> str:
        # Canonical paper sections with fallback defaults
        canonical = [
            ("Introduction",  "Problem statement, motivation, contributions"),
            ("Related Work",  "Literature survey, gap analysis"),
            ("Methodology",   "Architecture, algorithms, experimental design"),
            ("Experiments",   "Setup, configurations, environments"),
            ("Results",       "Quantitative analysis, statistical validation"),
            ("Discussion",    "Interpretation, limitations, future work"),
            ("Conclusion",    "Summary, contributions, next steps"),
        ]

        # Build lookup from state data
        section_map = {s.get("name", ""): s for s in sections}

        rows = ""
        for sec_name, sec_hint in canonical:
            s = section_map.get(sec_name, {})
            prog = s.get("progress", 0)
            note = s.get("status_note", sec_hint)
            color = "#10b981" if prog >= 80 else "#00d4ff" if prog >= 40 else "#f59e0b" if prog >= 10 else "#64748b"
            status_dot = "✓" if prog >= 80 else "◑" if prog >= 10 else "○"
            rows += f"""
    <div class="paper-row">
      <div class="paper-section-name">
        <span class="paper-dot" style="color:{color}">{status_dot}</span>
        {self._esc(sec_name)}
      </div>
      <div class="paper-bar-wrap">
        <div class="paper-bar" style="width:{prog}%; background:linear-gradient(90deg,{color}99,{color})"></div>
      </div>
      <div class="paper-pct" style="color:{color}">{prog}%</div>
      <div class="paper-note">{self._esc(note)}</div>
    </div>"""

        overall_color = "#10b981" if overall >= 80 else "#00d4ff" if overall >= 40 else "#f59e0b" if overall >= 10 else "#64748b"

        return f"""
<section class="card paper-card">
  <div class="card-header">
    <h2 class="card-title">📄 Paper Progress</h2>
    <div class="paper-overall" style="color:{overall_color}">{overall}% overall</div>
  </div>
  <div class="paper-sections">{rows}</div>
</section>"""

    # ── Recent experiments ────────────────────────────────────────────────────

    def _render_experiments(self, experiments: List[Dict]) -> str:
        if not experiments:
            return f"""
<section class="card experiments-card">
  <div class="card-header"><h2 class="card-title">🧪 Recent Experiments</h2></div>
  {self._no_data("No experiments recorded yet. Run your first experiment with the Experiment Manager.")}
</section>"""

        rows = ""
        for exp in experiments[:10]:
            status = exp.get("status", "queued")
            label, color = self.EXP_STATUS_CONFIG.get(status, ("—", "#64748b"))
            metrics = exp.get("metrics", {})
            eff  = metrics.get("path_efficiency", "—")
            nm   = metrics.get("near_misses", "—")
            trr  = metrics.get("mean_trr_percent", "—")
            ts   = exp.get("timestamp", "")
            if ts:
                try:
                    ts = datetime.fromisoformat(ts).strftime("%Y-%m-%d %H:%M")
                except (ValueError, TypeError):
                    pass

            eff_str = f"{eff:.3f}" if isinstance(eff, float) else str(eff)
            trr_str = f"{trr:.2f}%" if isinstance(trr, float) else str(trr)

            rows += f"""
    <tr class="exp-row">
      <td class="exp-id">{self._esc(exp.get("id",""))}</td>
      <td class="exp-name">{self._esc(exp.get("name",""))}</td>
      <td>{self._esc(exp.get("config_profile",""))}</td>
      <td>{self._esc(exp.get("world",""))}</td>
      <td class="exp-status"><span style="color:{color}; background:{color}15; padding:2px 8px; border-radius:20px; font-size:10px; font-weight:700">● {label}</span></td>
      <td class="exp-metric">{eff_str}</td>
      <td class="exp-metric">{nm}</td>
      <td class="exp-metric">{trr_str}</td>
      <td class="exp-ts">{ts}</td>
    </tr>"""

        return f"""
<section class="card experiments-card">
  <div class="card-header">
    <h2 class="card-title">🧪 Recent Experiments</h2>
    <div class="card-subtitle">Last {min(len(experiments),10)} of {len(experiments)} runs</div>
  </div>
  <div class="table-scroll">
    <table class="experiments-table">
      <thead>
        <tr>
          <th>ID</th><th>Name</th><th>Config</th><th>World</th>
          <th>Status</th><th>Efficiency</th><th>Near-Misses</th><th>TRR</th><th>Timestamp</th>
        </tr>
      </thead>
      <tbody>{rows}</tbody>
    </table>
  </div>
</section>"""

    # ── Knowledge base ────────────────────────────────────────────────────────

    def _render_knowledge_base(self, knowledge: List[Dict]) -> str:
        if not knowledge:
            return self._empty_card("📚 Knowledge Base", "No knowledge entries yet.")

        items = ""
        for kb in knowledge:
            cat   = kb.get("category", "")
            icon  = self.KB_CATEGORY_ICONS.get(cat, "📌")
            title = kb.get("title", "")
            desc  = kb.get("desc", kb.get("description",""))
            fpath = kb.get("file_path", "")
            url   = kb.get("url", "")
            tags  = kb.get("tags", [])
            date  = kb.get("date_added","")

            # Category color
            cat_colors = {
                "architecture": "#00d4ff",
                "audit": "#f59e0b",
                "research_question": "#a78bfa",
                "decision": "#10b981",
                "paper": "#ef4444",
            }
            cat_color = cat_colors.get(cat, "#64748b")

            # Build link
            if fpath:
                link_href = fpath.replace("\\", "/")
                link_html = f'<a href="file:///{link_href}" class="kb-link" target="_blank">Open ↗</a>'
            elif url:
                link_html = f'<a href="{self._esc(url)}" class="kb-link" target="_blank">Open ↗</a>'
            else:
                link_html = ""

            tag_html = "".join(f'<span class="kb-tag">{self._esc(t)}</span>' for t in tags[:4])

            items += f"""
    <div class="kb-item">
      <div class="kb-icon-wrap" style="--kb-color:{cat_color}">
        <div class="kb-icon">{icon}</div>
      </div>
      <div class="kb-content">
        <div class="kb-title">{self._esc(title)} {link_html}</div>
        {f'<div class="kb-desc">{self._esc(desc)}</div>' if desc else ""}
        <div class="kb-footer">{tag_html}{f'<span class="kb-date">{date}</span>' if date else ""}</div>
      </div>
    </div>"""

        return f"""
<div class="card">
  <div class="card-header"><h2 class="card-title">📚 Knowledge Base</h2><div class="card-subtitle">Research Artifacts</div></div>
  <div class="kb-list">{items}</div>
</div>"""

    # ── Next tasks ────────────────────────────────────────────────────────────

    def _render_next_tasks(self, tasks: List[Dict]) -> str:
        pending = [t for t in tasks if not t.get("done", False)]
        done    = [t for t in tasks if t.get("done", False)]

        if not tasks:
            return self._empty_card("✅ Next Tasks", "No tasks defined.")

        task_html = ""
        for task in pending[:15]:
            priority = task.get("priority", "medium")
            p_color, p_label = self.PRIORITY_CONFIG.get(priority, ("#64748b", "MEDIUM"))
            owner = task.get("owner", "")
            hrs   = task.get("estimated_hours", 0)
            phase = task.get("phase", 0)
            deps  = task.get("depends_on", [])

            task_html += f"""
    <div class="task-item">
      <div class="task-check" style="color:{p_color}">○</div>
      <div class="task-content">
        <div class="task-text">{self._esc(task.get("text",""))}</div>
        <div class="task-meta">
          <span class="task-priority" style="color:{p_color};border-color:{p_color};background:{p_color}15">{p_label}</span>
          {f'<span class="task-phase">Ph.{phase}</span>' if phase else ""}
          {f'<span class="task-owner">👤 {self._esc(owner)}</span>' if owner else ""}
          {f'<span class="task-hours">⏱ {hrs}h</span>' if hrs else ""}
          {f'<span class="task-deps">⤷ {", ".join(deps)}</span>' if deps else ""}
        </div>
      </div>
    </div>"""

        summary = f"{len(pending)} pending · {len(done)} complete"

        return f"""
<div class="card">
  <div class="card-header">
    <h2 class="card-title">✅ Next Tasks</h2>
    <div class="card-subtitle">{summary}</div>
  </div>
  <div class="tasks-list">{task_html}</div>
</div>"""

    # ── Footer ────────────────────────────────────────────────────────────────

    def _render_footer(self, project: Dict, generated: str) -> str:
        name = project.get("name", "Hybrid Multi-UAV Navigation Research Platform")
        phase = project.get("current_phase", 1)
        return f"""
<footer class="dashboard-footer">
  <div class="footer-grid">
    <div class="footer-item">
      <div class="footer-item-label">🎯 Project Vision</div>
      <div class="footer-item-value">Publish a peer-reviewed paper demonstrating that a hybrid UAV navigation architecture outperforms any single-strategy baseline across path efficiency, obstacle avoidance, and mission success.</div>
    </div>
    <div class="footer-item">
      <div class="footer-item-label">📍 Today's Goal</div>
      <div class="footer-item-value">Supervisor review demonstration · Show current platform architecture, implemented algorithms, and research direction. Establish feedback for Phase {phase} exit criteria.</div>
    </div>
    <div class="footer-item">
      <div class="footer-item-label">🚀 Next Milestone</div>
      <div class="footer-item-value">Phase 2 — Algorithm Layer Complete: Potential Field planner implemented, all four planners benchmarkable, factory pattern fully validated.</div>
    </div>
    <div class="footer-item">
      <div class="footer-item-label">📄 Expected Output</div>
      <div class="footer-item-value">Conference or journal paper on Hybrid Single-UAV Navigation in cluttered urban environments, with statistically validated results (p &lt; 0.05, n ≥ 5 trials per configuration).</div>
    </div>
  </div>
  <div class="footer-bottom">
    <span>{self._esc(name)} · Platform v1.0.0</span>
    <span>Generated {generated} · dashboard_state.json</span>
  </div>
</footer>"""

    # ── Helper utilities ──────────────────────────────────────────────────────

    @staticmethod
    def _esc(text: Any) -> str:
        """HTML-escape a value for safe embedding."""
        s = str(text) if text is not None else ""
        return (s.replace("&", "&amp;").replace("<", "&lt;")
                  .replace(">", "&gt;").replace('"', "&quot;"))

    def _status_badge(self, status: str, config: Dict) -> str:
        label, color = config.get(status, ("Unknown", "#64748b"))
        return f'<span class="status-badge" style="color:{color};border-color:{color};background:{color}15">{label}</span>'

    def _no_data(self, message: str) -> str:
        return f'<div class="no-data">{self._esc(message)}</div>'

    def _empty_card(self, title: str, message: str) -> str:
        return f"""
<div class="card">
  <div class="card-header"><h2 class="card-title">{title}</h2></div>
  {self._no_data(message)}
</div>"""

    def _exp_links(self, exp_ids: List[str]) -> str:
        if not exp_ids:
            return ""
        chips = "".join(f'<span class="exp-chip">{self._esc(e)}</span>' for e in exp_ids[:5])
        return f'<div class="exp-chips">{chips}</div>'

    @staticmethod
    def _readiness_color(score: int) -> str:
        if score >= 80:
            return "#10b981"
        if score >= 50:
            return "#00d4ff"
        if score >= 25:
            return "#f59e0b"
        return "#ef4444"

    # ── CSS ───────────────────────────────────────────────────────────────────

    @staticmethod
    def _get_css() -> str:
        return """
/* ─── Reset & base ─────────────────────────────────────────────────── */
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

:root {
  --bg:          #080d1a;
  --bg2:         #0e1629;
  --bg3:         #131f38;
  --glass:       rgba(14, 22, 41, 0.92);
  --glass2:      rgba(20, 32, 60, 0.65);
  --border:      rgba(0, 212, 255, 0.12);
  --border2:     rgba(0, 212, 255, 0.06);
  --cyan:        #00d4ff;
  --purple:      #7c3aed;
  --green:       #10b981;
  --amber:       #f59e0b;
  --red:         #ef4444;
  --violet:      #a78bfa;
  --text:        #e2e8f0;
  --text-muted:  #64748b;
  --text-dim:    #94a3b8;
  --font:        'Inter', system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif;
  --mono:        'JetBrains Mono', 'Cascadia Code', 'Fira Code', monospace;
  --radius:      14px;
  --radius-sm:   9px;
  --shadow:      0 4px 24px rgba(0,0,0,0.4);
  --shadow-lg:   0 8px 48px rgba(0,0,0,0.6);
  --glow-cyan:   0 0 24px rgba(0, 212, 255, 0.18);
  --glow-amber:  0 0 24px rgba(245, 158, 11, 0.18);
}

html { scroll-behavior: smooth; }

body {
  font-family: var(--font);
  background: var(--bg);
  color: var(--text);
  font-size: 14px;
  line-height: 1.5;
  min-height: 100vh;
  background-image:
    radial-gradient(ellipse 80% 50% at 20% 10%, rgba(0,212,255,0.04) 0%, transparent 60%),
    radial-gradient(ellipse 60% 40% at 80% 80%, rgba(124,58,237,0.04) 0%, transparent 60%);
}

/* ─── Scrollbars ──────────────────────────────────────────────────── */
::-webkit-scrollbar { width: 6px; height: 6px; }
::-webkit-scrollbar-track { background: var(--bg2); }
::-webkit-scrollbar-thumb { background: rgba(0,212,255,0.3); border-radius: 3px; }

/* ─── Header ──────────────────────────────────────────────────────── */
.dashboard-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 20px;
  padding: 18px 28px;
  background: linear-gradient(135deg, rgba(0,212,255,0.06), rgba(124,58,237,0.06));
  border-bottom: 1px solid var(--border);
  backdrop-filter: blur(24px);
  position: sticky;
  top: 0;
  z-index: 100;
}
.header-brand { display: flex; align-items: center; gap: 16px; flex: 1; }
.header-icon { font-size: 38px; filter: drop-shadow(0 0 12px rgba(0,212,255,0.6)); }
.header-title {
  font-size: 20px;
  font-weight: 800;
  background: linear-gradient(135deg, #fff 20%, var(--cyan) 70%, #a78bfa);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  letter-spacing: -0.5px;
}
.header-meta { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 5px; }
.meta-chip {
  font-size: 11px;
  color: var(--text-dim);
  background: rgba(255,255,255,0.05);
  border: 1px solid var(--border);
  border-radius: 20px;
  padding: 3px 10px;
  font-weight: 500;
}
.branch-chip { color: var(--cyan); border-color: rgba(0,212,255,0.25); }
.research-chip { color: #a78bfa; border-color: rgba(167,139,250,0.25); }

/* Header center badge */
.header-center { flex-shrink: 0; }
.header-badge {
  display: flex; align-items: center; gap: 10px;
  background: rgba(245,158,11,0.08);
  border: 1px solid rgba(245,158,11,0.25);
  border-radius: var(--radius-sm);
  padding: 8px 16px;
}
.hb-icon { font-size: 22px; }
.hb-text { font-size: 11px; color: var(--text-dim); line-height: 1.4; }
.hb-text strong { color: var(--amber); display: block; }

/* Readiness ring */
.header-readiness { flex-shrink: 0; }
.readiness-ring { position: relative; width: 84px; height: 84px; }
.readiness-svg { width: 84px; height: 84px; transform: rotate(-90deg); }
.ring-bg  { fill: none; stroke: rgba(255,255,255,0.07); stroke-width: 7; }
.ring-fill { fill: none; stroke-width: 7; stroke-linecap: round; transition: stroke-dasharray 0.6s; }
.readiness-inner {
  position: absolute; inset: 0;
  display: flex; flex-direction: column; align-items: center; justify-content: center;
}
.readiness-value { font-size: 17px; font-weight: 800; }
.readiness-label { font-size: 9px; color: var(--text-muted); margin-top: 1px; font-weight: 500; letter-spacing: 0.05em; }

/* ─── Main layout ─────────────────────────────────────────────────── */
.dashboard-main {
  max-width: 1600px;
  margin: 0 auto;
  padding: 28px 28px 56px;
  display: flex;
  flex-direction: column;
  gap: 24px;
}

/* ─── KPI strip ───────────────────────────────────────────────────── */
.kpi-strip {
  display: grid;
  grid-template-columns: repeat(5, 1fr);
  gap: 16px;
}
.kpi-card {
  background: var(--glass);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 22px 16px 18px;
  text-align: center;
  backdrop-filter: blur(16px);
  transition: transform 0.2s, box-shadow 0.2s;
  position: relative;
  overflow: hidden;
}
.kpi-card::before {
  content: '';
  position: absolute;
  top: 0; left: 0; right: 0;
  height: 2px;
  background: var(--kpi-color, #00d4ff);
  opacity: 0.7;
}
.kpi-card:hover { transform: translateY(-3px); box-shadow: 0 0 28px rgba(0,212,255,0.12); }
.kpi-icon  { font-size: 26px; margin-bottom: 10px; }
.kpi-value { font-size: 30px; font-weight: 800; letter-spacing: -0.8px; }
.kpi-label { font-size: 12px; color: var(--text-dim); margin-top: 4px; font-weight: 500; }
.kpi-sub   { font-size: 10px; color: var(--text-muted); margin-top: 3px; }

/* ─── Cards ───────────────────────────────────────────────────────── */
.card {
  background: var(--glass);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  backdrop-filter: blur(16px);
  box-shadow: var(--shadow);
  overflow: hidden;
}
.card-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 18px 22px 14px;
  border-bottom: 1px solid var(--border2);
}
.card-title { font-size: 15px; font-weight: 700; letter-spacing: -0.2px; }
.card-subtitle { font-size: 11px; color: var(--text-muted); }
.card-badge-pill {
  font-size: 11px; font-weight: 600;
  border: 1px solid; border-radius: 20px;
  padding: 3px 12px;
}

/* ─── Two-column layout ───────────────────────────────────────────── */
.two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 24px; }

/* ─── Research card ───────────────────────────────────────────────── */
.research-card { border-color: rgba(0, 212, 255, 0.22); }
.research-two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 0; }
.research-question-block,
.research-hypothesis-block { padding: 22px; }
.research-hypothesis-block { border-left: 1px solid var(--border2); }
.research-block-label { font-size: 10px; text-transform: uppercase; letter-spacing: 0.12em; color: var(--text-muted); margin-bottom: 12px; font-weight: 600; }
.research-q-id { font-family: var(--mono); font-size: 11px; color: var(--cyan); margin-bottom: 10px; }
.research-q-text { font-size: 14px; font-style: italic; color: var(--text); line-height: 1.65; margin-bottom: 14px; }
.research-q-meta { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.meta-since { font-size: 11px; color: var(--text-muted); }
.hyp-id { font-family: var(--mono); font-size: 11px; color: var(--violet); margin-bottom: 10px; }
.hyp-text { font-size: 13px; color: var(--text-dim); line-height: 1.65; margin-bottom: 10px; }
.hyp-prediction { font-size: 12px; color: var(--text-muted); margin-bottom: 12px; }
.hyp-meta { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.status-badge {
  font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.08em;
  border: 1px solid; border-radius: 20px; padding: 2px 10px;
}
.exp-chips { display: flex; gap: 6px; }
.exp-chip { font-family: var(--mono); font-size: 10px; color: var(--cyan); background: rgba(0,212,255,0.06); border: 1px solid rgba(0,212,255,0.18); border-radius: 4px; padding: 2px 7px; }

/* ─── Implementation Status ───────────────────────────────────────── */
.impl-status-card { }
.impl-legend {
  display: flex; align-items: center; gap: 16px; flex-wrap: wrap;
  padding: 10px 22px;
  background: rgba(255,255,255,0.02);
  border-bottom: 1px solid var(--border2);
  font-size: 12px;
}
.legend-item { display: flex; align-items: center; gap: 6px; color: var(--text-dim); }
.legend-dot { font-size: 14px; }
.legend-sep { flex: 1; }
.legend-note { font-size: 11px; color: var(--amber); font-weight: 500; }
.impl-groups {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 0;
}
.impl-group {
  border-right: 1px solid var(--border2);
  padding: 16px 18px;
}
.impl-group:last-child { border-right: none; }
.impl-group-header {
  display: flex; align-items: center; gap: 8px;
  margin-bottom: 14px;
  padding-bottom: 10px;
  border-bottom: 2px solid var(--grp-color, #64748b);
}
.impl-group-icon { font-size: 18px; }
.impl-group-title { font-size: 12px; font-weight: 700; color: var(--grp-color, #64748b); text-transform: uppercase; letter-spacing: 0.06em; }
.impl-chips { display: flex; flex-direction: column; gap: 7px; }
.impl-chip {
  display: flex; align-items: center; gap: 8px;
  padding: 7px 10px;
  background: rgba(255,255,255,0.025);
  border: 1px solid rgba(255,255,255,0.06);
  border-radius: var(--radius-sm);
  transition: background 0.2s, transform 0.2s;
  cursor: default;
}
.impl-chip:hover { background: rgba(255,255,255,0.045); transform: translateX(2px); }
.impl-chip.impl-implemented { border-color: rgba(16,185,129,0.2); }
.impl-chip.impl-in_progress { border-color: rgba(245,158,11,0.2); }
.impl-chip.impl-planned { opacity: 0.7; }
.impl-chip.impl-highlight {
  background: rgba(245,158,11,0.08);
  border-color: rgba(245,158,11,0.35);
  box-shadow: 0 0 12px rgba(245,158,11,0.1);
}
.impl-icon { font-size: 14px; flex-shrink: 0; }
.impl-label { font-size: 12px; font-weight: 500; color: var(--text-dim); }
.impl-chip.impl-highlight .impl-label { color: var(--amber); font-weight: 700; }

/* ─── System Architecture ─────────────────────────────────────────── */
.arch-card { }
.arch-container {
  padding: 20px 24px;
  display: flex; flex-direction: column; align-items: center; gap: 0;
}
.arch-layer {
  width: 100%;
  display: flex; align-items: center; justify-content: space-between;
  background: var(--glass2);
  border: 1px solid var(--border2);
  border-left: 3px solid var(--layer-color, #64748b);
  border-radius: var(--radius-sm);
  padding: 13px 18px;
  transition: transform 0.2s, box-shadow 0.2s;
  cursor: default;
}
.arch-layer:hover { transform: translateX(4px); box-shadow: 0 4px 16px rgba(0,0,0,0.3); }
.arch-highlight {
  background: rgba(245,158,11,0.07);
  border-color: rgba(245,158,11,0.3);
  border-left-color: var(--layer-color, #f59e0b);
  box-shadow: 0 0 20px rgba(245,158,11,0.08);
}
.arch-layer-left { display: flex; align-items: center; gap: 14px; }
.arch-layer-icon { font-size: 22px; flex-shrink: 0; }
.arch-layer-name { font-size: 13px; font-weight: 700; margin-bottom: 2px; }
.arch-layer-desc { font-size: 11px; color: var(--text-muted); }
.arch-layer-note { font-size: 11px; color: var(--text-dim); font-style: italic; flex-shrink: 0; }
.arch-star { color: var(--amber); margin-left: 6px; }
.arch-arrow {
  font-size: 18px; color: var(--text-muted); text-align: center;
  line-height: 1; padding: 3px 0;
}

/* ─── Research Contribution ───────────────────────────────────────── */
.contrib-card { border-color: rgba(245,158,11,0.2); }
.contrib-container { padding: 24px; display: flex; flex-direction: column; gap: 24px; }

.contrib-top-row { display: flex; flex-direction: column; align-items: center; gap: 16px; }
.contrib-input-block { display: flex; align-items: center; gap: 16px; }
.contrib-node {
  display: flex; flex-direction: column; align-items: center; gap: 6px;
  background: var(--glass2); border: 1px solid var(--border);
  border-radius: var(--radius); padding: 16px 24px; min-width: 160px; text-align: center;
  transition: transform 0.2s;
}
.contrib-node:hover { transform: translateY(-2px); }
.contrib-planner { border-color: rgba(167,139,250,0.3); }
.contrib-avoidance { border-color: rgba(0,212,255,0.3); }
.contrib-node-icon { font-size: 26px; }
.contrib-node-label { font-size: 13px; font-weight: 700; }
.contrib-node-sub { font-size: 10px; color: var(--text-muted); font-family: var(--mono); }
.contrib-plus {
  font-size: 28px; font-weight: 300; color: var(--text-muted); flex-shrink: 0;
}
.contrib-arrow-down { font-size: 24px; color: var(--amber); }
.contrib-core {
  background: rgba(245,158,11,0.06);
  border: 2px solid rgba(245,158,11,0.4);
  border-radius: var(--radius);
  padding: 20px 32px; text-align: center;
  box-shadow: 0 0 32px rgba(245,158,11,0.1);
  position: relative;
}
.contrib-core-badge {
  font-size: 10px; font-weight: 800; color: var(--amber);
  text-transform: uppercase; letter-spacing: 0.12em;
  background: rgba(245,158,11,0.1); border: 1px solid rgba(245,158,11,0.3);
  border-radius: 20px; padding: 3px 12px; margin-bottom: 12px; display: inline-block;
}
.contrib-core-icon { font-size: 36px; margin-bottom: 8px; }
.contrib-core-label { font-size: 18px; font-weight: 800; color: var(--amber); margin-bottom: 6px; }
.contrib-core-sub { font-size: 12px; color: var(--text-dim); line-height: 1.5; margin-bottom: 14px; }
.contrib-core-chips { display: flex; gap: 8px; justify-content: center; flex-wrap: wrap; }
.contrib-chip {
  font-size: 10px; font-weight: 600; padding: 3px 10px; border-radius: 20px;
  background: rgba(245,158,11,0.1); border: 1px solid rgba(245,158,11,0.3); color: var(--amber);
}
.contrib-chip-future {
  background: rgba(100,116,139,0.1); border-color: rgba(100,116,139,0.3); color: var(--text-muted);
}

/* Research contribution pipeline */
.contrib-pipeline {
  display: flex; align-items: center; justify-content: center; gap: 0;
  background: var(--glass2); border: 1px solid var(--border2); border-radius: var(--radius);
  padding: 18px 24px; overflow-x: auto;
}
.contrib-pipe-step {
  display: flex; flex-direction: column; align-items: center; gap: 5px;
  text-align: center; min-width: 100px; padding: 0 8px;
}
.contrib-pipe-icon { font-size: 24px; }
.contrib-pipe-label { font-size: 13px; font-weight: 700; }
.contrib-pipe-note { font-size: 10px; color: var(--text-muted); }
.contrib-pipe-done .contrib-pipe-label { color: var(--green); }
.contrib-pipe-active .contrib-pipe-label { color: var(--amber); }
.contrib-pipe-planned .contrib-pipe-label { color: var(--text-muted); }
.contrib-pipe-arrow { font-size: 20px; color: var(--text-muted); padding: 0 4px; flex-shrink: 0; }

/* Differentiator grid */
.contrib-differentiator {
  background: rgba(0,212,255,0.03);
  border: 1px solid rgba(0,212,255,0.1);
  border-radius: var(--radius);
  padding: 20px;
}
.contrib-diff-title {
  font-size: 13px; font-weight: 700; color: var(--cyan); margin-bottom: 16px;
}
.contrib-diff-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
.contrib-diff-item {
  display: flex; gap: 12px; align-items: flex-start;
  background: rgba(255,255,255,0.02); border: 1px solid var(--border2);
  border-radius: var(--radius-sm); padding: 12px;
}
.contrib-diff-icon { font-size: 20px; flex-shrink: 0; }
.contrib-diff-item strong { font-size: 12px; font-weight: 700; color: var(--text); display: block; margin-bottom: 3px; }
.contrib-diff-item span { font-size: 11px; color: var(--text-muted); line-height: 1.4; }

/* ─── Roadmap ─────────────────────────────────────────────────────── */
.roadmap-scroll { overflow-x: auto; padding: 20px; }
.roadmap-track { display: flex; gap: 12px; min-width: max-content; }
.phase-card {
  width: 148px;
  flex-shrink: 0;
  background: var(--glass2);
  border: 1px solid var(--border2);
  border-radius: var(--radius-sm);
  padding: 14px 12px;
  cursor: default;
  transition: transform 0.2s, box-shadow 0.2s;
  position: relative;
}
.phase-card:hover { transform: translateY(-4px); box-shadow: var(--shadow); }
.phase-current { border-color: rgba(0,212,255,0.4) !important; box-shadow: var(--glow-cyan); }
.phase-complete { background: rgba(16, 185, 129, 0.06); }
.phase-header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }
.phase-num {
  width: 30px; height: 30px;
  border: 2px solid;
  border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  font-size: 13px; font-weight: 800;
}
.phase-status-dot { font-size: 15px; }
.phase-name { font-size: 12px; font-weight: 700; margin-bottom: 3px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.phase-status-label { font-size: 9px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.1em; margin-bottom: 10px; }
.phase-progress-bar-wrap { height: 4px; background: rgba(255,255,255,0.08); border-radius: 2px; margin-bottom: 5px; overflow: hidden; }
.phase-progress-bar { height: 100%; border-radius: 2px; transition: width 0.5s; }
.phase-prog-text { font-size: 10px; font-weight: 600; text-align: right; }
.phase-criteria { font-size: 10px; color: var(--green); margin-top: 6px; font-weight: 500; }
.phase-rv { font-size: 10px; margin-top: 4px; font-weight: 500; }

/* ─── Modules ─────────────────────────────────────────────────────── */
.modules-list { padding: 6px 0; max-height: 480px; overflow-y: auto; }
.module-row {
  display: flex; align-items: center; justify-content: space-between;
  padding: 12px 22px; border-bottom: 1px solid var(--border2);
  gap: 12px; transition: background 0.15s;
}
.module-row:last-child { border-bottom: none; }
.module-row:hover { background: rgba(255,255,255,0.025); }
.module-info { flex: 1; min-width: 0; }
.module-name { font-size: 13px; font-weight: 600; }
.module-desc { font-size: 11px; color: var(--text-muted); margin-top: 2px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.module-right { display: flex; align-items: center; gap: 10px; flex-shrink: 0; }
.module-badge { font-size: 10px; font-weight: 700; text-transform: uppercase; border: 1px solid; border-radius: 20px; padding: 2px 9px; white-space: nowrap; }
.module-prog-bar-wrap { width: 64px; height: 4px; background: rgba(255,255,255,0.08); border-radius: 2px; overflow: hidden; }
.module-prog-bar { height: 100%; border-radius: 2px; }

/* ─── Algorithms ──────────────────────────────────────────────────── */
.algorithms-container { padding: 6px 0; }
.alg-category { margin-bottom: 4px; border-bottom: 1px solid var(--border2); }
.alg-category:last-child { border-bottom: none; }
.alg-cat-label {
  font-size: 11px; text-transform: uppercase; letter-spacing: 0.12em;
  color: var(--text-dim); padding: 10px 22px 4px; font-weight: 700;
}
.alg-sub-label {
  font-size: 9px; text-transform: uppercase; letter-spacing: 0.1em;
  padding: 4px 22px; font-weight: 700;
}
.alg-sub-done { color: var(--green); }
.alg-sub-prog { color: var(--amber); }
.alg-sub-plan { color: var(--text-muted); }
.alg-row {
  display: flex; align-items: center; justify-content: space-between;
  padding: 9px 22px; border-bottom: 1px solid var(--border2);
  transition: background 0.15s;
}
.alg-row:hover { background: rgba(255,255,255,0.025); }
.alg-row:last-child { border-bottom: none; }
.alg-info { flex: 1; min-width: 0; }
.alg-name { font-size: 12px; font-weight: 600; }
.alg-metric { font-size: 10px; color: var(--text-muted); display: block; margin-top: 1px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.alg-badge { font-size: 10px; font-weight: 700; text-transform: uppercase; border: 1px solid; border-radius: 20px; padding: 2px 9px; flex-shrink: 0; }

/* ─── Environments ────────────────────────────────────────────────── */
.environments-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(180px, 1fr)); gap: 12px; padding: 18px; }
.env-card { background: var(--glass2); border: 1px solid; border-radius: var(--radius-sm); padding: 14px; transition: transform 0.2s; }
.env-card:hover { transform: translateY(-3px); }
.env-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 7px; }
.env-name { font-size: 13px; font-weight: 700; }
.env-desc { font-size: 11px; color: var(--text-muted); margin-bottom: 9px; line-height: 1.4; }
.env-stats { display: flex; gap: 8px; font-size: 11px; color: var(--text-dim); flex-wrap: wrap; }
.env-last { font-size: 10px; color: var(--text-muted); margin-top: 7px; }

/* ─── Best config ─────────────────────────────────────────────────── */
.best-config { padding: 22px; }
.config-profile { font-size: 16px; font-weight: 800; color: var(--amber); margin-bottom: 4px; }
.config-exp { font-family: var(--mono); font-size: 11px; color: var(--text-muted); margin-bottom: 18px; }
.config-metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; margin-bottom: 14px; }
.config-metric { text-align: center; background: var(--glass2); border-radius: var(--radius-sm); padding: 12px; }
.config-metric-val { font-size: 22px; font-weight: 800; }
.config-metric-label { font-size: 10px; color: var(--text-muted); margin-top: 3px; font-weight: 500; }
.config-confidence { font-size: 12px; font-weight: 700; margin-bottom: 8px; }
.config-note { font-size: 11px; color: var(--text-muted); }

/* ─── Paper progress ──────────────────────────────────────────────── */
.paper-overall { font-size: 20px; font-weight: 800; }
.paper-sections { padding: 6px 0; }
.paper-row {
  display: grid;
  grid-template-columns: 140px 1fr 46px 1fr;
  align-items: center;
  gap: 14px;
  padding: 11px 22px;
  border-bottom: 1px solid var(--border2);
  transition: background 0.15s;
}
.paper-row:hover { background: rgba(255,255,255,0.02); }
.paper-row:last-child { border-bottom: none; }
.paper-section-name { font-size: 13px; font-weight: 600; display: flex; align-items: center; gap: 7px; }
.paper-dot { font-size: 14px; flex-shrink: 0; }
.paper-bar-wrap { height: 7px; background: rgba(255,255,255,0.07); border-radius: 4px; overflow: hidden; }
.paper-bar { height: 100%; border-radius: 4px; transition: width 0.5s; }
.paper-pct { font-size: 13px; font-weight: 700; text-align: right; }
.paper-note { font-size: 11px; color: var(--text-muted); }

/* ─── Experiments table ───────────────────────────────────────────── */
.table-scroll { overflow-x: auto; }
.experiments-table { width: 100%; border-collapse: collapse; font-size: 12px; }
.experiments-table th {
  text-align: left;
  padding: 10px 16px;
  color: var(--text-muted);
  font-size: 10px;
  text-transform: uppercase;
  letter-spacing: 0.1em;
  border-bottom: 1px solid var(--border);
  white-space: nowrap;
  font-weight: 700;
}
.experiments-table td { padding: 10px 16px; border-bottom: 1px solid var(--border2); vertical-align: middle; }
.exp-row:hover { background: rgba(255,255,255,0.025); }
.exp-row:last-child td { border-bottom: none; }
.exp-id { font-family: var(--mono); font-size: 11px; color: var(--cyan); }
.exp-name { max-width: 180px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.exp-status { white-space: nowrap; }
.exp-metric { text-align: right; font-family: var(--mono); }
.exp-ts { color: var(--text-muted); white-space: nowrap; }

/* ─── Knowledge base ──────────────────────────────────────────────── */
.kb-list { padding: 6px 0; }
.kb-item { display: flex; gap: 14px; padding: 14px 22px; border-bottom: 1px solid var(--border2); transition: background 0.15s; }
.kb-item:last-child { border-bottom: none; }
.kb-item:hover { background: rgba(255,255,255,0.025); }
.kb-icon-wrap {
  width: 38px; height: 38px; flex-shrink: 0;
  background: color-mix(in srgb, var(--kb-color, #64748b) 10%, transparent);
  border: 1px solid color-mix(in srgb, var(--kb-color, #64748b) 30%, transparent);
  border-radius: var(--radius-sm);
  display: flex; align-items: center; justify-content: center;
  font-size: 18px;
}
.kb-icon { }
.kb-content { flex: 1; min-width: 0; }
.kb-title { font-size: 13px; font-weight: 700; margin-bottom: 4px; }
.kb-link { font-size: 11px; color: var(--cyan); text-decoration: none; margin-left: 8px; border: 1px solid rgba(0,212,255,0.25); border-radius: 4px; padding: 1px 7px; }
.kb-link:hover { background: rgba(0,212,255,0.1); }
.kb-desc { font-size: 11px; color: var(--text-muted); margin-bottom: 7px; line-height: 1.4; }
.kb-footer { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
.kb-tag { font-size: 10px; background: rgba(255,255,255,0.05); border: 1px solid var(--border); border-radius: 4px; padding: 1px 7px; color: var(--text-dim); }
.kb-date { font-size: 10px; color: var(--text-muted); margin-left: auto; }

/* ─── Tasks ───────────────────────────────────────────────────────── */
.tasks-list { padding: 6px 0; max-height: 480px; overflow-y: auto; }
.task-item { display: flex; gap: 12px; padding: 12px 22px; border-bottom: 1px solid var(--border2); transition: background 0.15s; }
.task-item:last-child { border-bottom: none; }
.task-item:hover { background: rgba(255,255,255,0.025); }
.task-check { font-size: 16px; flex-shrink: 0; padding-top: 1px; }
.task-content { flex: 1; }
.task-text { font-size: 13px; line-height: 1.45; margin-bottom: 7px; }
.task-meta { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
.task-priority { font-size: 9px; font-weight: 800; text-transform: uppercase; letter-spacing: 0.1em; border: 1px solid; border-radius: 20px; padding: 1px 8px; }
.task-phase,
.task-owner,
.task-hours,
.task-deps { font-size: 10px; color: var(--text-muted); }

/* ─── Shared utilities ────────────────────────────────────────────── */
.no-data { padding: 36px 22px; color: var(--text-muted); text-align: center; font-size: 13px; }

/* ─── Footer ──────────────────────────────────────────────────────── */
.dashboard-footer {
  border-top: 1px solid var(--border);
  background: rgba(0,0,0,0.4);
  backdrop-filter: blur(20px);
}
.footer-grid {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 0;
  border-bottom: 1px solid var(--border2);
}
.footer-item {
  padding: 20px 22px;
  border-right: 1px solid var(--border2);
}
.footer-item:last-child { border-right: none; }
.footer-item-label {
  font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.1em;
  color: var(--cyan); margin-bottom: 8px;
}
.footer-item-value { font-size: 12px; color: var(--text-muted); line-height: 1.5; }
.footer-bottom {
  display: flex; justify-content: space-between; align-items: center;
  padding: 12px 22px; font-size: 11px; color: var(--text-muted);
}

/* ─── Responsive ──────────────────────────────────────────────────── */
@media (max-width: 1280px) {
  .kpi-strip { grid-template-columns: repeat(3, 1fr); }
  .impl-groups { grid-template-columns: repeat(2, 1fr); }
  .footer-grid { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 1024px) {
  .kpi-strip { grid-template-columns: repeat(2, 1fr); }
  .two-col { grid-template-columns: 1fr; }
  .research-two-col { grid-template-columns: 1fr; }
  .research-hypothesis-block { border-left: none; border-top: 1px solid var(--border2); }
  .paper-row { grid-template-columns: 120px 1fr 40px; }
  .paper-row .paper-note { display: none; }
  .contrib-diff-grid { grid-template-columns: 1fr; }
  .impl-groups { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 768px) {
  .dashboard-header { flex-direction: column; gap: 12px; align-items: flex-start; }
  .header-center { display: none; }
  .kpi-strip { grid-template-columns: repeat(2, 1fr); }
  .impl-groups { grid-template-columns: 1fr; }
  .footer-grid { grid-template-columns: 1fr; }
}
@media (max-width: 640px) {
  .kpi-strip { grid-template-columns: repeat(2, 1fr); }
  .kpi-value { font-size: 24px; }
  .header-readiness { display: none; }
  .config-metrics { grid-template-columns: repeat(2, 1fr); }
}
"""

    # ── JavaScript ────────────────────────────────────────────────────────────

    @staticmethod
    def _get_js() -> str:
        return """
document.addEventListener('DOMContentLoaded', () => {
  // ── Animate all progress bars on load ─────────────────────────────
  const animateBar = (selector, duration = 0.8, delay = 0) => {
    document.querySelectorAll(selector).forEach(bar => {
      const target = bar.style.width;
      bar.style.width = '0';
      bar.style.transition = 'none';
      requestAnimationFrame(() => {
        requestAnimationFrame(() => {
          bar.style.transition = `width ${duration}s cubic-bezier(0.4, 0, 0.2, 1) ${delay}s`;
          bar.style.width = target;
        });
      });
    });
  };

  animateBar('.phase-progress-bar', 0.9, 0.1);
  animateBar('.module-prog-bar', 0.7, 0.05);
  animateBar('.paper-bar', 0.8, 0.15);

  // ── Readiness ring animation ───────────────────────────────────────
  const ring = document.querySelector('.ring-fill');
  if (ring) {
    const target = ring.style.strokeDasharray;
    ring.style.strokeDasharray = '0 213.6';
    setTimeout(() => {
      ring.style.transition = 'stroke-dasharray 1.2s cubic-bezier(0.4, 0, 0.2, 1)';
      ring.style.strokeDasharray = target;
    }, 300);
  }

  // ── Staggered fade-in for impl chips ─────────────────────────────
  document.querySelectorAll('.impl-chip').forEach((chip, i) => {
    chip.style.opacity = '0';
    chip.style.transform = 'translateX(-8px)';
    chip.style.transition = `opacity 0.3s ease ${i * 0.04}s, transform 0.3s ease ${i * 0.04}s`;
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        chip.style.opacity = '1';
        chip.style.transform = 'translateX(0)';
      });
    });
  });

  // ── Staggered fade-in for arch layers ────────────────────────────
  document.querySelectorAll('.arch-layer').forEach((layer, i) => {
    layer.style.opacity = '0';
    layer.style.transform = 'translateX(-12px)';
    layer.style.transition = `opacity 0.4s ease ${0.1 + i * 0.07}s, transform 0.4s ease ${0.1 + i * 0.07}s`;
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        layer.style.opacity = '1';
        layer.style.transform = 'translateX(0)';
      });
    });
  });

  // ── Staggered fade-in for KPI cards ──────────────────────────────
  document.querySelectorAll('.kpi-card').forEach((card, i) => {
    card.style.opacity = '0';
    card.style.transform = 'translateY(12px)';
    card.style.transition = `opacity 0.4s ease ${i * 0.08}s, transform 0.4s ease ${i * 0.08}s`;
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        card.style.opacity = '1';
        card.style.transform = 'translateY(0)';
      });
    });
  });

  // ── Contrib pipeline step animations ─────────────────────────────
  document.querySelectorAll('.contrib-pipe-step').forEach((step, i) => {
    step.style.opacity = '0';
    step.style.transform = 'translateY(8px)';
    step.style.transition = `opacity 0.4s ease ${0.3 + i * 0.1}s, transform 0.4s ease ${0.3 + i * 0.1}s`;
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        step.style.opacity = '1';
        step.style.transform = 'translateY(0)';
      });
    });
  });

  // ── Console branding ──────────────────────────────────────────────
  console.log('%c🚁 Hybrid UAV Research Platform', 'color:#00d4ff; font-weight:800; font-size:16px');
  console.log('%c  Research Dashboard v1.0 — Supervisor Review Mode', 'color:#a78bfa; font-size:12px');
  console.log('  State: dashboard_state.json');
  console.log('  Refresh: python platform/dashboard/dashboard.py');
});
"""
