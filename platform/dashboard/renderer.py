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
            self._render_footer(generated_display),
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
      </div>
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
        <div class="readiness-value">{readiness}%</div>
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

        kpis = [
            ("Phases Complete",  f"{complete_phases}/{total_phases}", "🎯", "#10b981"),
            ("Open Questions",   str(open_q),                         "❓", "#00d4ff"),
            ("Active Hypotheses",str(active_h),                       "🔬", "#a78bfa"),
            ("Paper Progress",   f"{paper_pct}%",                     "📄", "#f59e0b"),
        ]

        cards = ""
        for label, value, icon, color in kpis:
            cards += f"""
    <div class="kpi-card">
      <div class="kpi-icon" style="color:{color}">{icon}</div>
      <div class="kpi-value" style="color:{color}">{value}</div>
      <div class="kpi-label">{label}</div>
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
      <div class="phase-num" style="border-color:{color}; color:{color}">{pid}</div>
      <div class="phase-status-dot" style="color:{color}">{dot}</div>
    </div>
    <div class="phase-name">{self._esc(phase.get('short_name', phase.get('name','')))}</div>
    <div class="phase-status-label" style="color:{color}">{label}</div>
    <div class="phase-progress-bar-wrap">
      <div class="phase-progress-bar" style="width:{prog}%; background:{color}"></div>
    </div>
    <div class="phase-prog-text">{prog}%</div>
    {f'<div class="phase-criteria">✅ {crit_text} criteria</div>' if total_criteria else ""}
    {f'<div class="phase-rv" style="color:{rv_color}">★ {phase.get("research_value","")}</div>' if phase.get("research_value") else ""}
  </div>"""

        return f"""
<section class="card roadmap-card">
  <div class="card-header">
    <h2 class="card-title">🗺 Development Roadmap</h2>
    <div class="card-subtitle">Phases complete when ALL exit criteria are met</div>
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
            rows += f"""
    <div class="module-row">
      <div class="module-info">
        <div class="module-name">{self._esc(mod.get("name",""))}</div>
        <div class="module-desc">{self._esc(mod.get("description",""))}</div>
      </div>
      <div class="module-right">
        <div class="module-badge" style="color:{color}; border-color:{color}">{label}</div>
        <div class="module-prog-bar-wrap">
          <div class="module-prog-bar" style="width:{prog}%; background:{color}"></div>
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
            return self._empty_card("⚙ Registered Algorithms", "No algorithms registered.")

        categories = ["planner", "avoidance", "hybrid"]
        cat_labels = {"planner": "Path Planners", "avoidance": "Collision Avoidance", "hybrid": "Hybrid Fusion"}
        sections_html = ""

        for cat in categories:
            algs = [a for a in algorithms if a.get("category") == cat]
            if not algs:
                continue
            icon = self.CATEGORY_ICONS.get(cat, "⚙")
            rows = ""
            for alg in algs:
                status = alg.get("status", "planned")
                label, color = self.ALGORITHM_STATUS_CONFIG.get(status, ("Planned", "#64748b"))
                rows += f"""
      <div class="alg-row">
        <div class="alg-info">
          <span class="alg-name">{self._esc(alg.get("name",""))}</span>
          {f'<span class="alg-metric">{self._esc(alg.get("best_metric",""))}</span>' if alg.get("best_metric") else ""}
        </div>
        <div class="alg-badge" style="color:{color}; border-color:{color}">{label}</div>
      </div>"""
            sections_html += f"""
    <div class="alg-category">
      <div class="alg-cat-label">{icon} {cat_labels.get(cat, cat.title())}</div>
      {rows}
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
    <div class="env-card" style="border-color:{color}20">
      <div class="env-header">
        <div class="env-name">{self._esc(env.get("name",""))}</div>
        <div class="env-badge" style="color:{color}">{label}</div>
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
        if not sections:
            return ""

        rows = ""
        for s in sections:
            prog = s.get("progress", 0)
            color = "#10b981" if prog >= 80 else "#00d4ff" if prog >= 40 else "#f59e0b" if prog >= 10 else "#64748b"
            note = s.get("status_note", "")
            rows += f"""
    <div class="paper-row">
      <div class="paper-section-name">{self._esc(s.get("name",""))}</div>
      <div class="paper-bar-wrap">
        <div class="paper-bar" style="width:{prog}%; background:{color}"></div>
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
      <td class="exp-status"><span style="color:{color}">● {label}</span></td>
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
      <div class="kb-icon">{icon}</div>
      <div class="kb-content">
        <div class="kb-title">{self._esc(title)} {link_html}</div>
        {f'<div class="kb-desc">{self._esc(desc)}</div>' if desc else ""}
        <div class="kb-footer">{tag_html}{f'<span class="kb-date">{date}</span>' if date else ""}</div>
      </div>
    </div>"""

        return f"""
<div class="card">
  <div class="card-header"><h2 class="card-title">📚 Knowledge Base</h2></div>
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
      <div class="task-check">○</div>
      <div class="task-content">
        <div class="task-text">{self._esc(task.get("text",""))}</div>
        <div class="task-meta">
          <span class="task-priority" style="color:{p_color};border-color:{p_color}">{p_label}</span>
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

    def _render_footer(self, generated: str) -> str:
        return f"""
<footer class="dashboard-footer">
  <div class="footer-left">
    Hybrid Multi-UAV Navigation Research Platform · Platform v1.0.0
  </div>
  <div class="footer-right">
    Generated {generated} · dashboard_state.json
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
        return f'<span class="status-badge" style="color:{color};border-color:{color}">{label}</span>'

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
  --glass:       rgba(14, 22, 41, 0.85);
  --glass2:      rgba(20, 32, 60, 0.6);
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
  --font:        system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif;
  --mono:        'Cascadia Code', 'Fira Code', 'Consolas', monospace;
  --radius:      12px;
  --radius-sm:   8px;
  --shadow:      0 4px 24px rgba(0,0,0,0.4);
  --shadow-lg:   0 8px 48px rgba(0,0,0,0.6);
  --glow-cyan:   0 0 20px rgba(0, 212, 255, 0.15);
}

html { scroll-behavior: smooth; }

body {
  font-family: var(--font);
  background: var(--bg);
  color: var(--text);
  font-size: 14px;
  line-height: 1.5;
  min-height: 100vh;
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
  padding: 20px 28px;
  background: linear-gradient(135deg, rgba(0,212,255,0.05), rgba(124,58,237,0.05));
  border-bottom: 1px solid var(--border);
  backdrop-filter: blur(20px);
  position: sticky;
  top: 0;
  z-index: 100;
}
.header-brand { display: flex; align-items: center; gap: 16px; }
.header-icon { font-size: 36px; filter: drop-shadow(0 0 8px rgba(0,212,255,0.5)); }
.header-title {
  font-size: 20px;
  font-weight: 700;
  background: linear-gradient(135deg, #fff 30%, var(--cyan));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  letter-spacing: -0.3px;
}
.header-meta { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 4px; }
.meta-chip {
  font-size: 11px;
  color: var(--text-dim);
  background: rgba(255,255,255,0.04);
  border: 1px solid var(--border);
  border-radius: 20px;
  padding: 2px 10px;
}
.branch-chip { color: var(--cyan); border-color: rgba(0,212,255,0.25); }

/* Readiness ring */
.header-readiness { flex-shrink: 0; }
.readiness-ring { position: relative; width: 80px; height: 80px; }
.readiness-svg { width: 80px; height: 80px; transform: rotate(-90deg); }
.ring-bg  { fill: none; stroke: rgba(255,255,255,0.07); stroke-width: 6; }
.ring-fill { fill: none; stroke-width: 6; stroke-linecap: round; transition: stroke-dasharray 0.6s; }
.readiness-inner {
  position: absolute; inset: 0;
  display: flex; flex-direction: column; align-items: center; justify-content: center;
}
.readiness-value { font-size: 18px; font-weight: 700; }
.readiness-label { font-size: 9px; color: var(--text-muted); margin-top: 1px; }

/* ─── Main layout ─────────────────────────────────────────────────── */
.dashboard-main {
  max-width: 1600px;
  margin: 0 auto;
  padding: 24px 24px 48px;
  display: flex;
  flex-direction: column;
  gap: 20px;
}

/* ─── KPI strip ───────────────────────────────────────────────────── */
.kpi-strip {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 16px;
}
.kpi-card {
  background: var(--glass);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 20px;
  text-align: center;
  backdrop-filter: blur(16px);
  transition: transform 0.2s, box-shadow 0.2s;
}
.kpi-card:hover { transform: translateY(-2px); box-shadow: var(--glow-cyan); }
.kpi-icon  { font-size: 24px; margin-bottom: 8px; }
.kpi-value { font-size: 28px; font-weight: 700; letter-spacing: -0.5px; }
.kpi-label { font-size: 12px; color: var(--text-muted); margin-top: 4px; }

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
  align-items: baseline;
  justify-content: space-between;
  padding: 16px 20px 12px;
  border-bottom: 1px solid var(--border2);
}
.card-title { font-size: 15px; font-weight: 600; letter-spacing: -0.2px; }
.card-subtitle { font-size: 11px; color: var(--text-muted); }

/* ─── Two-column layout ───────────────────────────────────────────── */
.two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }

/* ─── Research card ───────────────────────────────────────────────── */
.research-card { border-color: rgba(0, 212, 255, 0.2); }
.research-two-col { display: grid; grid-template-columns: 1fr 1fr; gap: 0; }
.research-question-block,
.research-hypothesis-block { padding: 20px; }
.research-hypothesis-block { border-left: 1px solid var(--border2); }
.research-block-label { font-size: 10px; text-transform: uppercase; letter-spacing: 0.1em; color: var(--text-muted); margin-bottom: 10px; }
.research-q-id { font-family: var(--mono); font-size: 11px; color: var(--cyan); margin-bottom: 8px; }
.research-q-text { font-size: 14px; font-style: italic; color: var(--text); line-height: 1.6; margin-bottom: 12px; }
.research-q-meta { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.meta-since { font-size: 11px; color: var(--text-muted); }
.hyp-id { font-family: var(--mono); font-size: 11px; color: var(--violet); margin-bottom: 8px; }
.hyp-text { font-size: 13px; color: var(--text-dim); line-height: 1.6; margin-bottom: 8px; }
.hyp-prediction { font-size: 12px; color: var(--text-muted); margin-bottom: 10px; }
.hyp-meta { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.status-badge {
  font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.08em;
  border: 1px solid; border-radius: 20px; padding: 2px 10px;
}
.exp-chips { display: flex; gap: 6px; }
.exp-chip { font-family: var(--mono); font-size: 10px; color: var(--cyan); background: rgba(0,212,255,0.06); border: 1px solid rgba(0,212,255,0.15); border-radius: 4px; padding: 2px 6px; }

/* ─── Roadmap ─────────────────────────────────────────────────────── */
.roadmap-scroll { overflow-x: auto; padding: 20px; }
.roadmap-track { display: flex; gap: 12px; min-width: max-content; }
.phase-card {
  width: 140px;
  flex-shrink: 0;
  background: var(--glass2);
  border: 1px solid var(--border2);
  border-radius: var(--radius-sm);
  padding: 14px;
  cursor: default;
  transition: transform 0.2s, box-shadow 0.2s;
  position: relative;
}
.phase-card:hover { transform: translateY(-3px); box-shadow: var(--shadow); }
.phase-current { border-color: rgba(0,212,255,0.35) !important; box-shadow: var(--glow-cyan); }
.phase-complete { background: rgba(16, 185, 129, 0.06); }
.phase-header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 8px; }
.phase-num {
  width: 28px; height: 28px;
  border: 2px solid;
  border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  font-size: 12px; font-weight: 700;
}
.phase-status-dot { font-size: 14px; }
.phase-name { font-size: 12px; font-weight: 600; margin-bottom: 3px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.phase-status-label { font-size: 10px; font-weight: 600; text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 8px; }
.phase-progress-bar-wrap { height: 3px; background: rgba(255,255,255,0.08); border-radius: 2px; margin-bottom: 4px; overflow: hidden; }
.phase-progress-bar { height: 100%; border-radius: 2px; transition: width 0.5s; }
.phase-prog-text { font-size: 10px; color: var(--text-muted); text-align: right; }
.phase-criteria { font-size: 10px; color: var(--green); margin-top: 5px; }
.phase-rv { font-size: 10px; margin-top: 4px; }

/* ─── Modules ─────────────────────────────────────────────────────── */
.modules-list { padding: 8px 0; max-height: 480px; overflow-y: auto; }
.module-row {
  display: flex; align-items: center; justify-content: space-between;
  padding: 12px 20px; border-bottom: 1px solid var(--border2);
  gap: 12px;
}
.module-row:last-child { border-bottom: none; }
.module-row:hover { background: rgba(255,255,255,0.02); }
.module-info { flex: 1; min-width: 0; }
.module-name { font-size: 13px; font-weight: 600; }
.module-desc { font-size: 11px; color: var(--text-muted); margin-top: 2px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.module-right { display: flex; align-items: center; gap: 10px; flex-shrink: 0; }
.module-badge { font-size: 10px; font-weight: 600; text-transform: uppercase; border: 1px solid; border-radius: 20px; padding: 2px 8px; white-space: nowrap; }
.module-prog-bar-wrap { width: 60px; height: 4px; background: rgba(255,255,255,0.08); border-radius: 2px; overflow: hidden; }
.module-prog-bar { height: 100%; border-radius: 2px; }

/* ─── Algorithms ──────────────────────────────────────────────────── */
.algorithms-container { padding: 8px 0; }
.alg-category { margin-bottom: 4px; }
.alg-cat-label { font-size: 10px; text-transform: uppercase; letter-spacing: 0.1em; color: var(--text-muted); padding: 8px 20px 4px; }
.alg-row {
  display: flex; align-items: center; justify-content: space-between;
  padding: 9px 20px; border-bottom: 1px solid var(--border2);
}
.alg-row:hover { background: rgba(255,255,255,0.02); }
.alg-info { flex: 1; min-width: 0; }
.alg-name { font-size: 12px; font-weight: 500; }
.alg-metric { font-size: 10px; color: var(--text-muted); display: block; margin-top: 1px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.alg-badge { font-size: 10px; font-weight: 600; text-transform: uppercase; border: 1px solid; border-radius: 20px; padding: 2px 8px; flex-shrink: 0; }

/* ─── Environments ────────────────────────────────────────────────── */
.environments-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(180px, 1fr)); gap: 12px; padding: 16px; }
.env-card { background: var(--glass2); border: 1px solid; border-radius: var(--radius-sm); padding: 14px; transition: transform 0.2s; }
.env-card:hover { transform: translateY(-2px); }
.env-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
.env-name { font-size: 13px; font-weight: 600; }
.env-badge { font-size: 10px; font-weight: 600; }
.env-desc { font-size: 11px; color: var(--text-muted); margin-bottom: 8px; line-height: 1.4; }
.env-stats { display: flex; gap: 8px; font-size: 11px; color: var(--text-dim); }
.env-last { font-size: 10px; color: var(--text-muted); margin-top: 6px; }

/* ─── Best config ─────────────────────────────────────────────────── */
.best-config { padding: 20px; }
.config-profile { font-size: 16px; font-weight: 700; color: var(--amber); margin-bottom: 4px; }
.config-exp { font-family: var(--mono); font-size: 11px; color: var(--text-muted); margin-bottom: 16px; }
.config-metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; margin-bottom: 12px; }
.config-metric { text-align: center; background: var(--glass2); border-radius: var(--radius-sm); padding: 10px; }
.config-metric-val { font-size: 20px; font-weight: 700; }
.config-metric-label { font-size: 10px; color: var(--text-muted); margin-top: 2px; }
.config-confidence { font-size: 12px; font-weight: 600; margin-bottom: 8px; }
.config-note { font-size: 11px; color: var(--text-muted); }

/* ─── Paper progress ──────────────────────────────────────────────── */
.paper-card .card-header { }
.paper-overall { font-size: 20px; font-weight: 700; }
.paper-sections { padding: 8px 0; }
.paper-row {
  display: grid;
  grid-template-columns: 130px 1fr 40px 1fr;
  align-items: center;
  gap: 12px;
  padding: 10px 20px;
  border-bottom: 1px solid var(--border2);
}
.paper-row:last-child { border-bottom: none; }
.paper-section-name { font-size: 13px; font-weight: 500; }
.paper-bar-wrap { height: 6px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden; }
.paper-bar { height: 100%; border-radius: 3px; transition: width 0.5s; }
.paper-pct { font-size: 13px; font-weight: 600; text-align: right; }
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
  letter-spacing: 0.08em;
  border-bottom: 1px solid var(--border);
  white-space: nowrap;
}
.experiments-table td { padding: 10px 16px; border-bottom: 1px solid var(--border2); vertical-align: middle; }
.exp-row:hover { background: rgba(255,255,255,0.02); }
.exp-row:last-child td { border-bottom: none; }
.exp-id { font-family: var(--mono); font-size: 11px; color: var(--cyan); }
.exp-name { max-width: 180px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.exp-status { white-space: nowrap; }
.exp-metric { text-align: right; font-family: var(--mono); }
.exp-ts { color: var(--text-muted); white-space: nowrap; }

/* ─── Knowledge base ──────────────────────────────────────────────── */
.kb-list { padding: 8px 0; }
.kb-item { display: flex; gap: 12px; padding: 12px 20px; border-bottom: 1px solid var(--border2); }
.kb-item:last-child { border-bottom: none; }
.kb-item:hover { background: rgba(255,255,255,0.02); }
.kb-icon { font-size: 20px; flex-shrink: 0; padding-top: 2px; }
.kb-content { flex: 1; min-width: 0; }
.kb-title { font-size: 13px; font-weight: 600; margin-bottom: 4px; }
.kb-link { font-size: 11px; color: var(--cyan); text-decoration: none; margin-left: 6px; }
.kb-link:hover { text-decoration: underline; }
.kb-desc { font-size: 11px; color: var(--text-muted); margin-bottom: 6px; line-height: 1.4; }
.kb-footer { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
.kb-tag { font-size: 10px; background: rgba(255,255,255,0.05); border: 1px solid var(--border); border-radius: 4px; padding: 1px 6px; color: var(--text-dim); }
.kb-date { font-size: 10px; color: var(--text-muted); margin-left: auto; }

/* ─── Tasks ───────────────────────────────────────────────────────── */
.tasks-list { padding: 8px 0; max-height: 480px; overflow-y: auto; }
.task-item { display: flex; gap: 12px; padding: 12px 20px; border-bottom: 1px solid var(--border2); }
.task-item:last-child { border-bottom: none; }
.task-item:hover { background: rgba(255,255,255,0.02); }
.task-check { font-size: 16px; color: var(--text-muted); flex-shrink: 0; padding-top: 1px; }
.task-content { flex: 1; }
.task-text { font-size: 13px; line-height: 1.4; margin-bottom: 6px; }
.task-meta { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
.task-priority { font-size: 9px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.1em; border: 1px solid; border-radius: 20px; padding: 1px 7px; }
.task-phase,
.task-owner,
.task-hours,
.task-deps { font-size: 10px; color: var(--text-muted); }

/* ─── Shared utilities ────────────────────────────────────────────── */
.no-data { padding: 32px 20px; color: var(--text-muted); text-align: center; font-size: 13px; }

/* ─── Footer ──────────────────────────────────────────────────────── */
.dashboard-footer {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 28px;
  border-top: 1px solid var(--border);
  font-size: 11px;
  color: var(--text-muted);
  background: rgba(0,0,0,0.3);
}

/* ─── Responsive ──────────────────────────────────────────────────── */
@media (max-width: 1024px) {
  .kpi-strip { grid-template-columns: repeat(2, 1fr); }
  .two-col { grid-template-columns: 1fr; }
  .research-two-col { grid-template-columns: 1fr; }
  .research-hypothesis-block { border-left: none; border-top: 1px solid var(--border2); }
  .paper-row { grid-template-columns: 110px 1fr 36px; }
  .paper-row .paper-note { display: none; }
}

@media (max-width: 640px) {
  .dashboard-header { flex-direction: column; gap: 12px; align-items: flex-start; }
  .kpi-strip { grid-template-columns: repeat(2, 1fr); }
  .kpi-value { font-size: 22px; }
  .header-readiness { display: none; }
  .config-metrics { grid-template-columns: repeat(2, 1fr); }
}
"""

    # ── JavaScript ────────────────────────────────────────────────────────────

    @staticmethod
    def _get_js() -> str:
        return """
// Animate progress bars on load
document.addEventListener('DOMContentLoaded', () => {
  // Animate phase progress bars
  document.querySelectorAll('.phase-progress-bar').forEach(bar => {
    const target = bar.style.width;
    bar.style.width = '0';
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        bar.style.transition = 'width 0.8s cubic-bezier(0.4, 0, 0.2, 1)';
        bar.style.width = target;
      });
    });
  });

  // Animate module progress bars
  document.querySelectorAll('.module-prog-bar').forEach(bar => {
    const target = bar.style.width;
    bar.style.width = '0';
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        bar.style.transition = 'width 0.6s ease-out';
        bar.style.width = target;
      });
    });
  });

  // Animate paper section bars
  document.querySelectorAll('.paper-bar').forEach(bar => {
    const target = bar.style.width;
    bar.style.width = '0';
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        bar.style.transition = 'width 0.7s ease-out';
        bar.style.width = target;
      });
    });
  });

  // Animate readiness ring
  const ring = document.querySelector('.ring-fill');
  if (ring) {
    const target = ring.style.strokeDasharray;
    ring.style.strokeDasharray = '0 213.6';
    setTimeout(() => {
      ring.style.transition = 'stroke-dasharray 1s ease-out';
      ring.style.strokeDasharray = target;
    }, 200);
  }

  // Add generated timestamp
  console.log('%c🚁 Research Dashboard loaded', 'color:#00d4ff; font-weight:bold; font-size:14px');
  console.log('State file: dashboard_state.json');
  console.log('To update: python platform/dashboard/dashboard.py');
});
"""
