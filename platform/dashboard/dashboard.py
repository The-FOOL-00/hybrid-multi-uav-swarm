"""
# -*- coding: utf-8 -*-
platform/dashboard/dashboard.py — Research Dashboard CLI

The single entry point for the Research Dashboard.

Usage
-----
  # Render and open in the default browser (most common):
  python platform/dashboard/dashboard.py

  # Render only, do not open browser:
  python platform/dashboard/dashboard.py --no-browser

  # Use a custom state file:
  python platform/dashboard/dashboard.py --state path/to/custom_state.json

  # Write HTML to a custom output path:
  python platform/dashboard/dashboard.py --output my_dashboard.html

  # Print dashboard summary to terminal without rendering HTML:
  python platform/dashboard/dashboard.py --summary

  # Print current project status to terminal:
  python platform/dashboard/dashboard.py --status

Workflow
--------
  1. Reads dashboard_state.json (or --state file)
  2. Passes state to DashboardRenderer.render()
  3. Writes the resulting HTML to dashboard.html (or --output path)
  4. Opens the HTML file in the system's default browser

No web server is required. The dashboard is a static HTML file.

Integration with other platform modules
----------------------------------------
  When other platform modules (Experiment Manager, Metrics Engine, etc.)
  want to update the dashboard, they use DashboardWriter:

    from platform.dashboard.writer import DashboardWriter
    writer = DashboardWriter()
    writer.add_experiment({...})  # auto-saves, auto-updates dashboard

  Then re-running this script regenerates the HTML with the new state.

Exit codes
----------
  0  — Success
  1  — State file not found or unreadable
  2  — Render error
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import webbrowser
from datetime import datetime
from pathlib import Path

# ── Path resolution (works when run from any working directory) ────────────────
_THIS_FILE = Path(__file__).resolve()
_DASHBOARD_DIR = _THIS_FILE.parent
_REPO_ROOT = _DASHBOARD_DIR.parents[1]   # platform/ → repo root

# Default file paths (relative to this module)
DEFAULT_STATE_FILE  = _DASHBOARD_DIR / "dashboard_state.json"
DEFAULT_OUTPUT_FILE = _DASHBOARD_DIR / "dashboard.html"

# Ensure platform/ is importable regardless of cwd
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


# ── Lazy imports (with helpful error messages) ─────────────────────────────────

def _import_renderer():
    try:
        from platform.dashboard.renderer import DashboardRenderer
        return DashboardRenderer
    except ImportError as exc:
        _die(f"Could not import DashboardRenderer: {exc}\n"
             f"Make sure you are running from the repository root:\n"
             f"  python platform/dashboard/dashboard.py", code=2)


def _import_writer():
    try:
        from platform.dashboard.writer import DashboardWriter
        return DashboardWriter
    except ImportError as exc:
        _die(f"Could not import DashboardWriter: {exc}", code=2)


# ── CLI definition ─────────────────────────────────────────────────────────────

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="dashboard",
        description="Hybrid Multi-UAV Navigation Research Platform — Research Dashboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python platform/dashboard/dashboard.py
  python platform/dashboard/dashboard.py --no-browser
  python platform/dashboard/dashboard.py --status
  python platform/dashboard/dashboard.py --state custom_state.json --output out.html
        """
    )

    parser.add_argument(
        "--state",
        metavar="PATH",
        default=str(DEFAULT_STATE_FILE),
        help=f"Path to dashboard_state.json (default: {DEFAULT_STATE_FILE})",
    )
    parser.add_argument(
        "--output",
        metavar="PATH",
        default=str(DEFAULT_OUTPUT_FILE),
        help=f"Path to write the HTML output (default: {DEFAULT_OUTPUT_FILE})",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        default=False,
        help="Render HTML but do not open the browser",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        default=False,
        help="Print a compact project summary to the terminal and exit",
    )
    parser.add_argument(
        "--status",
        action="store_true",
        default=False,
        help="Print current phase, readiness, and top tasks to terminal and exit",
    )

    return parser


# ── State loading ──────────────────────────────────────────────────────────────

def load_state(path: str) -> dict:
    """Load and return the dashboard state dictionary from a JSON file."""
    state_path = Path(path)
    if not state_path.is_file():
        _die(
            f"State file not found: {state_path}\n"
            f"Run the following to create a fresh state file:\n"
            f"  python platform/dashboard/dashboard.py --init",
            code=1,
        )
    try:
        with open(state_path, "r", encoding="utf-8") as f:
            state = json.load(f)
    except json.JSONDecodeError as exc:
        _die(f"State file contains invalid JSON: {exc}", code=1)
    except OSError as exc:
        _die(f"Could not read state file: {exc}", code=1)

    # Update the generated_at timestamp on load
    state.setdefault("meta", {})["generated_at"] = datetime.now().isoformat()
    return state


# ── Render ─────────────────────────────────────────────────────────────────────

def render_dashboard(state: dict, output_path: str) -> Path:
    """
    Render the dashboard HTML and write it to output_path.

    Returns the resolved output Path on success.
    """
    DashboardRenderer = _import_renderer()
    renderer = DashboardRenderer()

    try:
        html = renderer.render(state)
    except Exception as exc:
        _die(f"Rendering failed: {exc}", code=2)

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        out.write_text(html, encoding="utf-8")
    except OSError as exc:
        _die(f"Could not write output file {out}: {exc}", code=2)

    return out


# ── Terminal display helpers ───────────────────────────────────────────────────

def print_summary(state: dict) -> None:
    """Print a compact project summary to the terminal."""
    project  = state.get("project", {})
    research = state.get("research", {})
    roadmap  = state.get("roadmap", [])

    active_q_id = research.get("active_question_id", "")
    active_q    = next((q for q in research.get("questions", []) if q["id"] == active_q_id), None)

    complete   = sum(1 for p in roadmap if p.get("status") == "complete")
    total      = len(roadmap)
    readiness  = project.get("research_readiness", 0)

    sep = "─" * 62
    print(f"\n{sep}")
    print(f"  🚁  {project.get('name','Research Platform')}")
    print(sep)
    print(f"  Phase:      {project.get('current_phase',0)} / {total}  ({complete} complete)")
    print(f"  Branch:     {project.get('current_branch','—')}")
    print(f"  Sprint:     {project.get('current_sprint','—')}")
    print(f"  Readiness:  {readiness}%")
    print(f"  Updated:    {project.get('last_updated','—')}")

    if active_q:
        print(f"\n  ❓  Q: {active_q['text'][:72]}{'…' if len(active_q['text'])>72 else ''}")

    tasks = [t for t in state.get("next_tasks", []) if not t.get("done")]
    critical = [t for t in tasks if t.get("priority") == "critical"]
    if critical:
        print(f"\n  🔴  Critical tasks:")
        for t in critical[:3]:
            print(f"       · {t['text'][:70]}")
    print(f"\n{sep}\n")


def print_status(state: dict) -> None:
    """Print a brief status snapshot to the terminal."""
    project = state.get("project", {})
    roadmap = state.get("roadmap", [])
    tasks   = [t for t in state.get("next_tasks", []) if not t.get("done")]

    current_phase = next(
        (p for p in roadmap if p.get("id") == project.get("current_phase")), {}
    )

    print(f"\n  Phase {project.get('current_phase','?')}: "
          f"{current_phase.get('name','—')} — "
          f"{current_phase.get('progress',0)}% complete")
    print(f"  Research Readiness: {project.get('research_readiness',0)}%")
    print(f"  Branch: {project.get('current_branch','—')}")
    print(f"  Pending tasks: {len(tasks)}\n")

    if tasks:
        print("  Next steps:")
        for t in sorted(tasks, key=lambda x: {"critical":0,"high":1,"medium":2,"low":3}.get(x.get("priority","low"),3))[:5]:
            prio = t.get("priority","?").upper()
            print(f"    [{prio:8s}] {t['text'][:68]}")
    print()


# ── Browser open ───────────────────────────────────────────────────────────────

def open_in_browser(html_path: Path) -> None:
    """Open the rendered HTML file in the system's default browser."""
    url = html_path.as_uri()   # converts to file:///... URL
    try:
        webbrowser.open(url)
    except Exception as exc:
        _warn(f"Could not open browser automatically: {exc}")
        print(f"  → Open manually: {html_path}")


# ── Error helpers ──────────────────────────────────────────────────────────────

def _die(message: str, code: int = 1) -> None:
    print(f"\n  ✗ ERROR: {message}\n", file=sys.stderr)
    sys.exit(code)


def _warn(message: str) -> None:
    print(f"  ⚠ WARNING: {message}", file=sys.stderr)


# ── Main ───────────────────────────────────────────────────────────────────────

def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args   = parser.parse_args(argv)

    # ── Load state ────────────────────────────────────────────────────────────
    state = load_state(args.state)

    # ── Terminal-only modes ───────────────────────────────────────────────────
    if args.summary:
        print_summary(state)
        return 0

    if args.status:
        print_status(state)
        return 0

    # ── Render HTML ───────────────────────────────────────────────────────────
    _print_step("Reading state", args.state)
    _print_step("Rendering dashboard")

    html_path = render_dashboard(state, args.output)
    _print_ok(f"Dashboard written → {html_path}")

    # ── Open browser ──────────────────────────────────────────────────────────
    if not args.no_browser:
        _print_step("Opening browser")
        open_in_browser(html_path)
        _print_ok("Dashboard opened in browser")
    else:
        print(f"\n  → To open: {html_path.as_uri()}\n")

    return 0


def _print_step(msg: str, detail: str = "") -> None:
    suffix = f"  ({detail})" if detail else ""
    print(f"  ⟳  {msg}{suffix}")


def _print_ok(msg: str) -> None:
    print(f"  ✓  {msg}")


# ── Script entry point ────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Configure stdout to UTF-8 so Unicode renders on all terminals
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print()
    print("  ============================================================")
    print("   Research Dashboard -- Hybrid UAV Navigation Platform")
    print("  ============================================================")
    sys.exit(main())
