"""
platform/dashboard/ — Research Dashboard Package

Provides the Research Dashboard — the single entry point for all project
participants. The dashboard answers: "Where is the project right now and
what should be done next?"

Public API
----------
    DashboardWriter   — Write / update dashboard state from other modules
    DashboardRenderer — Render state dict → self-contained HTML document
    DashboardRunner   — Orchestrate read → render → open workflow (CLI)

All state is stored in dashboard_state.json.  Every other platform module
writes to that file via DashboardWriter.  The dashboard itself is a static
HTML file that any browser can open without a web server.
"""

from platform.dashboard.writer import DashboardWriter
from platform.dashboard.renderer import DashboardRenderer

__all__ = ["DashboardWriter", "DashboardRenderer"]
