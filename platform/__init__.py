"""
platform/ — Research Platform Infrastructure

The platform package provides the engineering layer that sits above the
Webots simulation. It is responsible for:

  - Research Dashboard (entry point for all project participants)
  - Experiment Manager (reproducible experiment execution)
  - Configuration Manager (versioned parameter profiles)
  - Metrics Engine (validated measurement collection)
  - Benchmark Engine (statistical comparative analysis)
  - Report Generator (publication-ready output)
  - Knowledge Base Manager (structured research memory)

The platform is simulation-agnostic. It does not import any Webots modules
and can be used, tested, and extended without Webots being installed.

Usage
-----
    # Open the Research Dashboard
    python platform/dashboard/dashboard.py

    # Update dashboard state from any platform module
    from platform.dashboard.writer import DashboardWriter
    writer = DashboardWriter()
    writer.add_experiment({...})
    writer.save()

Package Version
---------------
    1.0.0 — Phase 1: Research Dashboard Infrastructure
"""

__version__ = "1.0.0"
__author__ = "Hybrid Multi-UAV Navigation Research Team"
