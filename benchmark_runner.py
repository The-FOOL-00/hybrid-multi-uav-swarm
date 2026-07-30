#!/usr/bin/env python3
"""
benchmark_runner.py
===================
Research-grade Planner Benchmarking Platform launcher for the
Hybrid Multi-UAV Swarm Surveillance project.

Usage
-----
Interactive menu (no arguments):
    python benchmark_runner.py

Select specific planner + execution mode via CLI:
    python benchmark_runner.py --planner astar --visual
    python benchmark_runner.py --planner dijkstra --headless
    python benchmark_runner.py --planner rrt --visual

Run all three planners sequentially:
    python benchmark_runner.py --all --headless
    python benchmark_runner.py --all --visual

Generate comparison report only (without running Webots):
    python benchmark_runner.py --report-only

Execution Modes
---------------
    Visual   : Webots window opens with full 3D rendering.
                Use this for demonstrations and debugging.
    Headless : Webots runs with --no-rendering --mode=fast.
                Use this for benchmarking (faster execution).

Environment Variables Set for Webots
-------------------------------------
    PLANNER_TYPE    : "astar" | "dijkstra" | "rrt"
    BENCHMARK_MODE  : "true"
    WEBOTS_HEADLESS : "true" | "false"

These are picked up by uav_swarm_controller.py without any YAML edits.
"""

import argparse
import os
import subprocess
import sys
import time
import glob

# =============================================================================
# Project paths
# =============================================================================
ROOT_DIR   = os.path.dirname(os.path.abspath(__file__))

# Sprint 1: Easy World is the primary benchmark environment.
# Change WORLD_FILE and CONFIG_FILE here to switch worlds without touching
# environment_config.yaml (Downtown config remains untouched).
WORLD_FILE  = os.path.join(ROOT_DIR, "worlds", "easy_world.wbt")
CONFIG_FILE = "configs/environment_easy.yaml"   # relative to ROOT_DIR

COMPARE_SCRIPT = os.path.join(ROOT_DIR, "experiments", "benchmarks",
                               "generate_comparison.py")

PLANNERS = ["astar", "dijkstra", "rrt"]
PLANNER_DISPLAY = {
    "astar":    "A*  (A-Star)",
    "dijkstra": "Dijkstra",
    "rrt":      "RRT (Bidirectional)",
}

# =============================================================================
# Webots discovery  —  reuses _find_webots() from run_simulation.py
# =============================================================================
from run_simulation import _find_webots


# =============================================================================
# Run a single benchmark
# =============================================================================

def run_benchmark(planner: str, headless: bool = True) -> bool:
    """
    Launch Webots with the chosen planner and execution mode, then wait
    for it to finish.

    Parameters
    ----------
    planner  : one of "astar", "dijkstra", "rrt"
    headless : True  -> --no-rendering flags added (faster, no window)
               False -> no extra flags, full 3D rendering in Webots window

    Returns True if the run completed (Webots exited normally).
    """
    webots = _find_webots()
    if not webots:
        print("\n[ERROR] Webots executable not found.")
        print("  Checked paths defined in run_simulation.py.")
        return False

    planner_display = PLANNER_DISPLAY.get(planner, planner.upper())
    mode_label = "HEADLESS (no rendering)" if headless else "VISUAL  (full rendering)"

    print(f"\n{'='*60}")
    print(f"  BENCHMARK : {planner_display}")
    print(f"  Mode      : {mode_label}")
    print(f"  World     : {WORLD_FILE}")
    print(f"  Webots    : {webots}")
    print(f"{'='*60}\n")

    if not os.path.isfile(WORLD_FILE):
        print(f"[ERROR] World file not found: {WORLD_FILE}")
        return False

    # Build environment for the Webots subprocess
    env = os.environ.copy()
    env["PLANNER_TYPE"]    = planner
    env["BENCHMARK_MODE"]  = "true"
    env["WEBOTS_HEADLESS"] = "true" if headless else "false"
    env["CONFIG_FILE"]     = CONFIG_FILE  # tells controller which YAML to load

    # Build command — headless adds no-rendering flags; visual uses none
    cmd = [webots]
    if headless:
        cmd += ["--batch", "--minimize", "--no-rendering", "--mode=fast",
                "--stdout", "--stderr"]
    cmd.append(WORLD_FILE)

    print(f"[BenchmarkRunner] Command: {' '.join(cmd)}")
    print(f"[BenchmarkRunner] PLANNER_TYPE={planner}  BENCHMARK_MODE=true  "
          f"WEBOTS_HEADLESS={'true' if headless else 'false'}  "
          f"CONFIG_FILE={CONFIG_FILE}")
    if headless:
        print(f"[BenchmarkRunner] Waiting for mission to complete...\n")
    else:
        print(f"[BenchmarkRunner] Webots launched in visual mode.\n"
              f"  Press Play in Webots to start the simulation.\n"
              f"  The runner will wait until Webots is closed.\n")

    t0 = time.time()
    try:
        result = subprocess.run(cmd, env=env, timeout=3600)  # 1-hour hard timeout
        elapsed = time.time() - t0
        if result.returncode in (0, 1, -1, 9):
            # Webots often exits with non-zero on clean quit
            print(f"\n[BenchmarkRunner] Webots exited (code {result.returncode}) "
                  f"after {elapsed:.1f}s")
            return True
        else:
            print(f"\n[BenchmarkRunner] Webots exited with unexpected code "
                  f"{result.returncode} after {elapsed:.1f}s")
            return False
    except subprocess.TimeoutExpired:
        print("\n[ERROR] Webots run exceeded 1-hour timeout. Process killed.")
        return False
    except FileNotFoundError:
        print(f"\n[ERROR] Could not launch Webots at: {webots}")
        return False
    except KeyboardInterrupt:
        print("\n[BenchmarkRunner] Interrupted by user.")
        return False


# =============================================================================
# Comparison report generation
# =============================================================================

def run_comparison():
    """Generate the cross-planner comparison report and graphs."""
    print(f"\n[BenchmarkRunner] Generating comparison report...")
    if not os.path.isfile(COMPARE_SCRIPT):
        print(f"[WARN] generate_comparison.py not found at: {COMPARE_SCRIPT}")
        return

    try:
        subprocess.run([sys.executable, COMPARE_SCRIPT], check=True)
    except subprocess.CalledProcessError as e:
        print(f"[WARN] generate_comparison.py exited with code {e.returncode}")
    except Exception as e:
        print(f"[WARN] Could not run generate_comparison.py: {e}")


# =============================================================================
# Interactive menus
# =============================================================================

def interactive_planner_menu() -> list[str]:
    """Show the planner selection menu and return list of selected planners."""
    print("\n" + "="*60)
    print("  PLANNER BENCHMARKING PLATFORM")
    print("  Hybrid Multi-UAV Swarm Surveillance — Phase 1")
    print("="*60)
    print("\n  Select planner(s) to benchmark:\n")
    print("    1. A*  (A-Star)        — Optimal, grid-based")
    print("    2. Dijkstra            — Exhaustive, grid-based")
    print("    3. RRT                 — Sampling-based, seeded")
    print("    4. All three planners  — Sequential benchmark")
    print("    5. Report only         — Generate comparison from existing data")
    print("    0. Exit")
    print()

    while True:
        try:
            choice = input("  Enter planner [1-5, 0]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[BenchmarkRunner] Exiting.")
            sys.exit(0)

        if choice == "0":
            sys.exit(0)
        elif choice == "1":
            return ["astar"]
        elif choice == "2":
            return ["dijkstra"]
        elif choice == "3":
            return ["rrt"]
        elif choice == "4":
            return PLANNERS[:]
        elif choice == "5":
            return []
        else:
            print("  Invalid choice. Please enter 0–5.")


def interactive_mode_menu() -> bool:
    """
    Show the execution mode menu.

    Returns
    -------
    True  -> headless (no rendering, faster)
    False -> visual   (full rendering, Webots window)
    """
    print("\n  Execution Mode:\n")
    print("    1. Visual   — Webots window opens (watch the simulation)")
    print("    2. Headless — No rendering, faster execution")
    print()

    while True:
        try:
            choice = input("  Enter mode [1-2]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[BenchmarkRunner] Exiting.")
            sys.exit(0)

        if choice == "1":
            return False   # NOT headless -> visual
        elif choice == "2":
            return True    # headless
        else:
            print("  Invalid choice. Please enter 1 or 2.")


# =============================================================================
# Results summary
# =============================================================================

def _count_runs(planner: str) -> int:
    folder_map = {"astar": "A_star", "dijkstra": "Dijkstra", "rrt": "RRT"}
    folder = folder_map.get(planner, planner.capitalize())
    planner_dir = os.path.join(ROOT_DIR, "experiments", "benchmarks", folder)
    if not os.path.isdir(planner_dir):
        return 0
    return len([d for d in os.listdir(planner_dir) if d.startswith("run_")])


def print_results_summary(planners_run: list[str]):
    """Print a quick summary of results after the benchmark run."""
    import json

    folder_map = {"astar": "A_star", "dijkstra": "Dijkstra", "rrt": "RRT"}
    print("\n" + "="*60)
    print("  BENCHMARK RESULTS SUMMARY")
    print("="*60)
    print(f"  {'Planner':<14}  {'Runs':>5}  {'Success':>8}  {'Plan(ms)':>9}  {'Mission(s)':>11}  {'Path(m)':>8}")
    print(f"  {'-'*14}  {'-'*5}  {'-'*8}  {'-'*9}  {'-'*11}  {'-'*8}")

    for p in planners_run:
        folder = folder_map.get(p, p.capitalize())
        planner_dir = os.path.join(ROOT_DIR, "experiments", "benchmarks", folder)
        runs = sorted(glob.glob(os.path.join(planner_dir, "run_*", "metrics.json")))
        if not runs:
            print(f"  {p.capitalize():<14}  {'N/A':>5}")
            continue

        # Just show the latest run
        latest_json = runs[-1]
        try:
            with open(latest_json) as f:
                d = json.load(f)
            success = "PASS" if d.get("reached_target") else "FAIL"
            plan_ms = d.get("planner_compute_time_ms", 0)
            mission_s = d.get("travel_time_s", 0)
            path_m = d.get("planner_path_length_m", 0)
            print(f"  {p:<14}  {_count_runs(p):>5}  {success:>8}  "
                  f"{plan_ms:>9.2f}  {mission_s:>11.1f}  {path_m:>8.1f}")
        except Exception:
            print(f"  {p:<14}  {_count_runs(p):>5}  (error reading metrics)")

    comparison_csv = os.path.join(ROOT_DIR, "experiments", "benchmarks",
                                   "comparison", "planner_comparison.csv")
    comparison_report = os.path.join(ROOT_DIR, "experiments", "benchmarks",
                                      "comparison", "comparison_report.md")
    comparison_plot = os.path.join(ROOT_DIR, "experiments", "benchmarks",
                                    "comparison", "comparison_plots.png")
    print()
    if os.path.isfile(comparison_csv):
        print(f"  Comparison CSV    : {comparison_csv}")
    if os.path.isfile(comparison_report):
        print(f"  Comparison Report : {comparison_report}")
    if os.path.isfile(comparison_plot):
        print(f"  Comparison Plots  : {comparison_plot}")
    print()


# =============================================================================
# Main entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Planner Benchmarking Platform — launch and compare planners.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Execution modes (choose one):
  --visual    : Open Webots with full 3D rendering  (demo / debug)
  --headless  : Run without rendering, faster       (benchmarking)

When neither flag is given the interactive menu will ask.

Examples:
  python benchmark_runner.py                          interactive
  python benchmark_runner.py --planner astar --visual
  python benchmark_runner.py --planner dijkstra --headless
  python benchmark_runner.py --all --headless
  python benchmark_runner.py --report-only
        """
    )
    parser.add_argument(
        "--planner", choices=PLANNERS,
        help="Planner to benchmark (astar / dijkstra / rrt)"
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Run all three planners sequentially"
    )
    parser.add_argument(
        "--report-only", action="store_true",
        help="Skip Webots runs; only regenerate comparison report from existing data"
    )

    # Execution mode — mutually exclusive
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--visual", action="store_true",
        help="Launch Webots with full rendering (watch the simulation)"
    )
    mode_group.add_argument(
        "--headless", action="store_true",
        help="Launch Webots with no rendering for faster benchmarking"
    )

    args = parser.parse_args()

    # ---- Determine which planners to run ----
    if args.report_only:
        planners_to_run = []
    elif args.all:
        planners_to_run = PLANNERS[:]
    elif args.planner:
        planners_to_run = [args.planner]
    else:
        # Interactive — show planner menu
        planners_to_run = interactive_planner_menu()

    # ---- Determine execution mode ----
    if args.report_only or not planners_to_run:
        # No Webots launch needed; mode doesn't matter
        run_headless = True
    elif args.headless:
        run_headless = True
    elif args.visual:
        run_headless = False
    else:
        # Interactive — ask only when there are planners to run
        run_headless = interactive_mode_menu()

    # ---- Print selected configuration ----
    if planners_to_run:
        mode_str = "HEADLESS (no rendering)" if run_headless else "VISUAL  (full rendering)"
        print(f"\n  Selected configuration:")
        print(f"    Planners  : {', '.join(p.upper() for p in planners_to_run)}")
        print(f"    Mode      : {mode_str}")
        print()

    # ---- Run benchmarks ----
    ran_planners = []
    for planner in planners_to_run:
        ok = run_benchmark(planner, headless=run_headless)
        if ok:
            ran_planners.append(planner)
        else:
            print(f"\n[BenchmarkRunner] WARNING: Benchmark for '{planner}' may not have "
                  f"completed successfully.")

    # Generate comparison report
    run_comparison()

    # Print summary
    if ran_planners or args.report_only:
        print_results_summary(PLANNERS if args.report_only else ran_planners)

    print("[BenchmarkRunner] Done.\n")


if __name__ == "__main__":
    main()
