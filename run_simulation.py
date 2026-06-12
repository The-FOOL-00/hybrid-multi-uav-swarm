"""
Launch script for Multi-UAV Surveillance Simulation
====================================================
Usage:
    python run_simulation.py                               # GUI, downtown (5 UAVs)
    python run_simulation.py --scenario single_drone       # GUI, single-drone dev world
    python run_simulation.py --scenario event              # GUI, event
    python run_simulation.py --headless                    # Headless, downtown
    python run_simulation.py --scenario single_drone --headless
    python run_simulation.py --scenario mixed --headless
    python run_simulation.py --list                        # Show all scenarios
    python run_simulation.py --help                        # Full usage

Scenarios:
    downtown      — Full 5-UAV swarm, 15 buildings, 15 pedestrians  [production]
    single_drone  — DEV: 1 UAV only, same downtown, chase-cam       [development]
    event         — Dense crowd event plaza
    residential   — Low-density neighborhood
    mixed         — Mixed commercial + residential zones
    industrial    — Port/warehouse environment
"""
import subprocess
import argparse
import os
import sys

# ── Webots executable paths (tries both; edit to match your installation) ──────
_WEBOTS_CANDIDATES = [
    r"C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe",  # R2025a typical
    r"C:\Program Files\Webots\webots.exe",                       # Some versions
    r"C:\Program Files\Webots\bin\webots.exe",                   # Alternate
]

# ── Scenario registry ──────────────────────────────────────────────────────────
SCENARIOS = {
    "downtown":      "worlds/downtown.wbt",
    "single_drone":  "worlds/single_drone_downtown.wbt",
    "event":         "worlds/event.wbt",
    "residential":   "worlds/residential.wbt",
    "mixed":         "worlds/mixed.wbt",
    "industrial":    "worlds/industrial.wbt",
}

INFO = {
    "downtown":     {"buildings": 15, "crowd": 15,  "uavs": 5, "birds": 8},
    "single_drone": {"buildings": 15, "crowd": 15,  "uavs": 1, "birds": 8},
    "event":        {"buildings": 5,  "crowd": 40,  "uavs": 5, "birds": 8},
    "residential":  {"buildings": 8,  "crowd": 8,   "uavs": 5, "birds": 6},
    "mixed":        {"buildings": 14, "crowd": 35,  "uavs": 5, "birds": 8},
    "industrial":   {"buildings": 2,  "crowd": 10,  "uavs": 5, "birds": 5},
}

SCENARIO_DESCRIPTIONS = {
    "downtown":     "Dense urban 3x3 road grid, 15 high-rise buildings",
    "single_drone": "[DEV] Single-drone nav+avoidance dev world — UAV_0 only, chase-cam, (-40,48,15)→(50,48,15)",
    "event":        "Public event - central plaza, 40 crowd agents",
    "residential":  "Low-density neighborhood with parks, 8 houses",
    "mixed":        "Mixed commercial + residential zones, 35 agents",
    "industrial":   "Port/warehouse with containers and cranes, 10 workers",
}


def _find_webots() -> str:
    """Return first valid Webots executable path, or empty string."""
    for path in _WEBOTS_CANDIDATES:
        if os.path.isfile(path):
            return path
    return ""


def list_scenarios():
    """Print available scenarios and exit."""
    print("\n  Available scenarios:")
    print("  " + "-" * 52)
    for name, desc in SCENARIO_DESCRIPTIONS.items():
        info = INFO[name]
        print(f"  {name:<14} UAVs:{info['uavs']}  Crowd:{info['crowd']:>3}  {desc}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Launch Multi-UAV Surveillance Simulation in Webots R2025a",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_simulation.py                          GUI mode, downtown
  python run_simulation.py --scenario event         GUI mode, event
  python run_simulation.py --headless               Headless mode, downtown
  python run_simulation.py --scenario mixed --headless
  python run_simulation.py --list                   Show all scenarios
        """,
    )
    parser.add_argument(
        "--scenario",
        default="single_drone",
        choices=list(SCENARIOS.keys()),
        metavar="SCENARIO",
        help=f"Scenario to load. Choices: {', '.join(SCENARIOS)} (default: single_drone)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Headless mode: no GUI window, faster execution (for training/research)",
    )
    parser.add_argument(
        "--no-rendering",
        dest="headless",
        action="store_true",
        help="Alias for --headless",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all available scenarios and exit",
    )
    parser.add_argument(
        "--webots-exe",
        default="",
        metavar="PATH",
        help="Override Webots executable path",
    )
    args = parser.parse_args()

    # ── List mode ────────────────────────────────────────────────────────────
    if args.list:
        list_scenarios()
        sys.exit(0)

    # ── Resolve Webots executable ─────────────────────────────────────────────
    webots_exe = args.webots_exe or _find_webots()
    if not webots_exe or not os.path.isfile(webots_exe):
        print("[ERROR] Webots executable not found.")
        print("        Tried:")
        for p in _WEBOTS_CANDIDATES:
            print(f"          {p}")
        print("        Use --webots-exe /path/to/webots.exe to specify manually.")
        sys.exit(1)

    # ── Resolve world path ────────────────────────────────────────────────────
    script_dir = os.path.dirname(os.path.abspath(__file__))
    world_path = os.path.join(script_dir, SCENARIOS[args.scenario])

    if not os.path.isfile(world_path):
        print(f"[ERROR] World file not found: {world_path}")
        sys.exit(1)

    # ── Mode banner ───────────────────────────────────────────────────────────
    mode_label = "HEADLESS (Training / Research)" if args.headless else "GUI     (Demo / Debugging)"
    info = INFO[args.scenario]

    print(f"\n{'='*58}")
    print(f"  Multi-UAV Surveillance Simulation — Webots R2025a")
    print(f"{'='*58}")
    print(f"  Mode      : {mode_label}")
    print(f"  Scenario  : {args.scenario.upper()}")
    print(f"  World     : {world_path}")
    print(f"  Buildings : {info['buildings']}")
    print(f"  Crowd     : {info['crowd']}")
    print(f"  UAVs      : {info['uavs']}")
    print(f"  Birds     : {info['birds']}")
    if args.headless:
        print(f"  Rendering : DISABLED (--batch --minimize --no-rendering)")
    else:
        print(f"  Rendering : ENABLED  (full 3D visualization)")
    print(f"{'='*58}\n")

    # ── Build command ─────────────────────────────────────────────────────────
    cmd = [webots_exe]
    if args.headless:
        cmd += ["--batch", "--minimize", "--no-rendering", "--mode=fast", "--stdout", "--stderr"]
    cmd.append(world_path)

    print(f"Launching: {' '.join(cmd)}\n")
    
    # Propagate headless status to supervisor controller via environment variables
    env = os.environ.copy()
    env["WEBOTS_HEADLESS"] = "true" if args.headless else "false"
    
    proc = subprocess.Popen(cmd, env=env)

    if args.headless:
        print("Webots running in headless mode. Metrics will appear in console.")
        print("Waiting for simulation to finish...")
        proc.wait()
    else:
        print("Webots launched. Use Webots Play button to start simulation.")
        print("Coverage metrics will appear in the Webots console every 125 steps.")


if __name__ == "__main__":
    main()
