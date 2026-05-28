"""
Launch script for Multi-UAV Surveillance Simulation
Usage: python run_simulation.py --scenario downtown [--headless]
"""
import subprocess
import argparse
import os
import sys

WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"

SCENARIOS = {
    "downtown":    "worlds/downtown.wbt",
    "event":       "worlds/event.wbt",
    "residential": "worlds/residential.wbt",
    "mixed":       "worlds/mixed.wbt",
    "industrial":  "worlds/industrial.wbt",
}

INFO = {
    "downtown":    {"buildings": 15, "crowd": 15,  "uavs": 5, "birds": 8},
    "event":       {"buildings": 5,  "crowd": 40,  "uavs": 5, "birds": 8},
    "residential": {"buildings": 8,  "crowd": 8,   "uavs": 5, "birds": 6},
    "mixed":       {"buildings": 14, "crowd": 35,  "uavs": 5, "birds": 8},
    "industrial":  {"buildings": 2,  "crowd": 10,  "uavs": 5, "birds": 5},
}


def main():
    parser = argparse.ArgumentParser(
        description="Launch Multi-UAV Surveillance Simulation in Webots R2025a")
    parser.add_argument(
        "--scenario",
        default="downtown",
        choices=list(SCENARIOS.keys()),
        help="Scenario to load (default: downtown)")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Webots in headless/batch mode (no GUI)")
    args = parser.parse_args()

    # Resolve world path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    world_path = os.path.join(script_dir, SCENARIOS[args.scenario])

    if not os.path.isfile(world_path):
        print(f"[ERROR] World file not found: {world_path}")
        sys.exit(1)

    if not os.path.isfile(WEBOTS_EXE):
        print(f"[ERROR] Webots not found at: {WEBOTS_EXE}")
        print("        Edit WEBOTS_EXE in run_simulation.py to match your installation.")
        sys.exit(1)

    # Print scenario info
    info = INFO[args.scenario]
    print(f"\n{'='*55}")
    print(f"  Multi-UAV Surveillance Simulation")
    print(f"  Scenario  : {args.scenario.upper()}")
    print(f"  World     : {world_path}")
    print(f"  Buildings : {info['buildings']}")
    print(f"  Crowd     : {info['crowd']}")
    print(f"  UAVs      : {info['uavs']}")
    print(f"  Birds     : {info['birds']}")
    print(f"{'='*55}\n")

    cmd = [WEBOTS_EXE]
    if args.headless:
        cmd += ["--batch", "--minimize", "--no-rendering"]
    cmd.append(world_path)

    print(f"Launching: {' '.join(cmd)}")
    subprocess.Popen(cmd)
    print("Webots launched. Check the console for simulation metrics.")


if __name__ == "__main__":
    main()
