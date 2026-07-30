# Planner Benchmarking Platform

The **Hybrid Multi-UAV Swarm Surveillance** repository includes a standalone Benchmarking Platform. This platform allows you to evaluate and compare different deterministic path planning algorithms (A*, Dijkstra, RRT) in a highly repeatable environment.

## Overview

The benchmark runs inside Webots R2025a using the same environment (`single_drone_downtown.wbt`), drone model, collision avoidance system, motion controller, and mission parameters for all planners. This ensures apples-to-apples comparison.

## Running Benchmarks

Use the `benchmark_runner.py` CLI script located in the project root.

### Interactive Menu

Run the script without arguments to open an interactive menu:
```bash
python benchmark_runner.py
```

Options:
1. **A***: Runs the A-Star global planner.
2. **Dijkstra**: Runs the Dijkstra global planner.
3. **RRT**: Runs the Bidirectional RRT planner.
4. **All three planners**: Runs them sequentially.
5. **Report only**: Regenerates the comparison report from existing data.

### Headless CLI Arguments

You can bypass the menu for scripting/automation:
```bash
python benchmark_runner.py --planner astar
python benchmark_runner.py --planner dijkstra
python benchmark_runner.py --planner rrt
python benchmark_runner.py --all
python benchmark_runner.py --report-only
```

Windows batch/PowerShell shortcuts are also available:
```cmd
scripts\run_benchmark_planner.bat --planner astar
scripts\run_benchmark_planner.ps1 --all
```

## Directory Structure

When a benchmark runs, it sets `BENCHMARK_MODE=true` and routes all telemetry output to a planner-specific subfolder in `experiments/benchmarks/`:

```text
experiments/benchmarks/
├── A_star/
│   ├── runs.csv                    (Appended each run)
│   ├── run_001/
│   │   ├── metrics.json
│   │   ├── metrics.csv
│   │   ├── trajectory.csv          (Raw step-by-step telemetry)
│   │   └── flight_trajectory.png   (If generated)
│   └── run_002/
├── Dijkstra/
│   └── ...
├── RRT/
│   └── ...
└── comparison/
    ├── planner_comparison.csv      (Aggregated summary)
    ├── comparison_report.md        (Markdown table of averages)
    └── comparison_plots.png        (6 Bar charts comparing performance)
```

## Metrics Collected

The platform automatically collects and charts the following metrics:
1. **Planning Time (ms)**: Computation time spent in the planner's `plan()` method.
2. **Mission Time (s)**: Total real-time seconds to complete the navigation mission.
3. **Distance Travelled (m)**: Total 3D distance flown by the UAV.
4. **Min Obstacle Distance (m)**: The closest the UAV ever got to a building surface.
5. **Near Misses**: Number of steps the drone spent inside the `emergency_radius`.
6. **Max Speed (m/s)**: Peak XY velocity achieved during the flight.
7. **Replans**: Number of times the Collision Safety Layer rejected an unsafe path segment and fell back to a reactive waypoint.
8. **Smoothness Score**: Geometric smoothness of the generated path (lower is smoother).

## Technical Details

- **No Code Changes Required**: The `benchmark_runner.py` uses the `PLANNER_TYPE` environment variable to override the YAML configuration. You do not need to manually edit `configs/environment_config.yaml` to switch planners.
- **Auto-Quit**: Benchmarks run with `WEBOTS_HEADLESS=true`. Once the drone reaches the `ARRIVED` state, Webots automatically saves metrics and terminates.
- **Reporting**: The `generate_comparison.py` script scans all `run_NNN` folders and re-builds the `comparison/` directory outputs.
