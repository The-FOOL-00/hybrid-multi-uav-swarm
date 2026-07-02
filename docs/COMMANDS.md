# Command Reference

This is the official execution manual for the Hybrid Multi-UAV Navigation Research Platform. Below are the exact commands required to run every component of the system. 

---

## 1. Dashboard

**Purpose**: Generates the static HTML research dashboard based on the latest experiment and architecture status (`dashboard_state.json`), and optionally opens it in the browser.

**Command**:
```bash
python platform/dashboard/dashboard.py
```
*(Use `--no-browser` to only generate the file without opening the browser)*

**Expected output**:
```text
  ============================================================
   Research Dashboard -- Hybrid UAV Navigation Platform
  ============================================================
  ⟳  Reading state  (...)
  ⟳  Rendering dashboard
  ✓  Dashboard written → ...\dashboard.html
```

---

## 2. Webots (GUI Simulation)

**Purpose**: Launches Webots in GUI mode for visual debugging and presentation. The default world is the "downtown" scenario.

**Command**:
```bash
python run_simulation.py
```

**Alternative Commands**:
- List all available scenarios: `python run_simulation.py --list`
- Run the event scenario: `python run_simulation.py --scenario event`
- Run the residential scenario: `python run_simulation.py --scenario residential`
- Run the mixed scenario: `python run_simulation.py --scenario mixed`
- Run the industrial scenario: `python run_simulation.py --scenario industrial`

**Expected output**:
The Webots 3D interface will open, loading the specified world and placing the drones.

---

## 3. Simulation (Headless / Experiment)

**Purpose**: Launches a specific scenario in headless mode without the Webots GUI. This is used for running fast experiments where only the console output and metrics are needed.

**Command**:
```bash
python run_simulation.py --scenario single_drone --headless
```

**Expected output**:
Webots will run silently in the background. Progress and metrics (e.g., coverage, distance) will be printed directly to the terminal.

---

## 4. Benchmark

**Purpose**: Runs the main validation loop which executes the `single_drone` headless simulation 5 consecutive times to benchmark reliability, trajectory, and consistency.

**Command**:
```bash
python val_loop.py
```

**Expected output**:
```text
Starting validation loop (5 runs)...
=== RUN 1 ===
Run 1 finished naturally in 45.2s
Log for run 1 written to val_run_1.log
=== RUN 2 ===
...
All 5 runs completed.
```
*(Produces `val_run_1.log` to `val_run_5.log` in the root directory)*

---

## 5. Reports

**Purpose**: Parses the historical performance data (`baseline_runs.csv`) and the latest run's metrics to generate visual plots and a markdown report.

**Command**:
```bash
python experiments/single_drone/generate_plots.py
python experiments/single_drone/generate_report.py
```

**Expected output**:
Images (`baseline_comparison.png`, `flight_trajectory.png`) and the markdown report (`mission_report.md`) are generated inside `experiments/single_drone/` and automatically archived in the latest specific run folder.

---

## 6. Utilities & Cleaning

**Purpose**: Check or fix unicode encoding issues in Python files across the repository.

**Command (Check)**:
```bash
python check_unicode.py
```

**Command (Fix)**:
```bash
python fix_unicode2.py
```

---

## EXECUTION CHEAT SHEET

-----------------------------------------
Run Dashboard
`python platform/dashboard/dashboard.py`
-----------------------------------------
Run Webots
`python run_simulation.py`
-----------------------------------------
Run Simulation (Headless)
`python run_simulation.py --scenario single_drone --headless`
-----------------------------------------
Run Experiment (Validation Loop)
`python val_loop.py`
-----------------------------------------
Generate Report
`python experiments/single_drone/generate_plots.py && python experiments/single_drone/generate_report.py`
-----------------------------------------
Run Everything (Windows Batch)
`scripts\run_all.bat`
-----------------------------------------
