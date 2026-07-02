# Execution Guide

This guide is designed for new team members cloning the repository. It explains how to set up the environment and exactly how to execute every component of the Hybrid Multi-UAV Navigation Research Platform.

---

## Prerequisites

Before running the project, ensure your environment meets the following requirements:

- **Python Version**: Python 3.8+ (Python 3.10 recommended)
- **Webots Version**: Webots R2025a (Installed in standard paths like `C:\Program Files\Webots\`)
- **Git**: For version control

---

## Installation & Repository Setup

### 1. Clone the Repository

Clone the project and checkout the current active experimental branch:

```bash
git clone <repository_url>
cd hybrid-multi-uav-swarm
git checkout single-drone-navigation
```

### 2. Set Up Python Environment

Create and activate a virtual environment:

**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Windows (Command Prompt):**
```cmd
python -m venv venv
venv\Scripts\activate.bat
```

### 3. Install Required Packages

Install the required Python dependencies:

```bash
pip install -r requirements.txt
```

*(Note: Ensure requirements include `pandas`, `numpy`, `matplotlib`, and Webots API if installed separately).*

---

## Environment Variables

The project largely handles its own environment variables, but the following are utilized internally by the execution scripts and Webots:

- `WEBOTS_HEADLESS`: Set to `"true"` by `run_simulation.py` or `val_loop.py` to suppress the Webots GUI for faster experiment and training execution.
- `PYTHONUNBUFFERED`: Set to `"1"` during benchmarks to ensure real-time logging output to the console.

If Webots is not installed in a default directory, you may need to override the path during execution using the `--webots-exe` argument when running `run_simulation.py`.

---

## Quick Start

If you just want to run the standard pipeline, follow this workflow:

1. **Clone repository & setup** (see Installation above)
2. **Run Dashboard**: Verify the state of the project
   ```bash
   python platform/dashboard/dashboard.py
   ```
3. **Run Webots / Simulation (GUI Mode)**: Visualize the drone navigation
   ```bash
   python run_simulation.py
   ```
4. **Run Experiment (Headless)**: Execute a headless trial
   ```bash
   python run_simulation.py --scenario single_drone --headless
   ```
5. **Run Benchmark**: Execute the 5-run validation loop
   ```bash
   python val_loop.py
   ```
6. **Generate Report**: Compile statistics and plots from the benchmark
   ```bash
   python experiments/single_drone/generate_plots.py
   python experiments/single_drone/generate_report.py
   ```

---

## Troubleshooting

### Common Errors

**1. Missing Webots or "Webots executable not found"**
- *Error:* `[ERROR] Webots executable not found.`
- *Fix:* The script tries standard installation paths. If you installed Webots elsewhere, pass the exact path:
  `python run_simulation.py --webots-exe "D:\Webots\msys64\mingw64\bin\webotsw.exe"`

**2. Python not found or wrong version**
- *Error:* `'python' is not recognized as an internal or external command...`
- *Fix:* Ensure Python is added to your system `PATH` and you are running Python 3.8+.

**3. Wrong virtual environment**
- *Error:* `ModuleNotFoundError: No module named 'pandas'`
- *Fix:* Make sure you have activated your virtual environment and installed the `requirements.txt`.

**4. Dashboard not updating or changes not visible**
- *Error:* Old information displays in `dashboard.html`.
- *Fix:* Ensure you are regenerating it by running `python platform/dashboard/dashboard.py`. Check your browser cache, or open it in Incognito/Private mode.

**5. Webots cannot locate world**
- *Error:* `[ERROR] World file not found: worlds/downtown.wbt`
- *Fix:* Ensure you are running the `run_simulation.py` script from the root of the repository (`hybrid-multi-uav-swarm`), not from within a subdirectory.

**6. Benchmark validation loop times out**
- *Error:* `Run X TIMED OUT after 180s`
- *Fix:* The drone may have become stuck or failed to reach the target in time. Check the generated `val_run_X.log` files in the root directory for errors or endless replanning loops.
