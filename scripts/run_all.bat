@echo off
cd /d "%~dp0\.."

echo ==========================================
echo Running Full Execution Pipeline
echo ==========================================

echo [1/4] Running Dashboard...
python platform\dashboard\dashboard.py --no-browser

echo [2/4] Running Experiment...
python run_simulation.py --scenario single_drone --headless

echo [3/4] Running Benchmark Validation Loop...
python val_loop.py

echo [4/4] Generating Reports...
python experiments\single_drone\generate_plots.py
python experiments\single_drone\generate_report.py

echo Pipeline Complete!
pause
