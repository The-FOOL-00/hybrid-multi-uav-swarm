@echo off
cd /d "%~dp0\.."
echo Launching Headless Experiment...
python run_simulation.py --scenario single_drone --headless
pause
