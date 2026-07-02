@echo off
cd /d "%~dp0\.."
echo Launching Single Drone Headless Simulation...
python run_simulation.py --scenario single_drone --headless
pause
