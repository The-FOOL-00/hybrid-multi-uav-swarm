@echo off
cd /d "%~dp0\.."
echo Starting Benchmark Validation Loop (5 Runs)...
python val_loop.py
pause
