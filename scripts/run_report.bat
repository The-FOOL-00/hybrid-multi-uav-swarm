@echo off
cd /d "%~dp0\.."
echo Generating Plots and Report...
python experiments\single_drone\generate_plots.py
python experiments\single_drone\generate_report.py
pause
