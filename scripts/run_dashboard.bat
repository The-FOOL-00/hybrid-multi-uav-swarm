@echo off
cd /d "%~dp0\.."
echo Launching Research Dashboard...
python platform\dashboard\dashboard.py
pause
