@echo off
setlocal
cd /d "%~dp0\.."

python benchmark_runner.py %*

endlocal
