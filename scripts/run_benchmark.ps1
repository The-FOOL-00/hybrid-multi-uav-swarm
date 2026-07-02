Set-Location -Path "$PSScriptRoot\.."
Write-Host "Starting Benchmark Validation Loop (5 Runs)..." -ForegroundColor Cyan
python val_loop.py
