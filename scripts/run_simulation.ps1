Set-Location -Path "$PSScriptRoot\.."
Write-Host "Launching Single Drone Headless Simulation..." -ForegroundColor Cyan
python run_simulation.py --scenario single_drone --headless
