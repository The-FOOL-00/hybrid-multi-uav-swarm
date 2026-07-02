Set-Location -Path "$PSScriptRoot\.."
Write-Host "Launching Research Dashboard..." -ForegroundColor Cyan
python platform\dashboard\dashboard.py
