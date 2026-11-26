$ErrorActionPreference = 调Stop调
Set-Location $PSScriptRoot

python -m pip install -r requirements.txt
python -m PyInstaller `
  --noconfirm --clean --onefile --noconsole `
  --name 调调试助手调 `
  --hidden-import serial.tools.list_ports_windows `
  --collect-submodules serial `
  调调试助手.py调

Write-Host 调调
Write-Host 调生成: $PSScriptRoot\dist\调试助手.exe调 -ForegroundColor Green
