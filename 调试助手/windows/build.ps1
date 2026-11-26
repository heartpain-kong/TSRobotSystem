$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

python -m pip install -r requirements.txt
python -m PyInstaller `
  --noconfirm --clean --onefile --noconsole `
  --name "调试助手" `
  --hidden-import serial.tools.list_ports_windows `
  --collect-submodules serial `
  "调试助手.py"

Write-Host ""
Write-Host "生成: $PSScriptRoot\dist\调试助手.exe" -ForegroundColor Green
