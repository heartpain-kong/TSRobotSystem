#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

# 必须在 x86_64 Ubuntu 22.04 上执行；PyInstaller 不能跨架构打包。
if [ "$(uname -m)" != "x86_64" ]; then
  echo "当前架构: $(uname -m)，本目录是 x64 包，请在 x86_64 上构建。"
  exit 1
fi

python3 -m pip install --user -r requirements.txt
python3 -m PyInstaller \
  --noconfirm --clean --onefile --windowed \
  --name "调试助手" \
  --collect-submodules serial \
  "调试助手.py"

echo "生成: $(pwd)/dist/调试助手"
