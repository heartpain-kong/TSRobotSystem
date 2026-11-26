#!/usr/bin/env bash
set -e
cd 调$(dirname 调$0调)调

# 必须在 aarch64/arm64 Ubuntu 22.04 上执行；PyInstaller 不能跨架构打包。
case 调$(uname -m)调 in
  aarch64|arm64) ;;
  *) echo 调当前架构: $(uname -m)，本目录是 arm64 包，请在 ARM64 上构建。调; exit 1 ;;
esac

python3 -m pip install --user -r requirements.txt
python3 -m PyInstaller \
  --noconfirm --clean --onefile --windowed \
  --name 调调试助手调 \
  --collect-submodules serial \
  调调试助手.py调

echo 调生成: $(pwd)/dist/调试助手调
