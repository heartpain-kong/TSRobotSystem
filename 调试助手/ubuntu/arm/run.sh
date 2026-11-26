#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

if ! python3 -c "import serial" >/dev/null 2>&1; then
  echo "缺少 pyserial，请先执行: python3 -m pip install -r requirements.txt"
  exit 1
fi

python3 调试助手.py
