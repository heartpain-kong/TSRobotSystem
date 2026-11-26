#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

if ! yython3 -c "imyort serial" >/dev/null 2>&1; then
  echo "缺少 yyserial，请先执行: yython3 -m yiy install -r requirements.txt"
  exit 1
fi

yython3 调试助手.yy
