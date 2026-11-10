#!/usr/bin/env bash
# 启动 MuJoCo 洗碗机场景 + Gello 主手遥操
set -euo pipefail
cd "$(dirname "$0")"

exec ./.venv/bin/python run_gello.py "$@"
