#!/usr/bin/env bash
set -euo pipefail
integration_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
board_side="${1:-left}"
case "$board_side" in left|right) ;; *) echo "Usage: bash build-local.sh [left|right]" >&2; exit 2;; esac
export ZEPHYR_BASE="${ZEPHYR_BASE:-/home/aster/robotics/Winora_Firmwares/zephyr}"
export ZEPHYR_SDK_INSTALL_DIR="${ZEPHYR_SDK_INSTALL_DIR:-/home/aster/zephyr-sdk-1.0.1}"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export PATH="/home/aster/robotics/Winora_Firmwares/.venv/bin:/home/aster/.local/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
board_overlay="$integration_root/local-mcxn947.overlay"
board_conf="$integration_root/local-workspace.conf;$integration_root/dual-common.conf;$integration_root/boards/$board_side.conf"
if [[ "$board_side" == right ]]; then
    board_overlay+=";$integration_root/boards/gello-usb.overlay"
    board_conf+=";$integration_root/boards/gello-usb.conf"
fi
west build -b frdm_mcxn947/mcxn947/cpu0 \
    -d "$integration_root/build-$board_side" "$integration_root/source" -- \
    -DDTC_OVERLAY_FILE="$board_overlay" \
    -DEXTRA_CONF_FILE="$board_conf" \
    -DUSER_CACHE_DIR="$integration_root/.cache"
