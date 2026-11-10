#!/usr/bin/env bash
set -euo pipefail
integration_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
host_python="${PANTHERA_PYTHON:-/home/aster/miniforge3/envs/panthera/bin/python}"
cd "$integration_root/source/host/examples"
"$host_python" -m unittest -v \
    test_arm_examples test_hand_slider_demo test_priority_calibration test_hand_bus_selection test_board_identity test_gello_home_zero \
    test_gello_arm_compare.Tests.test_mujoco_mapping_matches_original_with_offsets \
    test_gello_arm_compare.Tests.test_read_packet_and_signed_velocity \
    test_gello_arm_compare.Tests.test_missing_stale_data_and_direction_conversion \
    test_gello_arm_compare.Tests.test_arm_worker_sends_only_gravity_not_gello_targets
cd "$integration_root"
gcc -std=c11 -Isource/src source/tests/test_id_and_full_checksum.c \
    source/src/devices/can_codec.c -o evidence/test_can_codec
./evidence/test_can_codec
gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_peer_codec.c \
    source/src/peer/peer_codec.c -o evidence/test_peer_codec
./evidence/test_peer_codec evidence/peer-c-golden.bin
PYTHONPATH=source/host "$host_python" -c 'from pathlib import Path;from dual_peer import decode;s=decode(Path("evidence/peer-c-golden.bin").read_bytes());assert s["node"]==12 and s["boot"]==0x123456789abcdef0 and s["hand_control_ack"]==456 and s["motor_feedback_count"]==list(range(1000,1006));print("Peer C/Python wire compatibility passed")'
"$host_python" -m unittest discover -s source/host/examples -p test_dual_peer.py -v
"$host_python" -m unittest discover -s source/host/examples -p test_one_gello_two_arms.py -v
"$host_python" -m unittest discover -s source/host/examples -p test_soft_limits.py -v
"$host_python" -m unittest discover -s source/host/examples -p test_teleop_tuning.py -v
"$host_python" -m unittest discover -s source/host/examples -p test_gravity_wall.py -v

gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_local_home.c source/src/homing/homing.c source/src/homing/home_path.c source/src/homing/home_still.c source/src/homing/gravity.c -lm -o evidence/test_local_home
./evidence/test_local_home
gcc -std=c11 -Wall -Wextra -Werror -shared -fPIC -Isource/src source/src/homing/gravity.c -lm -o evidence/home-gravity.so
"$host_python" tools/check-home-model.py
"$host_python" -m unittest discover -s source/host/examples -p test_local_home.py -v

gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_paired_home.c source/src/homing/paired_home.c -o evidence/test_paired_home
./evidence/test_paired_home

gcc -std=c11 -Wall -Wextra -Werror -shared -fPIC -Isource/src source/src/homing/home_path.c -lm -o evidence/home-path.so
"$host_python" tools/check-upright-path.py
gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_home_still.c source/src/homing/home_still.c -lm -o evidence/test_home_still
./evidence/test_home_still
gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_home_stop.c source/src/homing/homing.c source/src/homing/home_path.c source/src/homing/home_still.c source/src/homing/gravity.c source/src/devices/can_codec.c -lm -o evidence/test_home_stop
./evidence/test_home_stop
gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_soft_stop.c source/src/homing/homing.c source/src/homing/home_path.c source/src/homing/home_still.c source/src/homing/gravity.c -lm -o evidence/test_soft_stop
./evidence/test_soft_stop

gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_gello_codec.c source/src/gello/gello_codec.c -o evidence/test_gello_codec
./evidence/test_gello_codec
"$host_python" -m unittest discover -s source/host/examples -p test_board_gello.py -v
gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_gello_usb_desc.c source/src/gello/gello_usb_desc.c -o evidence/test_gello_usb_desc
./evidence/test_gello_usb_desc

gcc -std=c11 -Wall -Wextra -Werror -Isource/src source/tests/test_stopped_can.c source/src/devices/stopped_can.c -o evidence/test_stopped_can
./evidence/test_stopped_can
