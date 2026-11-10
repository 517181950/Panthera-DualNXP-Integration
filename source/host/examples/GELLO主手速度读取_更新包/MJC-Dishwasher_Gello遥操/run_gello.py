"""Control both dishwasher arms from one serial Gello reader and Tk tuner."""

from __future__ import annotations

import argparse
import multiprocessing as mp
import os
from pathlib import Path
from queue import Empty, Full
import sys
import time
import tkinter as tk

# The native MuJoCo viewer uses GLFW.  This must be set before importing
# dishwasher_sim, which imports mujoco.
os.environ["MUJOCO_GL"] = "glfw"

from dishwasher_sim import DishwasherSim
from gello import (
    GelloConfig,
    GroupSnapshot,
    ReaderSnapshot,
    load_config,
    map_group_to_targets,
)
from gello.tk_app import GelloTunerApp
from gpu_check import require_hardware_opengl


DEFAULT_CONFIG = Path(__file__).with_name("gello_config.json")
GROUP_CONFIGS = ((0, "left"), (1, "right"))


def apply_snapshot(
    sim: DishwasherSim,
    snapshot: ReaderSnapshot,
    config: GelloConfig,
) -> tuple[str, ...]:
    """Apply every complete group in a reader snapshot to its MuJoCo arm."""
    applied: list[str] = []
    for group_index, config_name in GROUP_CONFIGS:
        group = snapshot.group(group_index)
        if group is None or not group.data_valid:
            continue
        side = (
            ("right" if group_index == 0 else "left")
            if config.swap_arms
            else config_name
        )
        arm_config = config.left if group_index == 0 else config.right
        targets = map_group_to_targets(
            group, arm_config, DishwasherSim.INITIAL_ARM_ANGLES
        )
        sim.set_arm_angles(side, targets.joints)
        sim.set_gripper(side, targets.gripper)
        applied.append(side)
    return tuple(applied)


def _smoke_snapshot() -> ReaderSnapshot:
    now = time.monotonic()
    groups = tuple(
        GroupSnapshot(
            group=group,
            servo_ids=tuple(range(1 + group * 10, 8 + group * 10)),
            raw_positions=(2048,) * 7,
            unwrapped_ticks=(2048,) * 7,
            arm_radians=(0.05, 0.05, 0.04, -0.04, 0.03, -0.03),
            gripper_radians=0.0,
            gripper_normalized=0.5,
            gripper_calib_min_raw=0,
            gripper_calib_max_raw=4095,
            gripper_calib_active=False,
            gripper_calib_done=True,
            zeroed=True,
            data_valid=True,
            captured_at=now,
        )
        for group in range(2)
    )
    return ReaderSnapshot(groups, now, sequence=1, scan_duration_s=0.0)


def run_smoke_test(config: GelloConfig) -> None:
    sim = DishwasherSim()
    snapshot = _smoke_snapshot()
    applied = apply_snapshot(sim, snapshot, config)
    sim.step(20)
    expected = ("right", "left") if config.swap_arms else ("left", "right")
    if applied != expected:
        raise RuntimeError(f"unexpected applied arms: {applied}")
    print("Gello to MuJoCo smoke test passed")
    if config.swap_arms:
        print("Applied groups: 0 -> right, 1 -> left")
    else:
        print("Applied groups: 0 -> left, 1 -> right")


def _mujoco_process(
    updates,
    stop_event,
    *,
    control_hz: float,
    status_interval: float,
    allow_software_rendering: bool,
) -> None:
    """Run GLFW and MuJoCo in a process isolated from Tk."""
    sim = DishwasherSim()
    require_hardware_opengl(allow_software=allow_software_rendering)
    period = 1.0 / control_hz
    timestep = float(sim.model.opt.timestep)
    physics_remainder = 0.0
    next_tick = time.monotonic()
    next_status = next_tick + status_interval
    last_sequence: int | None = None
    last_applied: tuple[str, ...] = ()
    pending: tuple[ReaderSnapshot, GelloConfig] | None = None

    with sim.launch_passive_viewer() as viewer:
        while viewer.is_running() and not stop_event.is_set():
            try:
                while True:
                    pending = updates.get_nowait()
            except Empty:
                pass

            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.005))
                continue

            physics_remainder += period
            physics_steps = int(physics_remainder / timestep)
            physics_remainder -= physics_steps * timestep

            with viewer.lock():
                if pending is not None:
                    snapshot, config = pending
                    last_applied = apply_snapshot(sim, snapshot, config)
                    last_sequence = snapshot.sequence
                    pending = None
                if physics_steps:
                    sim.step(physics_steps)
            viewer.sync()

            if status_interval > 0 and now >= next_status:
                sequence_text = "—" if last_sequence is None else str(last_sequence)
                arms_text = ",".join(last_applied) if last_applied else "—"
                print(f"seq={sequence_text} applied={arms_text}", flush=True)
                next_status = now + status_interval

            next_tick += period
            if next_tick <= now:
                next_tick = now + period

    # On the current Mesa/Wayland stack, MuJoCo 3.13 reaches this point and
    # then crashes in a GLFW process-exit destructor.  The viewer is isolated
    # in this process specifically so it can finish immediately after its
    # context manager has closed all live viewer resources.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def _publish_latest(updates, value: tuple[ReaderSnapshot, GelloConfig]) -> bool:
    """Replace a queued stale update so the viewer receives the newest pose."""
    try:
        updates.put_nowait(value)
        return True
    except Full:
        try:
            updates.get_nowait()
        except Empty:
            return False
        try:
            updates.put_nowait(value)
            return True
        except Full:
            return False


def run_application(args: argparse.Namespace, config: GelloConfig) -> None:
    context = mp.get_context("spawn")
    updates = context.Queue(maxsize=1)
    stop_event = context.Event()
    process = context.Process(
        target=_mujoco_process,
        kwargs={
            "updates": updates,
            "stop_event": stop_event,
            "control_hz": args.control_hz,
            "status_interval": args.status_interval,
            "allow_software_rendering": args.allow_software_rendering,
        },
        name="dishwasher-mujoco",
    )
    process.start()
    app: GelloTunerApp | None = None
    last_sequence: int | None = None

    try:
        root = tk.Tk()
        app = GelloTunerApp(root, args.config, config)
        print("Hold both Gello arms at the Panthera URDF zero pose, then click 连接.")
        print("Close either window or press Ctrl-C to stop.")

        while process.is_alive() and app.pump_events():
            fatal_error = app.fatal_error
            if fatal_error is not None:
                raise RuntimeError(
                    f"Gello serial thread stopped: {fatal_error}"
                ) from fatal_error
            snapshot = app.latest_snapshot()
            if snapshot is not None and snapshot.sequence != last_sequence:
                if _publish_latest(updates, (snapshot, app.active_config)):
                    last_sequence = snapshot.sequence
            time.sleep(0.005)
    finally:
        stop_event.set()
        if app is not None:
            app.close()
        process.join(timeout=5.0)
        if process.is_alive():
            process.terminate()
            process.join()
            raise RuntimeError("MuJoCo viewer process did not stop within 5 seconds")
        updates.close()
        updates.join_thread()

    if process.exitcode != 0:
        raise RuntimeError(f"MuJoCo viewer process exited with code {process.exitcode}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--control-hz", type=float, default=60.0)
    parser.add_argument(
        "--status-interval",
        type=float,
        default=1.0,
        help="seconds between control status lines; 0 disables them",
    )
    parser.add_argument("--allow-software-rendering", action="store_true")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="test snapshot mapping and physics without serial ports or windows",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.control_hz <= 0:
        raise SystemExit("--control-hz must be positive")
    if args.status_interval < 0:
        raise SystemExit("--status-interval must be non-negative")

    config = load_config(args.config)
    if args.smoke_test:
        run_smoke_test(config)
        return

    try:
        run_application(args, config)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
