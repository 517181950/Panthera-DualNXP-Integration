#!/usr/bin/env python3
"""Synthetic ideal-tracking replay. No SDK, sockets, serial, or robot imports."""
import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'source/host'))
from soft_limits import Limits, SoftLimitGovernor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'evidence/soft-limit-replay')
    parser.add_argument('--plot', action='store_true', help='Also render PNG (requires matplotlib)')
    args = parser.parse_args()
    c = Limits(**json.loads((ROOT/'soft-limits.json').read_text()))
    governor = SoftLimitGovernor(c)
    q, v = c.center.copy(), np.zeros(6)
    records = []; runtimes = []; previous_v = np.zeros(6); max_accel = np.zeros(6)
    dt = .01
    for step in range(9001):
        now = step*dt
        target = c.upper+np.deg2rad(30) if now < 30 else c.lower-np.deg2rad(30) if now < 60 else c.center
        sample = dict(q_rad=q, dq_rad_s=v, feedback_age_us=[1000]*6,
                      fault_code_raw=[0]*6, online_mask=63, valid_q_mask=63,
                      valid_dq_mask=63, motor_fault_valid_mask=63,
                      host_received_monotonic_s=now)
        start = time.perf_counter_ns()
        q, v, diag = governor.update(target, sample, now)
        runtimes.append((time.perf_counter_ns()-start)/1000)
        max_accel = np.maximum(max_accel, np.abs(v-previous_v)/dt)
        previous_v = v.copy()
        records.append([now, diag['min_barrier'], int(diag['target_clipped'])]+
                       target.tolist()+q.tolist()+v.tolist())
    data = np.asarray(records)
    qref = data[:, 9:15]; velocity = data[:, 15:21]
    assert np.all(qref >= c.lower-1e-9) and np.all(qref <= c.upper+1e-9)
    assert np.all(np.abs(velocity) <= c.speed+1e-9)
    assert np.all(max_accel <= c.acceleration+1e-9)
    assert np.min(data[:, 1]) >= -1e-9
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output/'trace.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t', 'min_barrier', 'clipped']+
                        [f'{name}{i}' for name in ('requested_q', 'sent_q', 'sent_dq') for i in range(1, 7)])
        writer.writerows(records)
    summary = dict(synthetic_ideal_tracking=True, hardware_test=False, seconds=90,
                   sample_period_s=dt, samples=len(records), min_barrier=float(data[:, 1].min()),
                   min_position_deg=np.rad2deg(qref.min(axis=0)).tolist(),
                   max_position_deg=np.rad2deg(qref.max(axis=0)).tolist(),
                   max_speed_deg_s=np.rad2deg(np.abs(velocity).max(axis=0)).tolist(),
                   max_acceleration_deg_s2=np.rad2deg(max_accel).tolist(),
                   governor_runtime_us=dict(mean=float(np.mean(runtimes)),
                                            p99=float(np.percentile(runtimes, 99)), max=float(max(runtimes))),
                   runtime_note='One six-axis governor only; excludes transport, gravity, logging and OS deadline guarantees.')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    if not args.plot:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(data[:, 0], np.rad2deg(data[:, 3]), '--', color='gray', label='GELLO request, J1')
    axes[0].plot(data[:, 0], np.rad2deg(qref[:, 0]), label='Filtered reference, J1')
    for bound in (c.lower_deg[0], c.upper_deg[0]): axes[0].axhline(bound, color='red', linestyle=':', linewidth=1)
    axes[0].set_ylabel('Position (deg)'); axes[0].legend(loc='upper right')
    axes[1].plot(data[:, 0], np.rad2deg(velocity[:, 0]), label='J1 reference velocity')
    axes[1].set_ylabel('Velocity (deg/s)'); axes[1].legend(loc='upper right')
    axes[2].plot(data[:, 0], data[:, 1], label='Minimum NCBF over six references')
    axes[2].axhline(0, color='red', linestyle=':'); axes[2].set_ylabel('Barrier h'); axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right')
    for axis in axes: axis.grid(alpha=.25)
    fig.suptitle('Soft-limit replay: synthetic ideal tracking, no hardware')
    fig.tight_layout(); fig.savefig(args.output/'replay.png', dpi=160); plt.close(fig)


if __name__ == '__main__': main()
