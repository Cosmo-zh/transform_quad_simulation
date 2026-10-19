#!/usr/bin/env python3
"""Summarize a stereo run and plot independent truth and measured arm motion."""
import argparse
import csv
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/transform-stereo-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_records(path):
    if not path.exists():
        return []
    records = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            print(f'Skipped incomplete record: {path.name}:{number}')
    return records


def write_csv(path, fields, rows):
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or args.run / 'analysis'
    output.mkdir(parents=True, exist_ok=True)
    events = read_records(args.run / 'events.jsonl')
    truth = read_records(args.run / 'ground_truth.jsonl')
    resources = read_records(args.run / 'resources.jsonl')
    telemetry = [e for e in events if e['event'] == 'telemetry' and e['attitude'] is not None]
    plans = [e for e in events if e['event'] == 'plan']
    summary = json.loads((args.run / 'result.json').read_text())
    vision_stats = args.run / 'vision.json'
    if vision_stats.exists():
        summary['vision'] = json.loads(vision_stats.read_text())
    if resources:
        summary['minimum_available_memory_mb'] = min(r['available_memory_mb'] for r in resources)
    positions = np.array([r['position'] for r in truth]).reshape(-1, 3)
    summary.update(source=str(args.run.resolve()), plans=plans,
                   samples=dict(measurements=sum(e['event'] == 'measurement' for e in events),
                                telemetry=len(telemetry), ground_truth=len(truth)),
                   max_abs_roll_pitch_deg=([math.degrees(float(v)) for v in
                       np.max(np.abs(np.array([e['attitude'][:2] for e in telemetry])), axis=0)] if telemetry else None),
                   closest_gate_centers=([min(truth, key=lambda r: abs(r['position'][0]-x))['position']
                                         for x in [3, 7]] if truth else []))
    (output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    write_csv(output / 'measurements.csv', ['time', 'stamp', 'state', 'width', 'height', 'forward',
                                          'right', 'down', 'normal_yaw', 'lateral_error', 'uncertainty', 'full_height'],
              (e for e in events if e['event'] == 'measurement'))
    flattened = []
    for event in telemetry:
        row = dict(time=event['time'], state=event['state'], gate_contacts=event.get('gate_contacts'))
        for key, names in [('position', ['north', 'east', 'down']), ('velocity', ['vn', 've', 'vd']),
                           ('attitude', ['roll', 'pitch', 'yaw']),
                           ('joints', [f'arm{i}_rad' for i in range(1, 5)]),
                           ('servo_commands', [f'cmd{i}_rad' for i in range(1, 5)])]:
            row.update(zip(names, event.get(key) or []))
        flattened.append(row)
    write_csv(output / 'telemetry.csv', ['time', 'state', 'north', 'east', 'down', 'vn', 've', 'vd',
                                      'roll', 'pitch', 'yaw', 'arm1_rad', 'arm2_rad', 'arm3_rad', 'arm4_rad',
                                      'cmd1_rad', 'cmd2_rad', 'cmd3_rad', 'cmd4_rad',
                                      'gate_contacts'], flattened)
    write_csv(output / 'ground_truth.csv', ['stamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'],
              (dict(zip(['stamp', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'],
                        [r['stamp'], *r['position'], *r['quaternion']])) for r in truth))

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    axes[0].plot(positions[:, 0], positions[:, 1], label='Gazebo body center')
    axes[1].plot(positions[:, 0], positions[:, 2], label='Gazebo body center')
    if not len(positions):
        for ax in axes[:2]:
            ax.text(.5, .5, 'No ground-truth samples', transform=ax.transAxes, ha='center')
    for x, width in [(3, .6), (7, .44)]:
        for low, high in [(-.4, -width/2), (width/2, .4)]:
            axes[0].fill_between([x-.125, x+.125], low, high, color='gray', alpha=.5)
        for low, high in [(0, 1.25), (1.95, 2.2)]:
            axes[1].fill_between([x-.125, x+.125], low, high, color='gray', alpha=.5)
    axes[0].set(xlabel='World X (m)', ylabel='World Y (m)', ylim=(-.4, .4),
                title='Independent truth trajectory (body center; disk contacts checked separately)')
    axes[1].set(xlabel='World X (m)', ylabel='World Z (m)', ylim=(0, 2.2))
    joints = [e for e in telemetry if e['joints'] is not None]
    for i in range(4):
        axes[2].plot([e['time'] for e in joints], [-e['joints'][i]/(math.pi/3) for e in joints],
                     linewidth=.8, label=f'Arm {i+1} measured')
    for plan in plans:
        axes[2].scatter([plan['time']], [plan['progress']], marker='x', color='black')
    axes[2].set(xlabel='Companion elapsed time (s)', ylabel='Fold progress', ylim=(-.08, 1.05))
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(loc='best')
    fig.savefig(output / 'trajectory.png', dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
