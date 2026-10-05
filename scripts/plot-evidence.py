#!/usr/bin/env python3
"""Plot recorded Gazebo ground truth and odometry against the supplied maze."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from scenario_config import in_bounds, load_scenario


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads((args.run / 'run-summary.json').read_text(encoding='utf-8'))
    scenario_file = args.run / 'scenario.json'
    scenario = (json.loads(scenario_file.read_text(encoding='utf-8')) if scenario_file.exists()
                else load_scenario(summary.get('scenario', 's_maze')))
    with (args.run / 'trajectory.csv').open(newline='', encoding='utf-8') as source:
        odometry = list(csv.DictReader(source))
    with (args.run / 'ground-truth.csv').open(newline='', encoding='utf-8') as source:
        poses = list(csv.DictReader(source))
    if not poses:
        raise RuntimeError('The run has no recorded ground-truth poses')
    world = Path(__file__).resolve().parents[1] / 'turtlebot3_gazebo/worlds' / (scenario['world'] + '.world')
    figure = Figure(figsize=(10, 8), constrained_layout=True)
    FigureCanvasAgg(figure)
    axes = figure.add_subplot()
    walls = []
    for model in ET.parse(world).findall('.//world/model'):
        size = model.find('link/collision/geometry/box/size')
        if size is not None:
            pose = [float(value) for value in model.findtext('pose').split()]
            dimensions = [float(value) for value in size.text.split()]
            walls.append((pose[0], pose[1], dimensions[0] / 2, dimensions[1] / 2))
            axes.add_patch(Rectangle((pose[0] - dimensions[0] / 2, pose[1] - dimensions[1] / 2),
                dimensions[0], dimensions[1], facecolor='#64748b', edgecolor='#334155'))
    x = [float(pose['world_x']) for pose in poses]
    y = [float(pose['world_y']) for pose in poses]
    minimum_clearance = min(math.hypot(max(abs(px - cx) - hx, 0),
                                     max(abs(py - cy) - hy, 0))
                            for px, py in zip(x, y) for cx, cy, hx, hy in walls)
    checks = {
        'ground_truth_samples': len(poses),
        'minimum_robot_origin_to_wall_surface_m': minimum_clearance,
        'scenario': scenario['name'],
        'ground_truth_exit_reached': summary['maze_exit_reached'],
        'note': 'Clearance is for the model origin, not the complete robot footprint; no collision-free-body claim is inferred from this alone.',
    }
    checkpoint_index = 0
    for pose in zip(x, y):
        if checkpoint_index < len(scenario['checkpoints']) and in_bounds(
                pose, scenario['checkpoints'][checkpoint_index]['bounds']):
            checkpoint_index += 1
    checks['ordered_checkpoints_visited'] = [checkpoint['name'] for checkpoint in scenario['checkpoints'][:checkpoint_index]]
    checks['all_ordered_checkpoints_visited'] = checkpoint_index == len(scenario['checkpoints'])
    (args.run / 'trajectory-checks.json').write_text(json.dumps(checks, indent=2) + '\n', encoding='utf-8')
    axes.plot(x, y, color='#2563eb', linewidth=2, label='Recorded Gazebo model position')
    axes.plot([float(pose['world_x']) for pose in odometry],
              [float(pose['world_y']) for pose in odometry], color='#d97706',
              linestyle='--', linewidth=1, alpha=0.7, label='Wheel odometry (shows drift)')
    axes.scatter(x[0], y[0], color='#15803d', s=70, zorder=4, label='Start')
    axes.scatter(x[-1], y[-1], color='#be123c', s=70, zorder=4, label='Final recorded pose')
    bounds = scenario['plot_bounds']
    # Include drifting odometry too; do not silently clip the comparison curve.
    comparison_x = [float(pose['world_x']) for pose in odometry]
    comparison_y = [float(pose['world_y']) for pose in odometry]
    axes.set(xlim=(min(bounds[0], min(x + comparison_x) - 0.2), max(bounds[1], max(x + comparison_x) + 0.2)),
             ylim=(min(bounds[2], min(y + comparison_y) - 0.2), max(bounds[3], max(y + comparison_y) + 0.2)),
             aspect='equal', xlabel='World x (m)', ylabel='World y (m)')
    status = 'Exit reached' if summary['maze_exit_reached'] else 'Diagnostic run: exit not reached'
    axes.set_title(f"{status}: {scenario['label']}\nSensor-driven right-wall following; ground truth used only for observation")
    axes.grid(alpha=0.15)
    axes.legend(loc='upper left', fontsize=9)
    output = args.run / 'trajectory.png'
    figure.savefig(output, dpi=160)
    print(output)
    with (args.run / 'laser-final.csv').open(newline='', encoding='utf-8') as source:
        laser = [sample for sample in csv.DictReader(source) if sample['range_m']]
    laser_figure = Figure(figsize=(6, 6), constrained_layout=True)
    FigureCanvasAgg(laser_figure)
    laser_axes = laser_figure.add_subplot()
    laser_axes.scatter([float(sample['range_m']) * math.cos(float(sample['angle_rad'])) for sample in laser],
                       [float(sample['range_m']) * math.sin(float(sample['angle_rad'])) for sample in laser],
                       s=10, color='#dc2626', label='Actual finite LaserScan returns')
    laser_axes.scatter([0], [0], color='#2563eb', label='LiDAR origin')
    laser_axes.annotate('Forward', xy=(0.7, 0), xytext=(0, 0.3),
                        arrowprops={'arrowstyle': '->', 'color': '#2563eb'})
    laser_axes.set(xlim=(-3.5, 3.5), ylim=(-3.5, 3.5), aspect='equal',
                   xlabel='Sensor x (m): forward', ylabel='Sensor y (m): left',
                   title='Recorded final LiDAR scan\nNo-return / non-finite samples omitted')
    laser_axes.grid(alpha=0.2)
    laser_axes.legend(loc='upper right', fontsize=8)
    laser_output = args.run / 'laser-final.png'
    laser_figure.savefig(laser_output, dpi=160)
    print(laser_output)


if __name__ == '__main__':
    main()
