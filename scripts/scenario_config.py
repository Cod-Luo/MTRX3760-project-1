#!/usr/bin/env python3
"""Test-world metadata, deliberately separate from navigation code."""
import argparse
import json
import math
from pathlib import Path


CONFIG = Path(__file__).resolve().parents[1] / 'turtlebot3_gazebo/params/project1_scenarios.json'


def in_bounds(pose, bounds):
    return bounds[0] <= pose[0] <= bounds[1] and bounds[2] <= pose[1] <= bounds[3]


def load_scenario(name):
    scenarios = json.loads(CONFIG.read_text(encoding='utf-8'))
    if name not in scenarios:
        raise ValueError(f'Unknown scenario {name!r}; choose from {", ".join(scenarios)}')
    scenario = scenarios[name]
    numbers = scenario['spawn'] + scenario['exit'] + scenario['plot_bounds']
    for checkpoint in scenario['checkpoints']:
        numbers += checkpoint['bounds']
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError('Non-finite scenario coordinates')
    scenario['name'] = name
    return scenario


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario')
    parser.add_argument('field', nargs='?', default='world')
    args = parser.parse_args()
    value = load_scenario(args.scenario)[args.field]
    print(value if isinstance(value, str) else json.dumps(value))


if __name__ == '__main__':
    main()
