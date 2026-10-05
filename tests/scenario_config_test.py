"""Metadata/geometry checks independent of ROS, not evidence of robot traversal."""
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from scenario_config import CONFIG, in_bounds, load_scenario


class ScenarioTests(unittest.TestCase):
    def test_all_worlds_match_metadata(self):
        for name in json.loads(CONFIG.read_text(encoding='utf-8')):
            with self.subTest(scenario=name):
                config = load_scenario(name)
                world_file = CONFIG.parents[1] / 'worlds' / (config['world'] + '.world')
                world = ET.parse(world_file).find('world')
                self.assertEqual(world.get('name'), config['world'])
                self.assertFalse(in_bounds(config['spawn'], config['exit']))
                self.assertEqual(config['spawn'][2], 0.0)
                names = [checkpoint['name'] for checkpoint in config['checkpoints']]
                self.assertEqual(len(names), len(set(names)))
                for bounds in [config['exit'], config['plot_bounds']] + [c['bounds'] for c in config['checkpoints']]:
                    self.assertLess(bounds[0], bounds[1])
                    self.assertLess(bounds[2], bounds[3])
                for model in world.findall('model'):
                    size = model.find('link/collision/geometry/box/size')
                    if size is None:
                        continue
                    pose = [float(v) for v in model.findtext('pose').split()]
                    dimensions = [float(v) for v in size.text.split()]
                    self.assertTrue(all(v > 0 for v in dimensions))
                    bounds = [pose[0] - dimensions[0] / 2, pose[0] + dimensions[0] / 2,
                              pose[1] - dimensions[1] / 2, pose[1] + dimensions[1] / 2]
                    self.assertFalse(in_bounds(config['spawn'], bounds), f'Spawn inside {model.get("name")}')

    def test_unknown_scenario_rejected(self):
        with self.assertRaises(ValueError):
            load_scenario('typo')

    def test_bounds(self):
        self.assertTrue(in_bounds((0.5, 0.5), [0, 1, 0, 1]))
        self.assertFalse(in_bounds((1.1, 0.5), [0, 1, 0, 1]))


if __name__ == '__main__':
    unittest.main()
