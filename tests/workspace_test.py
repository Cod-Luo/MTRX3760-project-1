"""Regression checks for workspace selection; no ROS installation required."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'


class WorkspaceTests(unittest.TestCase):
    def selected_workspace(self, value=None):
        environment = os.environ.copy()
        environment.pop('PROJECT1_WORKSPACE', None)
        if value is not None:
            environment['PROJECT1_WORKSPACE'] = value
        return subprocess.check_output(
            ['bash', '-c', 'source "$1"; printf "%s" "$project_workspace"',
             'workspace-test', str(SCRIPTS / 'workspace.sh')],
            env=environment, text=True)

    def test_default_workspace(self):
        self.assertEqual(self.selected_workspace(), str(Path.home() / 'project1_ws'))

    def test_custom_workspace_including_spaces(self):
        self.assertEqual(self.selected_workspace('/tmp/project1 workspace'),
                         '/tmp/project1 workspace')

    def test_runtime_rejects_unbuilt_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            environment = os.environ.copy()
            environment['PROJECT1_WORKSPACE'] = directory
            result = subprocess.run(['bash', str(SCRIPTS / 'run-ros.sh'), 'true'],
                                    env=environment, text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Workspace is not built', result.stderr)

    def test_runtime_rejects_another_checkouts_build(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            (workspace / 'install').mkdir()
            (workspace / 'install/setup.bash').touch()
            cache = workspace / 'build/turtlebot3_gazebo/CMakeCache.txt'
            cache.parent.mkdir(parents=True)
            cache.write_text('CMAKE_HOME_DIRECTORY:INTERNAL=/another/checkout\n',
                             encoding='utf-8')
            environment = os.environ.copy()
            environment['PROJECT1_WORKSPACE'] = directory
            result = subprocess.run(['bash', str(SCRIPTS / 'run-ros.sh'), 'true'],
                                    env=environment, text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('another checkout', result.stderr)


if __name__ == '__main__':
    unittest.main()
