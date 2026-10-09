"""ROS-only safety checks using synthetic scans; not physical/Gazebo evidence.

The Python harness is development tooling, not part of the assessed C++ controller.
It checks delivered commands, diagnostic transitions and both normal stop signals.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class RosNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Isolate synthetic sensor traffic from any running project simulation.
        os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'turtlebot3_gazebo'], text=True).strip()
        cls.executable = Path(prefix) / 'lib/turtlebot3_gazebo/turtlebot3_drive'
        rclpy.init()
        cls.node = rclpy.create_node('wall_follower_safety_test')
        cls.publisher = cls.node.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)
        cls.commands = []
        cls.subscription = cls.node.create_subscription(
            TwistStamped, 'cmd_vel', cls.commands.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.pump(0.1)
        self.commands.clear()
        self.directory = tempfile.TemporaryDirectory(prefix='project1-ros-test-')
        self.log_path = Path(self.directory.name) / 'controller.log'
        self.log_file = self.log_path.open('w', encoding='utf-8')
        self.process = None

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.log_file.close()
        self.directory.cleanup()

    def start(self, parameters=None):
        command = [str(self.executable)]
        if parameters:
            command += ['--ros-args'] + parameters
        self.process = subprocess.Popen(
            command, stdout=self.log_file, stderr=subprocess.STDOUT)
        self.wait_for(lambda: bool(self.commands), 'No initial stop received')
        self.assertNotIn('logging was initialized more than once', self.log())

    def log(self):
        return self.log_path.read_text(encoding='utf-8')

    def scan(self, kind):
        message = LaserScan()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angle_min = -math.pi
        message.angle_increment = 2.0 * math.pi / 360
        message.range_min = 0.12
        message.range_max = 3.5
        message.ranges = [float('inf')] * 360
        if kind == 'invalid':
            message.angle_increment = 0.0
        elif kind == 'future':
            message.header.stamp.sec += 10
        elif kind == 'nan':
            message.ranges = [float('nan')] * 360
        elif kind in ('blocked_missing_right', 'clear_missing_right'):
            front = 0.25 if kind == 'blocked_missing_right' else 0.6
            for index in range(160, 201):
                message.ranges[index] = front
            for index in range(80, 101):
                message.ranges[index] = float('nan')
        return message

    def pump(self, seconds, kind=None):
        deadline = time.monotonic() + seconds
        next_scan = 0.0
        while time.monotonic() < deadline:
            if kind is not None and time.monotonic() >= next_scan:
                self.publisher.publish(self.scan(kind))
                next_scan = time.monotonic() + 0.2  # Burger's 5 Hz cadence.
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_for(self, predicate, failure, kind=None, timeout=6):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if self.process is not None:
                self.assertIsNone(self.process.poll(), self.log())
            self.pump(0.05, kind)
        self.assertTrue(predicate(), failure + '\n' + self.log())

    def assert_stopped(self):
        self.assertTrue(self.commands, 'No command received')
        self.assertEqual(self.commands[-1].twist.linear.x, 0.0)
        self.assertEqual(self.commands[-1].twist.angular.z, 0.0)

    def assert_shutdown(self, stop_signal):
        previous_count = len(self.commands)
        self.process.send_signal(stop_signal)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and (
                self.process.poll() is None or len(self.commands) == previous_count):
            self.pump(0.05)
        self.assertEqual(self.process.poll(), 0, self.log())
        self.assertGreater(len(self.commands), previous_count, 'Final stop not received')
        self.assert_stopped()
        self.pump(0.15)
        self.assert_stopped()
        self.assertIn('Shutting down: stop command sent', self.log())

    def test_input_diagnostics_recovery_and_sigint(self):
        self.start()
        self.pump(0.7)
        self.assertTrue(all(command.twist.linear.x == 0.0
                            and command.twist.angular.z == 0.0
                            for command in self.commands))
        self.assertEqual(self.log().count('waiting for laser data'), 1)
        self.wait_for(lambda: self.commands[-1].twist.linear.x > 0,
                      'Valid scan did not enable motion', kind='valid')

        self.pump(0.9)
        self.assert_stopped()
        self.assertEqual(self.log().count('Laser data is stale; stopping'), 1)
        self.pump(0.3)
        self.assertEqual(self.log().count('Laser data is stale; stopping'), 1)

        self.pump(0.6, 'invalid')
        self.assertEqual(self.commands[-1].twist.linear.x, 0.0)
        self.assertLess(self.commands[-1].twist.angular.z, 0.0)
        self.assertEqual(self.log().count('Invalid laser readings or metadata'), 1)
        self.pump(2.5, 'nan')
        self.assertEqual(self.commands[-1].twist.linear.x, 0.0)
        self.assertLess(self.commands[-1].twist.angular.z, 0.0)
        self.assertEqual(self.log().count('Invalid laser readings or metadata'), 1)
        self.pump(0.9)
        self.assert_stopped()
        self.pump(0.6, 'future')
        self.assert_stopped()
        self.assertEqual(self.log().count('Invalid laser timestamp or scan age'), 1)
        self.wait_for(lambda: self.commands[-1].twist.linear.x > 0,
                      'Valid data did not restore motion', kind='valid')
        self.assertEqual(self.log().count('Usable laser data received; motion enabled'), 2)
        self.assert_shutdown(signal.SIGINT)

    def test_partial_scans_do_not_cancel_corner_turns(self):
        self.start()
        self.pump(0.8, 'blocked_missing_right')
        self.assertEqual(self.commands[-1].twist.linear.x, 0.0)
        self.assertGreater(self.commands[-1].twist.angular.z, 0.0)
        self.assertIn('front=valid, right=invalid', self.log())

        self.pump(0.8, 'nan')
        self.assertEqual(self.commands[-1].twist.linear.x, 0.0)
        self.assertGreater(self.commands[-1].twist.angular.z, 0.0)

        self.pump(0.8, 'clear_missing_right')
        self.assertAlmostEqual(self.commands[-1].twist.linear.x, 0.01)
        self.assertLess(self.commands[-1].twist.angular.z, 0.0)

        self.pump(0.8, 'valid')
        self.assertGreater(self.commands[-1].twist.linear.x, 0.01)
        self.assertLess(self.commands[-1].twist.angular.z, 0.0)
        self.assert_shutdown(signal.SIGTERM)

    def test_sigterm_delivers_stop(self):
        self.start()
        self.wait_for(lambda: self.commands[-1].twist.linear.x > 0,
                      'No motion before SIGTERM', kind='valid')
        self.assert_shutdown(signal.SIGTERM)

    def test_invalid_settings_keep_motion_disabled(self):
        self.start(['-p', 'wall_distance:=-1.0'])
        self.pump(0.8, 'valid')
        self.assertTrue(all(command.twist.linear.x == 0.0
                            and command.twist.angular.z == 0.0
                            for command in self.commands))
        self.assertEqual(self.log().count('Invalid wall-follower settings'), 1)
        self.assert_shutdown(signal.SIGTERM)


if __name__ == '__main__':
    unittest.main(verbosity=2)
