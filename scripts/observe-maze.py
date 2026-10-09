#!/usr/bin/env python3
"""Observe a real ROS/Gazebo run; save sensors, odometry and ground truth.

Scenario coordinates are test instrumentation only. It sends no movement commands.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import struct
import signal
import subprocess
import threading
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image, LaserScan
from scenario_config import in_bounds, load_scenario


class MazeObserver(Node):
    def __init__(self, output, scenario, requested_rate, model):
        super().__init__('project1_maze_observer')
        self.output = output
        self.model = model  # Gazebo entity name of the robot to track.
        self.started = time.monotonic()
        self.scenario = scenario
        self.requested_rate = requested_rate
        self.stop_requested = False
        self.simulation_start = None
        self.simulation_end = None
        self.checkpoints_reached = []
        (output / 'scenario.json').write_text(json.dumps(scenario, indent=2) + '\n', encoding='utf-8')
        self.scan_count = 0
        self.image_count = 0
        self.image_size = None
        self.last_image = None
        self.last_scan = None
        self.path_count = 0
        self.initial_pose = None
        self.world_pose = None
        self.ground_truth_pose = None
        self.ground_truth_count = 0
        self.ground_truth_error = None
        self.reached_exit = False
        self.ready_written = False
        self.csv_file = (output / 'trajectory.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['elapsed_s', 'odom_x', 'odom_y', 'world_x', 'world_y'])
        self.ground_truth_file = (output / 'ground-truth.csv').open('w', newline='', encoding='utf-8')
        self.ground_truth_writer = csv.writer(self.ground_truth_file)
        self.ground_truth_writer.writerow(['elapsed_s', 'simulation_s', 'world_x', 'world_y'])
        # Read-only Gazebo instrumentation: never supplied to the drive controller.
        self.pose_process = subprocess.Popen(
            ['stdbuf', '-oL', 'gz', 'topic', '-e', '-t',
             f'/world/{scenario["world"]}/dynamic_pose/info', '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, start_new_session=True)
        self.pose_thread = threading.Thread(target=self.read_ground_truth, daemon=True)
        self.pose_thread.start()
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RosPath, '/wall_follower/path', self.path, path_qos)
        self.create_timer(10.0, self.progress)

    def read_ground_truth(self):
        previous_record = -1.0
        try:
            for line in self.pose_process.stdout:
                if not line.strip():
                    continue
                message = json.loads(line)
                for pose in message.get('pose', []):
                    if pose.get('name') != self.model:
                        continue
                    position = pose.get('position', {})
                    x, y = float(position.get('x', 0)), float(position.get('y', 0))
                    self.ground_truth_pose = (x, y)
                    self.ground_truth_count += 1
                    stamp = message.get('header', {}).get('stamp', {})
                    simulation_s = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) * 1e-9
                    if self.simulation_start is None:
                        self.simulation_start = simulation_s
                    self.simulation_end = simulation_s
                    next_checkpoint = len(self.checkpoints_reached)
                    checkpoints = self.scenario['checkpoints']
                    if self.ready_written and next_checkpoint < len(checkpoints):
                        checkpoint = checkpoints[next_checkpoint]
                        if in_bounds((x, y), checkpoint['bounds']):
                            self.checkpoints_reached.append(checkpoint['name'])
                    reached_exit = (self.ready_written
                                    and len(self.checkpoints_reached) == len(checkpoints)
                                    and in_bounds((x, y), self.scenario['exit']))
                    elapsed = time.monotonic() - self.started
                    if elapsed - previous_record >= 0.1 or reached_exit:
                        self.ground_truth_writer.writerow([elapsed, simulation_s, x, y])
                        previous_record = elapsed
                    # Latch success: later samples must not erase the exit event.
                    if reached_exit:
                        self.reached_exit = True
                        return
        except (ValueError, OSError) as error:
            self.ground_truth_error = str(error)

    def check_ready(self):
        at_start = self.ground_truth_pose is not None and math.hypot(
            self.ground_truth_pose[0] - self.scenario['spawn'][0],
            self.ground_truth_pose[1] - self.scenario['spawn'][1]) < 0.1
        if not self.ready_written and at_start and self.initial_pose is not None and self.scan_count > 0 and self.image_count > 0:
            ready = {'initial_odom_pose': self.initial_pose, 'camera_dimensions': self.image_size,
                     'laser_samples': len(self.last_scan.ranges), 'ground_truth_start': self.ground_truth_pose}
            (self.output / 'sensors-ready.json').write_text(json.dumps(ready) + '\n', encoding='utf-8')
            self.ready_written = True

    def progress(self):
        progress = {'elapsed_s': round(time.monotonic() - self.started, 1),
                    'ground_truth_pose': self.ground_truth_pose, 'laser_messages': self.scan_count,
                    'camera_messages': self.image_count, 'checkpoints': self.checkpoints_reached}
        (self.output / 'progress.json').write_text(json.dumps(progress) + '\n', encoding='utf-8')

    def scan(self, message):
        if message.ranges:
            self.scan_count += 1
            self.last_scan = message
            self.check_ready()

    def image(self, message):
        if message.data and message.width > 0 and message.height > 0:
            self.image_count += 1
            self.image_size = [message.width, message.height]
            self.last_image = message
            if self.image_count == 1:
                self.save_image(message, 'camera-start.png')
            self.check_ready()

    def save_image(self, message, filename):
        # Lossless format conversion of actual ROS pixels; no generated scene.
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
        if channels is None:
            return
        raw = bytearray()
        for row in range(message.height):
            raw.append(0)  # PNG's unfiltered scanline marker.
            start = row * message.step
            for column in range(message.width):
                pixel = start + column * channels
                rgb = bytes(message.data[pixel:pixel + 3])
                raw.extend(rgb[::-1] if message.encoding.startswith('bgr') else rgb)

        def chunk(kind, data):
            return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))

        header = struct.pack('!2I5B', message.width, message.height, 8, 2, 0, 0, 0)
        png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b'')
        (self.output / filename).write_bytes(png)

    def path(self, message):
        self.path_count = len(message.poses)

    def odometry(self, message):
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        if self.initial_pose is None:
            self.initial_pose = (x, y)
        # Translate wheel odometry for comparison only; it may drift significantly.
        spawn_x, spawn_y, yaw = self.scenario['spawn']
        dx, dy = x - self.initial_pose[0], y - self.initial_pose[1]
        world_x = spawn_x + dx * math.cos(yaw) - dy * math.sin(yaw)
        world_y = spawn_y + dx * math.sin(yaw) + dy * math.cos(yaw)
        self.world_pose = (world_x, world_y)
        self.check_ready()
        self.writer.writerow([time.monotonic() - self.started, x, y, world_x, world_y])

    def finish(self):
        try:
            os.killpg(self.pose_process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.pose_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.pose_process.pid, signal.SIGKILL)
            self.pose_process.wait()
        self.pose_thread.join(timeout=3)
        self.pose_process.stdout.close()
        self.ground_truth_file.close()
        self.csv_file.close()
        if self.last_image is not None:
            self.save_image(self.last_image, 'camera-final.png')
        if self.last_scan is not None:
            with (self.output / 'laser-final.csv').open('w', newline='', encoding='utf-8') as output:
                writer = csv.writer(output)
                writer.writerow(['angle_rad', 'range_m'])
                for index, distance in enumerate(self.last_scan.ranges):
                    angle = self.last_scan.angle_min + index * self.last_scan.angle_increment
                    writer.writerow([angle, distance if math.isfinite(distance) else ''])
        summary = {
            'scenario': self.scenario['name'],
            'world': self.scenario['world'],
            'elapsed_seconds': time.monotonic() - self.started,
            'simulation_seconds': (self.simulation_end - self.simulation_start
                                   if self.simulation_start is not None else None),
            'requested_real_time_factor': self.requested_rate,
            'ordered_checkpoints_reached': self.checkpoints_reached,
            'laser_messages': self.scan_count,
            'camera_messages': self.image_count,
            'camera_dimensions': self.image_size,
            'recorded_path_poses': self.path_count,
            'final_world_pose': self.ground_truth_pose,
            'odometry_inferred_final_pose': self.world_pose,
            'ground_truth_messages': self.ground_truth_count,
            'ground_truth_error': self.ground_truth_error,
            'maze_exit_reached': self.reached_exit,
            'note': 'Exit and ground-truth.csv use Gazebo model poses. trajectory.csv and the RViz path use drifting wheel odometry. The controller receives neither ground truth nor maze coordinates.',
        }
        passed = (self.reached_exit and self.ready_written and self.scan_count > 0
                  and self.image_count > 0 and self.path_count > 0 and self.ground_truth_error is None)
        summary['runtime_checks_passed'] = passed
        if summary['simulation_seconds'] is not None:
            summary['measured_average_real_time_factor'] = summary['simulation_seconds'] / summary['elapsed_seconds']
        (self.output / 'run-summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary, indent=2), flush=True)
        return passed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=600.0)
    parser.add_argument('--scenario', default='s_maze')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--model', default=os.environ.get('TURTLEBOT3_MODEL', 'burger_cam'),
                        help='Robot entity name in Gazebo; run-ros.sh sets TURTLEBOT3_MODEL')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MazeObserver(args.output, load_scenario(args.scenario), args.rate, args.model)
    def request_stop(signum, frame):
        node.stop_requested = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.reached_exit and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        passed = node.finish()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
