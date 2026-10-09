# Project 1: Gazebo maze, sensor-equipped TurtleBot3, RViz and wall follower.
import os
import atexit
import json
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_scenario(context):
    package = get_package_share_directory('turtlebot3_gazebo')
    # The simulation model is chosen here, not from the user's TURTLEBOT3_MODEL,
    # which is set to burger for the real robot and has no simulated camera.
    model = LaunchConfiguration('model').perform(context)
    if not os.path.isfile(os.path.join(package, 'urdf', 'turtlebot3_' + model + '.urdf')):
        raise RuntimeError(f'Unknown TurtleBot3 model: {model}')
    os.environ['TURTLEBOT3_MODEL'] = model  # Read by spawn_turtlebot3.launch.py.
    gazebo = get_package_share_directory('ros_gz_sim')
    launch_dir = os.path.join(package, 'launch')
    with open(os.path.join(package, 'params', 'project1_scenarios.json'), encoding='utf-8') as source:
        scenarios = json.load(source)
    scenario_name = LaunchConfiguration('scenario').perform(context)
    if scenario_name not in scenarios:
        raise RuntimeError(f'Unknown scenario: {scenario_name}; choose {", ".join(scenarios)}')
    scenario = scenarios[scenario_name]
    world = os.path.join(package, 'worlds', scenario['world'] + '.world')
    gui_config = os.path.join(package, 'params', 'project1_gui.config')
    controller = LaunchConfiguration('controller')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items())
    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-g -v2 --gui-config ', gui_config], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))
    with open(os.path.join(package, 'urdf', 'turtlebot3_' + model + '.urdf'), encoding='utf-8') as source:
        robot_description = source.read()
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}])
    spawn_arguments = {'x_pose': str(scenario['spawn'][0]), 'y_pose': str(scenario['spawn'][1])}
    if LaunchConfiguration('fast').perform(context).lower() == 'true':
        # Keep physical geometry and laser unchanged; reduce camera rendering load.
        temporary = tempfile.TemporaryDirectory(prefix='mtrx3760-camera-')
        atexit.register(temporary.cleanup)
        robot = ET.parse(os.path.join(package, 'models', 'turtlebot3_' + model, 'model.sdf'))
        for sensor in robot.findall('.//sensor[@type="camera"]'):
            sensor.find('update_rate').text = '10'
            sensor.find('camera/image/width').text = '320'
            sensor.find('camera/image/height').text = '240'
        model_path = os.path.join(temporary.name, model + '.sdf')
        robot.write(model_path, encoding='utf-8', xml_declaration=True)
        spawn_arguments['model_sdf'] = model_path
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments=spawn_arguments.items())
    drive = Node(
        package='turtlebot3_gazebo', executable='turtlebot3_drive', output='screen',
        parameters=[{'use_sim_time': True, 'use_stamped_velocity': True}],
        condition=IfCondition(controller))
    display = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(package, 'rviz', 'project1.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz))

    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(package, 'models')),
        server, client, state_publisher,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[drive, display]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=os.environ.get('PROJECT1_SIM_MODEL', 'burger_cam'),
                              description='Simulated robot; burger_cam is a Burger with the lab camera'),
        DeclareLaunchArgument('scenario', default_value='s_maze',
                              description='s_maze, branched or open_track'),
        DeclareLaunchArgument('fast', default_value='false',
                              description='320x240 camera at 10 Hz; robot physics and LiDAR unchanged'),
        DeclareLaunchArgument('controller', default_value='false',
                              description='Start autonomous driving after checking the sensors'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_scenario),
    ])
