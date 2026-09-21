import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_articubot = get_package_share_directory('articubot_one')

    robots = [
        {'name': 'bot1', 'x': '0.0', 'y': '0.0', 'z': '0.1'},
        {'name': 'bot2', 'x': '2.0', 'y': '1.0', 'z': '0.1'},
        {'name': 'bot3', 'x': '-2.0', 'y': '-1.0', 'z': '0.1'}
    ]

    launch_actions = []

    for bot in robots:
        robot_group = GroupAction([
            PushRosNamespace(bot['name']),
            # 1. Robot State Publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_articubot, 'launch', 'rsp.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            ),
            # 2. Gazebo Spawner
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_articubot, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={
                    'robot_name': bot['name'],
                    'x_pose': bot['x'],
                    'y_pose': bot['y'],
                    'z_pose': bot['z']
                }.items()
            )
        ])
        launch_actions.append(robot_group)

    return LaunchDescription(launch_actions)
