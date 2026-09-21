import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Declare Launch Configurations
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # 2. Declare Arguments
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='bot1', description='Name of the robot entity in Gazebo'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace of the robot'
    )
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial X position')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.1', description='Initial Z position')

    # 3. Gazebo Entity Spawner Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', [namespace, '/robot_description'],
            '-entity', robot_name,
            '-robot_namespace', namespace,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_name,
        declare_namespace,
        declare_x,
        declare_y,
        declare_z,
        spawn_entity
    ])
