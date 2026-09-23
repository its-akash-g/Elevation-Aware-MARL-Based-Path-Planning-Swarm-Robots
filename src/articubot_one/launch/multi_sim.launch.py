import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_articubot = get_package_share_directory('articubot_one')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    rovers = [
        {'name': 'rover_1', 'x': 0.0, 'y': 0.0, 'z': 0.1},
        {'name': 'rover_2', 'x': 2.0, 'y': 0.0, 'z': 0.1},
        {'name': 'rover_3', 'x': 4.0, 'y': 0.0, 'z': 0.1},
    ]

    nodes = [gazebo]
    xacro_file = os.path.join(pkg_articubot, 'description', 'robot.urdf.xacro')

    for rover in rovers:
        ns = rover['name']
        doc = xacro.process_file(xacro_file, mappings={'namespace': ns})
        robot_desc = doc.toxml()

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
                'frame_prefix': f'{ns}/'
            }]
        )

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace=ns,
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-entity', ns,
                '-x', str(rover['x']),
                '-y', str(rover['y']),
                '-z', str(rover['z'])
            ],
            output='screen'
        )

        jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad', '--controller-manager', f'/{ns}/controller_manager'],
            output='screen'
        )

        diff_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', '--controller-manager', f'/{ns}/controller_manager'],
            output='screen'
        )

        nodes.extend([rsp_node, spawn_node, jsb_spawner, diff_spawner])

    return LaunchDescription(nodes)
