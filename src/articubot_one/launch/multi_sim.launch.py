import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_articubot = get_package_share_directory('articubot_one')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

   # Define world path
    world_path = os.path.join(pkg_articubot, 'worlds', 'rough_terrain.world')

    # Launch Gazebo World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + os.path.join(pkg_articubot, 'config', 'gazebo_params.yaml')
        }.items()
    )
    # Multi-rover pose configurations
    rovers = [
        {'name': 'rover_1', 'x': -2.0, 'y':  0.0, 'z': 0.1, 'yaw': 0.0},
        {'name': 'rover_2', 'x': -2.0, 'y':  1.5, 'z': 0.1, 'yaw': 0.0},
        {'name': 'rover_3', 'x': -2.0, 'y': -1.5, 'z': 0.1, 'yaw': 0.0},
    ]

    xacro_file = os.path.join(pkg_articubot, 'description', 'robot.urdf.xacro')
    launch_nodes = [gazebo]

    for rover in rovers:
        name = rover['name']
        x, y, z, yaw = str(rover['x']), str(rover['y']), str(rover['z']), str(rover['yaw'])

        # Process Xacro with rover namespace
        robot_description_config = Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' sim_mode:=true',
            ' namespace:=', name
        ])

        rover_group = GroupAction([
            PushRosNamespace(name),

            # Robot State Publisher with frame_prefix
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description_config,
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{name}/'
                }]
            ),

            # Spawn Entity in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{name}/robot_description',
                    '-entity', name,
                    '-robot_namespace', name,
                    '-x', x, '-y', y, '-z', z, '-Y', yaw
                ],
                output='screen'
            ),

            # Controller Spawner: Joint State Broadcaster
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad', '--controller-manager', f'/{name}/controller_manager'],
                output='screen'
            ),

            # Controller Spawner: Diff Drive Controller
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont', '--controller-manager', f'/{name}/controller_manager'],
                output='screen'
            ),
        ])

        launch_nodes.append(rover_group)

    return LaunchDescription(launch_nodes)
