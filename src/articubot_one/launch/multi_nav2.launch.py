import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    articubot_dir = get_package_share_directory('articubot_one')

    nav2_params_file = os.path.join(articubot_dir, 'config', 'nav2_params.yaml')

    rovers = ['rover_1', 'rover_2', 'rover_3']
    ld = LaunchDescription()

    for rover in rovers:
        rover_nav = GroupAction(
            actions=[
                PushRosNamespace(rover),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': rover,
                        'use_namespace': 'True',
                        'params_file': nav2_params_file,
                        'autostart': 'True',
                        'use_sim_time': 'True',
                    }.items()
                )
            ]
        )
        ld.add_action(rover_nav)

    return ld
