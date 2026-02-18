import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_amr_nav = get_package_share_directory('amr_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_amr_nav, 'config', 'nav2_params.yaml'))
    
    # We use a mocked map for now, but in reality this would load a yaml
    # Since mock_robot publishes /map, we might skip the map_server 
    # But usually Nav2 needs a static map server.
    # For now, we assume map_server is NOT needed because mock_robot publishes it with Transient Local QoS
    # IF we wanted to load from file:
    # map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_amr_nav, 'maps', 'warehouse.yaml'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot_1',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_amr_nav, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        # Bringup Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_namespace': 'True',
                'map': '', # We use map from topic or skip map server
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'True', # Compose nodes for performance
                'use_respawn': 'False'
            }.items(),
        ),
    ])
