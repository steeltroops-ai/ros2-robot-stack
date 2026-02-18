from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    # Path to the mock_robot.py script
    # Since we are running in dev mode without full install, we use relative path from workspace root
    # or absolute path. Assuming workspace root is /root/projects/ros2-robot-stack
    
    script_path = '/root/projects/ros2-robot-stack/robotics/ros2_ws/src/simulation_manager/src/mock_robot.py'

    return LaunchDescription([
        # Robot 1
        ExecuteProcess(
            cmd=['python3', script_path, '--ros-args', '-p', 'robot_id:=robot_1', '-p', 'initial_x:=2.0', '-p', 'initial_y:=2.0'],
            output='screen',
            emulate_tty=True
        ),
        
        # Robot 2
        ExecuteProcess(
            cmd=['python3', script_path, '--ros-args', '-p', 'robot_id:=robot_2', '-p', 'initial_x:=5.0', '-p', 'initial_y:=3.0'],
            output='screen',
            emulate_tty=True
        ),
        
        # Robot 3
        ExecuteProcess(
            cmd=['python3', script_path, '--ros-args', '-p', 'robot_id:=robot_3', '-p', 'initial_x:=8.0', '-p', 'initial_y:=5.0'],
            output='screen',
            emulate_tty=True
        )
    ])
