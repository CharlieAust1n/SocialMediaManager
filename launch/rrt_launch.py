# my_multi_package_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch file for the robot
    robot_launch_dir = get_package_share_directory('spatial3r_description')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_launch_dir, 'launch', 'no_js_gui.launch.py')
        )
    )

    # Node that actually executes the RMPPath trajectory
    move_on_path = Node(
        package='csc495_p2',
        executable='move_on_path',
        name='move_on_path',
        output='screen',
    )

    # RRTNode that runs the planner
    rrt = Node(
        package='rrt',
        executable='rrt',
        name='rrt',
        output='screen',
    )

    return LaunchDescription([robot_launch, move_on_path, rrt])
