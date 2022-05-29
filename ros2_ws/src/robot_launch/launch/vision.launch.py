# LAUNCHES VISION ASPECT OF ROBOT

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os
from utils.utils import *


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_launch'),
        'config',
        'params.yml'
    )

    # VISION
    # create launch description with initial camera launch file
    ld = LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'))
        )]
    )
    # launch lidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidarNode",
        parameters=[
            config
        ]
    )
    # launch line following node
    lines_node = Node(
        package="path_detection",
        executable="lines",
        parameters=[
            config
        ]
    )
    # launch obstacle detection
    obstacles_node = Node(
        package="path_detection",
        executable="obstacles",
        parameters=[
            config
        ]
    )

    # vision
    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(obstacles_node)

    return ld
