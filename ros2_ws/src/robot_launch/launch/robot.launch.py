from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import math
import os
import sys
sys.path.insert(1, '/home/autonav/autonav/')
from utils import *


def generate_launch_description():
    following_dir = DIRECTION.RIGHT
    crop_top = 0.0
    crop_bottom = .2
    crop_side = .2

    # create launch description with initial camera launch file
    # disgusting but not quite sure how to make local path
    ld = LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        '/home/autonav/autonav/ros2_ws/src/vision/realsense-ros/realsense2_camera/launch/rs_launch.py'))]
    )

    # launch lidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidarNode",
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
        ]
    )

    # launch line following node
    lines_node = Node(
        package="lines",
        executable="lines",
        parameters=[
            {'/LineDetectCropTop':      crop_top},  # fraction to remove
            {'/LineDetectCropBottom':   crop_bottom},
            {'/LineDetectCropSide':     crop_side},
            {"/FollowingDirection":     following_dir},
            {'/Debug': False},
        ]
    )

    # launch rs2l_transform node
    transform_node = Node(
        package="rs2l_transform",
        executable="transform",
        parameters=[
            {'/LIDARTrimMin':           1.57},  # radians
            {'/LIDARTrimMax':           4.71},  # radians
            {'/ObstacleFOV':            math.pi / 6.0},  # radians
            {'/PotholeDetectCropTop':   crop_top},
            {'/PotholeDetectCropBottom':crop_bottom},
            {'/PotholeDetectCropSide':  crop_side},
            {'/PotholeBufferSize':      5},
            {"/Debug": False},
        ]
    )

    gps_node = Node(
        package="gps",
        executable="gps_publisher",
        parameters=[
            {'/WaypointLat1':   0.0},
            {'/WaypointLon1':   0.0},
            {'/WaypointLat2':   0.0},
            {'/WaypointLon2':   0.0},
            {'/WaypointLat3':   0.0},
            {'/WaypointLon3':   0.0},
            {'/WaypointLat4':   0.0},
            {'/WaypointLon4':   0.0},
            {'/ExitAngle':      0.2},   # radians
            {'GPSFollowGoal':   1.0},
            {'/LineToGPSTrans': 5.0},
            {'/Port':           '/dev/ttyACM0'},
        ]
    )

    encoder_node = Node(
        package="encoders",
        executable="encoder_pub",
        parameters=[
            {'/TeensyEncodersPort':     '/dev/ttyACM0'},
            {'/TeensyBaudrate':         115200},
            {'/TeensyUpdateDelay':      .01},
            {'/Debug':                  True},
        ]
    )

    motor_node = Node(
        package="wheels",
        executable="controller",
        parameters=[
            {'/FollowingDirection':     following_dir},
            {'/LineDist':               0.5},
            {'/SideObjectDist':         0.5},
            {'/DefaultSpeed':           15},
            {'/BoostIncrease':          2},
            {'/BoostCountThreshold':    20},
            {'/LineBoostMargin':        30.0},
            {'/GPSBoostMargin':         0.1745}
        ]
    )

    fsm_node = Node(
        package="master_fsm",
        executable="fsm",
        parameters=[
            {'/DefaultSpeed':       15},
            {'/FollowingDirection': following_dir},
            {'/TimerRate':          .05},
            {'/StartState':         STATE.LINE_FOLLOWING},
        ]
    )

    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(transform_node)
    ld.add_action(gps_node)
    ld.add_action(encoder_node)
    ld.add_action(motor_node)
    ld.add_action(fsm_node)

    # set debug
    if True:
        sub_node = Node(
            package="subscriber",
            executable="sub"
        )
        ld.add_action(sub_node)

    return ld
