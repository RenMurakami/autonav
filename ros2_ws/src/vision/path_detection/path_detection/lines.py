################################
# AutoNav 2022 Competition Robot
# Package: lines
# File: lines.py
# Purpose: controls line detection and line following
# Author: Modified from 2020-21 autonav team code for ROS2
# Date Modified: 11 May 2022
################################

# !/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils.utils import *

from .line_detection import LineDetection
from .line_following import LineFollowing
from std_msgs.msg import Int32
from std_msgs.msg import String

import cv2

class Lines(Node):
    def __init__(self):
        super().__init__('lines')

        self.count = 0

        self.state = STATE.LINE_FOLLOWING
        self.LINE_CODE = "LIN,"

        # Subscribe to the camera color image
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)

        # Publish events that could change the robot state
        self.event_pub = self.create_publisher(String, "line_events", 10)

        # Publish distance from the center of the line
        self.motor_pub = self.create_publisher(String, "wheel_distance", 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)

        # Read ROS Params - Line Detection
        self.declare_parameter('/FollowingDirection', DIRECTION.LEFT)
        self.declare_parameter('/LineDetectBufferSize', 5)
        self.declare_parameter('/LineDetectBufferFill', 0.6)
        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineApproachCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)
        self.declare_parameter('/LineDetectMaxWhite', 0.5)
        # commented if statement out for testing
        if self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT:
            self.declare_parameter('/LineDetectMinSlope', 1.25)
        else:
            self.declare_parameter('/LineDetectMinSlope', -1.25)
        self.declare_parameter('/LineDetectMinLineLength', 0.35)
        self.declare_parameter('/LineDetectDistance', 1.25)
        self.declare_parameter('/Debug', True)
        self.declare_parameter('/UseYellow', False)

        self.get_logger().warning(f"STARTUP { self.get_parameter('/UseYellow').value,}")

        # Initialize Classes
        self.line_detection = LineDetection(
            self.get_parameter('/LineDetectBufferSize').value,
            self.get_parameter('/LineDetectBufferFill').value,
            self.get_parameter('/LineDetectCropTop').value,
            self.get_parameter('/LineApproachCropTop').value,
            self.get_parameter('/LineDetectCropBottom').value,
            self.get_parameter('/LineDetectCropSide').value,
            self.get_parameter('/LineDetectMaxWhite').value,
            self.get_parameter('/LineDetectMinSlope').value,
            self.get_parameter('/LineDetectMinLineLength').value,
            self.get_parameter('/LineDetectDistance').value,
            self.get_parameter('/Debug').value,
            self.get_parameter('/UseYellow').value
        )
        self.line_following = LineFollowing(
            self.get_parameter('/FollowingDirection').value,
            self.get_parameter('/UseYellow').value,
            self.get_parameter('/Debug').value
        )

        # Line Detection States Set
        self.line_detection_states = {STATE.OBJECT_AVOIDANCE_FROM_LINE, STATE.OBJECT_TO_LINE, STATE.FIND_LINE, STATE.LINE_ORIENT}

        # Set ROS Params

        self.get_logger().info("Waiting for image topics...")

    def image_callback(self, ros_image):
        self.count += 1
        image = bridge_image(ros_image, "bgr8")

        # Line Followings
        if self.state == STATE.LINE_FOLLOWING:
            self.line_detection.reset()
            close_windows(self.line_detection.window_handle)

            distance = self.line_following.image_callback(image)
            # self.get_logger().warning("LINE_DISTANCE: " + str(distance))
            msg = String()
            msg.data = self.LINE_CODE+str(distance)#+f"00{self.count}"
            # if self.count%2==1:
            # self.get_logger().info(f"sending {msg.data}")
            self.motor_pub.publish(msg)

        # Line Detection
        elif self.state in self.line_detection_states:
            found_line, aligned = self.line_detection.image_callback(image, self.state)
            # self.get_logger().info(f"Finding: {found_line}, {aligned}")
            close_windows(self.line_following.window_handle)
            # self.get_logger().warning("Line Detection")
            if found_line:
                # self.get_logger().warning(self.FOUND_LINE)
                msg = String()
                msg.data = STATUS.FOUND_LINE
                self.event_pub.publish(msg)
            if aligned:
                # self.get_logger().warning(self.ALIGNED)
                # self.line_following.reset()
                msg = String()
                msg.data = STATUS.ALIGNED
                self.event_pub.publish(msg)
        else:
            close_windows(self.line_following.window_handle)
            # self.get_logger().info("In bad state spot")
            self.line_detection.reset()

    def state_callback(self, new_state):
        self.get_logger().info(f"New State Received {new_state.data}")

        self.state = new_state.data


def main():
    rclpy.init()
    lines = Lines()

    try: rclpy.spin(lines)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Keyboard interrupt")


if __name__ == "__main__":
    main()
