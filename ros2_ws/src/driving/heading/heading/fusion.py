################################
# AutoNav 2022 Competition Robot
# Package: heading
# File: fusion.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 22 May 2022
# Fuses encoder and gps headings
################################

# !/usr/bin/env python
import cmath
import math

import rclpy
from rclpy.node import Node
from utils.utils import *

from custom_msgs.msg import EncoderData
from custom_msgs.msg import ImuData
from custom_msgs.msg import HeadingStatus
from std_msgs.msg import String


class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')

        self.declare_parameter("/InitialHeading", 0.000)
        self.declare_parameter('/EncoderWeight', .5)
        self.declare_parameter('/ImuWeight', .5)
        self.declare_parameter('/Debug', False)
        self.declare_parameter('/ExitAngle', .2)

        self.exit_angle = self.get_parameter('/ExitAngle').value
        self.state = STATE.LINE_FOLLOWING
        self.curr_heading = self.get_parameter('/InitialHeading').value
        self.target_heading = 0.0
        self.encoder_curr_heading = degrees_to_radians(self.get_parameter('/InitialHeading').value)
        self.moving_avg = np.zeros((5,), dtype=np.float32)
        self.moving_avg_idx = 0
        self.distance_from_waypoint = 100.0

        self.encoder_weight = self.get_parameter('/EncoderWeight').value
        self.imu_weight = self.get_parameter('/ImuWeight').value

        self.encoder_sub = self.create_subscription(EncoderData, "encoder_data", self.enc_callback, 10)
        self.imu_sub = self.create_subscription(ImuData, "imu_data", self.imu_callback, 10)
        self.gps_heading_sub = self.create_subscription(HeadingStatus, "gps_heading", self.gps_callback, 10)
        self.wheel_pub = self.create_publisher(String, "wheel_distance", 10)
        self.fused_pub = self.create_publisher(HeadingStatus, "fused_heading", 10)

    def state_callback(self, new_state):
        self.state = new_state.data
        if self.state == STATE.ORIENT_TO_GPS:
            self.imu_weight = 0.5
            self.encoder_weight = 0.5
        else:
            self.imu_weight = self.get_parameter('/ImuWeight').value
            self.encoder_weight = self.get_parameter('/EncoderWeight').value

    # encoder must be sufficiently faster than GPS to be considered updated
    def enc_callback(self, enc_msg):
        # 64 cm gap between wheel centers
        if abs(enc_msg.left) < .5 and abs(enc_msg.right) < .5:
            self.encoder_curr_heading += (((enc_msg.left - enc_msg.right) / .64) % (math.pi * 2))
            if self.encoder_curr_heading > math.pi:
                self.encoder_curr_heading += -2*math.pi

        # calculate weighted current heading
        diff = sub_angles(self.curr_heading, self.encoder_curr_heading)
        self.curr_heading = (self.curr_heading - diff*self.encoder_weight) % (2*math.pi)
        if self.curr_heading > math.pi:
            self.curr_heading -= 2*math.pi

        # construct and publish heading message
        heading_msg = HeadingStatus()
        heading_msg.current_heading = self.curr_heading
        heading_msg.target_heading = self.target_heading
        heading_msg.distance = self.distance_from_waypoint
        self.fused_pub.publish(heading_msg)
        self.publish_to_motors()

        if self.get_parameter('/Debug').value:
            self.get_logger().info(f"Current heading from encoder callback: {heading_msg.current_heading}, target: {heading_msg.target_heading}")

    def imu_callback(self, imu_msg):
        try:
            # calculate IMU heading
            self.imu_curr_heading =  (imu_msg.euler_x - self.imu_initial_heading + self.get_parameter('/InitialHeading').value)*math.pi/180
            if self.imu_curr_heading > math.pi:
                self.imu_curr_heading -= 2*math.pi

            # calculate weighted current heading
            diff = sub_angles(self.curr_heading, self.imu_curr_heading)
            self.curr_heading = (self.curr_heading - diff*self.imu_weight) % (2*math.pi)
            if self.curr_heading > math.pi:
                self.curr_heading -= 2*math.pi
            
            # construct and publish heading message
            heading_msg = HeadingStatus()
            heading_msg.current_heading = self.curr_heading
            heading_msg.target_heading = self.target_heading
            heading_msg.distance = self.distance_from_waypoint
            self.fused_pub.publish(heading_msg)
            self.publish_to_motors()

            if self.get_parameter('/Debug').value:
                self.get_logger().info(f"Current heading from IMU callback: {heading_msg.current_heading}, target: {heading_msg.target_heading}")

        except:
            # we don't have an initial heading stored, so store the current message as
            # that initial heading
            self.imu_initial_heading = imu_msg.euler_x
            self.imu_curr_heading = imu_msg.euler_x - self.imu_initial_heading + self.get_parameter(
                '/InitialHeading').value

    # a 5 point moving average filter
    def filter_angle(self, new_val):
        self.moving_avg = np.roll(self.moving_avg, 1)
        self.moving_avg[0] = new_val
        return sum(self.moving_avg) / self.moving_avg.size

    # subtracts the angles (x - y) and gives the answer between (-pi, pi]
    # return true if we can switch back from obstacle avoidance to GPS nav
    def is_heading_restored(self, error_angle):
        return abs(error_angle) < self.exit_angle

    def publish_to_motors(self):
        # calculate and filter the error in the angle of the two headings
        error_angle = sub_angles(self.target_heading, self.curr_heading)
        msg = String()
        msg.data = CODE.GPS_SENDER + ',' + str(error_angle) + ',' + str(self.distance_from_waypoint)
        # self.get_logger().warning(f"SENDING GPS ERROR {error_angle}")
        self.wheel_pub.publish(msg)
        if self.get_parameter('/Debug').value:
            self.get_logger().warning("Error: " + str(error_angle))

    def gps_callback(self, gps_msg):
        if self.state == STATE.GPS_NAVIGATION and not -0.01 < gps_msg.current_heading < 0.01:
            self.encoder_weight = self.get_parameter('/EncoderWeight').value
        # elif self.state == STATE.LINE_FOLLOWING and not -0.01 < gps_msg.current_heading < 0.01:
        #     self.encoder_weight = 1.0
        # else:
        #     self.encoder_weight = 1.0
        # self.get_logger().info(f"GPS HEADING: {gps_msg}, encoder heading: {self.encoder_curr_heading}")
        self.target_heading = gps_msg.target_heading

        if self.distance_from_waypoint < 2:
            self.imu_weight = 0.9
            self.encoder_weight = 0.0

        # calculate weighted current heading
        if not -0.01 < gps_msg.current_heading < 0.01:
            diff = sub_angles(self.curr_heading, gps_msg.current_heading)
            self.curr_heading = (self.curr_heading - diff*(1 - self.encoder_weight - self.imu_weight)) % (2*math.pi)
            if self.curr_heading > math.pi:
                self.curr_heading -= 2*math.pi
            self.distance_from_waypoint = gps_msg.distance

            # construct and publish heading message
            heading_msg = HeadingStatus()
            heading_msg.current_heading = self.curr_heading
            heading_msg.target_heading = self.target_heading
            heading_msg.distance = self.distance_from_waypoint
            self.fused_pub.publish(heading_msg)
            self.publish_to_motors()

            if self.get_parameter('/Debug').value:
                self.get_logger().warning("Current GPS HEADING: " + str(gps_msg.current_heading) + '\n' +
                                          "Current Encoder Heading: " + str(self.encoder_curr_heading) + '\n' +
                                          "Current IMU Heading: " + str(self.imu_curr_heading) + '\n' +
                                          "Current Weighted Heading " + str(self.curr_heading) + '\n' +
                                          "Target Heading: " + str(gps_msg.target_heading) + '\n')


def main(args=None):
    rclpy.init(args=args)
    fuser = Fusion()

    try:
        rclpy.spin(fuser)
    except KeyboardInterrupt:
        # Destroy the node explicitly (optional)
        fuser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
