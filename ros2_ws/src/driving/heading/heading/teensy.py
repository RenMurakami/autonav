################################
# AutoNav 2022 Competition Robot
# Package: encoders
# File: encoder_pub.py
# Purpose: interface with teensy to report wheel turning and
# Date Modified: 2 June 2022
# To run: ros2 run rs2l_transform transform
################################

from custom_msgs.msg import EncoderData
from custom_msgs.msg import LightCmd
from custom_msgs.msg import ImuData
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import serial
from time import sleep
from utils.utils import *

from .pid_controller import PIDController

ticks_per_meter = 1400


class Teensy(Node):
    def __init__(self):
        super().__init__('teensy')
        self.declare_parameter('/TeensyEncodersPort', '/dev/ttyACM1')
        self.declare_parameter('/TeensyBaudrate', 115200)
        self.declare_parameter('/TeensyUpdateDelay', .02)
        self.declare_parameter('/Debug', False)
        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/LineDist', 0.175)
        self.declare_parameter('/SideObjectDist', 0.6)
        self.declare_parameter('/LineSpeed', 33.0)
        self.declare_parameter('/ObjectSpeed', 33.0)
        self.declare_parameter('/GpsSpeed', 30.0)
        self.declare_parameter('/BoostIncrease', 1)
        self.declare_parameter('/BoostCountThreshold', 20)
        self.declare_parameter('/GPSBoostCountThreshold', 10)
        self.declare_parameter('/LineBoostMargin', 30.0)
        self.declare_parameter('/GPSBoostMargin', 0.1745)
        self.declare_parameter('/Port', '/dev/ttyUSB0')

        self.serialPort = serial.Serial(self.get_parameter('/TeensyEncodersPort').value,
                                        self.get_parameter('/TeensyBaudrate').value, timeout=0.01)
        self.serialPort.flushInput()
        self.serialPort.flushOutput()

        #  publish for right and left encoder distances
        self.rate = self.get_parameter('/TeensyUpdateDelay').value
        self.encoder_pub = self.create_publisher(EncoderData, 'encoder_data', 10)
        self.imu_pub = self.create_publisher(ImuData, 'imu_data', 10)
        self.timer = self.create_timer(self.rate, self.timer_callback)
        self.light_sub = self.create_subscription(LightCmd, "light_events", self.light_callback, 5)
        self.wheel_sub = self.create_subscription(String, "wheel_distance", self.wheel_callback, 10)
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)

        self.following_direction = self.get_parameter('/FollowingDirection').value
        self.target_line_dist = self.get_parameter('/LineDist').value*1000
        self.target_obj_dist = self.get_parameter('/SideObjectDist').value
        self.line_speed = self.get_parameter('/LineSpeed').value
        self.object_speed = self.get_parameter('/ObjectSpeed').value
        self.gps_speed = self.get_parameter('/GpsSpeed').value
        self.speed_boost = self.get_parameter('/BoostIncrease').value
        self.boost_count_threshold = self.get_parameter('/BoostCountThreshold').value
        self.gps_boost_count_threshold = self.get_parameter('/GPSBoostCountThreshold').value
        self.line_boost_margin = self.get_parameter('/LineBoostMargin').value
        self.gps_boost_margin = self.get_parameter('/GPSBoostMargin').value

        self.pid_line = PIDController(-0.025, 0.0, 0.0, 6, -6) # for line following
        self.pid_obj = PIDController(1.5, 0.0, 0.0, 8, -8)   # for object avoidance
        self.pid_gps = PIDController(2.0, 0, 0.0, 6, -6)  # for during gps navigation

        # encoder parameters
        self.unitChange = 1  # assuming passed in meters, need mm
        self.boost_count = 0
        self.past_left_ticks = 0
        self.past_right_ticks = 0

        # motor parameters
        self.curr_linear = 0
        self.curr_angular = 0
        self.toggle = False
        self.following_mode = FollowMode.eeLine
        self.STOP_LIMIT = 7777
        self.state = STATE.LINE_FOLLOWING
        self.MAX_CHANGE = 5
        self.MAX_ANGULAR_CHANGE = 5
        self.serialPort.write("M,89,89,**".encode())
        # self.get_logger().info("WAIT: Enable power to motors")
        # x = input("Hit enter when ready to proceed")
        # sleep(5)
        self.get_logger().info("Launching motors")

    def __del__(self):
        self.serialPort.write("M,89,89,**".encode())

    def state_callback(self, new_state):
        self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = int(new_state.data)

        # effects internal FollowMode
        if self.state == STATE.GPS_TO_OBJECT or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS or \
                self.state == STATE.LINE_TO_OBJECT or self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            self.following_mode = FollowMode.eeObject
            self.MAX_CHANGE = 4
        elif self.state == STATE.LINE_FOLLOWING:
            self.following_mode = FollowMode.eeLine
            self.boost_count = 0
            self.MAX_CHANGE = 4
            # self.get_logger().info("SWITCHED TO LINE FOLLOWING")
        elif self.state == STATE.GPS_NAVIGATION:
            self.following_mode = FollowMode.eeGps
            self.boost_count = 0
            self.MAX_CHANGE = 5
            # self.get_logger().info("SWITCHED TO GPS NAVIGATION")
        elif self.state == STATE.OBJECT_TO_LINE or self.state == STATE.FIND_LINE or \
                self.state == STATE.LINE_ORIENT or self.state == STATE.ORIENT_TO_GPS or self.state == STATE.GPS_EXIT:
            self.following_mode = FollowMode.eeTransition
            self.MAX_CHANGE = 2
            self.boost_count = 0

    def wheel_callback(self, msg):
        msg = msg.data
        linear = self.curr_linear
        angular = self.curr_angular

        stop_override = False
        message_valid = True

        if msg[:3] == CODE.TRANSITION_CODE:
            cmds = msg[4:].split(',')
            if len(cmds) != 2:
                self.get_logger().warning("ERROR: MISFORMATTED MESSAGE")
            else:
                linear = int(float(cmds[0]))
                angular = int(float(cmds[1]))
            # self.get_logger().info(f"FOLLOWING TRA with delta {angular}, speed {linear}")
        elif self.following_mode == FollowMode.eeLine and msg[:3] == CODE.LIN_SENDER:
            # self.get_logger().info(f"IN FOLLOW MODE: msg = {msg}")
            position = float(msg[4:])
            if position >= self.STOP_LIMIT:
                linear = 0
                angular = 0
                stop_override = True
                self.boost_count = 0
            else:
                position_error = self.target_line_dist - position
                #self.get_logger().warning(f"POSITION ERROR {position_error}")

                # Position error is negative if we need to turn toward the line
                # delta will come out negative if you need to turn toward the line

                delta = self.pid_line.control(position_error)
                delta = delta if self.following_direction == DIRECTION.RIGHT else -1 * delta
                # self.get_logger().warning(f"compensating by {delta}")
                linear = round(self.line_speed)
                angular = round(delta)
                #
                # // Check if we should in the acceptable zone for picking up speed.i
                if abs(position_error) <= self.line_boost_margin:
                    self.boost_count += 1
                else:
                    self.boost_count = 0
                if self.boost_count > self.boost_count_threshold and message_valid:
                    linear += self.speed_boost
        elif self.following_mode == FollowMode.eeObject and msg[:3] == CODE.OBJECT_SENDER:
            # self.get_logger().info(f"sending motor commands {msg}")
            position = float(msg[4:])
            # delta is negative if we need to go toward object
            delta = self.pid_obj.control(self.target_obj_dist - position)
            delta = delta if self.following_direction == DIRECTION.LEFT else -1 * delta
            linear = round(self.object_speed)
            angular = round(delta * 3/4)
            self.get_logger().info(f"FOLLOWING OBJECT with delta {delta}, speed {linear}")

        elif self.following_mode == FollowMode.eeGps and msg[:3] == CODE.GPS_SENDER:
            parts = msg.split(',')
            angle_error = float(parts[1])  # error angle
            gps_distance = float(parts[2])
            delta = self.pid_gps.control(angle_error)
            # GPS sends the error as - for left turns and + for right turns
            adjustment = 0.0
            if gps_distance <= 2.5:
                adjustment = 4*(2.5 - gps_distance)
                self.get_logger().info(f"adjusted linear by {adjustment}")

            linear = round(self.gps_speed - adjustment)
            angular = round(delta)
            if abs(angle_error*(gps_distance**(1./3))) <= self.gps_boost_margin:
                self.boost_count += 1
            else:
                self.boost_count = 0
            if self.boost_count > self.gps_boost_count_threshold and message_valid:
                linear += self.speed_boost

            self.get_logger().info(f"setting speeds: ({linear, angular})")

        else:
            message_valid = False



        # send speed command to wheels

        # self.get_logger().info(f"current speeds: ({self.curr_linear}, {self.curr_angular}); new speeds: ({linear, angular})")
        if message_valid and (linear != self.curr_linear or angular != self.curr_angular):
            if not stop_override:
                if linear > self.curr_linear + self.MAX_CHANGE:
                    linear = self.curr_linear + self.MAX_CHANGE
                elif linear < self.curr_linear - self.MAX_CHANGE:
                    linear = self.curr_linear - self.MAX_CHANGE
                if angular > self.curr_angular + self.MAX_ANGULAR_CHANGE:
                    angular = self.curr_angular + self.MAX_ANGULAR_CHANGE
                elif angular < self.curr_angular - self.MAX_ANGULAR_CHANGE:
                    angular = self.curr_angular - self.MAX_ANGULAR_CHANGE
            # self.get_logger().info(f"handling {x}")

            self.send_speed(linear+89, angular+89)
            self.get_logger().info(f"setting speeds: ({linear, angular})")

        self.curr_linear = linear
        self.curr_angular = angular

    def send_speed(self, linear, angular):
        # Get the lock before proceeding
        msg = f"M,{linear},{angular},**"
        self.serialPort.write(msg.encode())
        sleep(.02)

    def light_callback(self, msg):
        # Get the lock before proceeding
        # self.lock.acquire()
        # cmd = ''
        if msg.type == 'G' or msg.type == 'Y' or msg.type == 'B':
            cmd = f"{msg.type},{int(msg.on)},**"
            # try:
            self.serialPort.write(cmd.encode('utf-8'))
            # except serial.serialutil.SerialException:
            #     self.get_logger().warning("Serial Exception occurred")
            #     sleep(0.5)
            # except Exception as ex:
            #     template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            #     message = template.format(type(ex).__name__, ex.args)
            #     self.get_logger().warning(message)
            #     sleep(0.5)
            #     self.close()
        else:
            self.get_logger().info(f"Message type {msg.type} invalid")
        # self.lock.release()

    def timer_callback(self):
        # self.lock.acquire()
        try:
            self.serialPort.write('Q,**'.encode('utf-8'))
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            if data[0] == 'E' and len(data) == 4 and data[3] == '**\r\n':
                # find ticks since last change
                left_ticks = int(data[1]) - self.past_left_ticks
                right_ticks = int(data[2]) - self.past_right_ticks

                # update history
                self.past_left_ticks = int(data[1])
                self.past_right_ticks = int(data[2])

                left_dist = left_ticks / ticks_per_meter
                right_dist = right_ticks / ticks_per_meter

                # if self.get_parameter('/Debug').value:
                #     self.get_logger().info(f'left_dist {left_dist}')
                #     self.get_logger().info(f'right_dist {right_dist}')
            
                msg = EncoderData()
                msg.left = -left_dist
                msg.right = -right_dist
                self.encoder_pub.publish(msg)
            else:
                self.serialPort.flushInput()

            self.serialPort.write('I,**'.encode('utf-8'))
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            msg = ImuData()
            if data[0] == 'ABS' and len(data) == 5 and data[4] == '**\r\n':
                msg.abs_x = float(data[1])
                msg.abs_y = float(data[2])
                msg.abs_z = float(data[3])  
            else:
                self.serialPort.flushInput()
                return     
            
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            if data[0] == 'Euler' and len(data) == 5 and data[4] == '**\r\n':
                msg.euler_x = float(data[1])
                msg.euler_y = float(data[2])
                msg.euler_z = float(data[3])  
            else:
                self.serialPort.flushInput()
                return 
            
            read = self.serialPort.readline().decode('utf-8')
            data = read.split(',')
            if data[0] == 'Quaterion' and len(data) == 6 and data[5] == '**\r\n':
                msg.quat_w = float(data[1])
                msg.quat_x = float(data[2])
                msg.quat_y = float(data[3])  
                msg.quat_z = float(data[4])  
            else:
                self.serialPort.flushInput()
                return 
            self.imu_pub.publish(msg)    
        except serial.serialutil.SerialException:
            self.get_logger().info("encoder error in serial port")
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            self.get_logger().warning(message)
        # self.lock.release()


def main(args=None):
    rclpy.init()
    encoder_pub = Teensy()
    try:
        rclpy.spin(encoder_pub)
    except KeyboardInterrupt:
        pass
