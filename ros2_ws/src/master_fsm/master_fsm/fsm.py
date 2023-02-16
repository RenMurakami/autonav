#!/usr/bin/env python3

################################
# AutoNav 2022 Competition Robot
# Package: master_fsm
# File: fsm.py
# Purpose: robot big brain based on 2021 brain
# Date Modified: 21 May 2022
################################
import math

import rclpy
from rclpy.node import Node
import sys
import threading
import time
from utils.utils import *
from std_msgs.msg import Int32
from std_msgs.msg import String
from custom_msgs.msg import HeadingStatus
from custom_msgs.msg import LightCmd


class MainRobot(Node):
    def __init__(self):
        super().__init__("fsm")

        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/TimerRate', .05)
        self.declare_parameter('/StartState', STATE.LINE_FOLLOWING)
        self.declare_parameter('/TurnSpeed', 20)
        self.declare_parameter('/SlightTurn', 18)
        self.declare_parameter('/ExitAngle', math.pi/8)
        self.declare_parameter('/GpsExitHeading', 0.0)
        self.declare_parameter('/CrossRampInGps', True)

        # Make a lock so the callbacks don't create race conditions
        self.lock = threading.Lock()

        # Publish to the wheels, lights, state changes
        self.wheel_pub = self.create_publisher(String, "wheel_distance", 10)
        self.lights_pub = self.create_publisher(LightCmd, "light_events", 10)
        self.state_pub = self.create_publisher(Int32, "state_topic", 100)

        # Subscribe to new event notifications from the lights node, gps, lidar
        self.line_sub = self.create_subscription(String, "line_events", self.line_callback, 10)
        self.gps_sub = self.create_subscription(String, "gps_events", self.gps_callback, 10)
        self.depth_sub = self.create_subscription(String, "/mod_lidar", self.lidar_callback, 10)
        self.heading_sub = self.create_subscription(HeadingStatus, 'fused_heading', self.heading_callback, 10)

        # already declared messages to save a couple lines
        self.state_msg = Int32()
        self.wheel_msg = String()

        self.state = self.get_parameter("/StartState").value
        self.state_msg.data = self.state
        self.state_pub.publish(self.state_msg)
        self.get_logger().info("Initializing Main Robot Controller...")

        self.transition_set = {STATE.LINE_TO_OBJECT, STATE.GPS_TO_OBJECT}

        self.obj_seen = False
        self.found_line = False
        self.aligned = False
        self.waypoint_found = False
        self.heading_restored = False
        self.path_clear = False
        self.follow_dir = self.get_parameter('/FollowingDirection').value
        self.TURN_SPEED = self.get_parameter('/TurnSpeed').value
        self.SLIGHT_TURN = self.get_parameter('/SlightTurn').value
        self.exit_angle = self.get_parameter('/ExitAngle').value
        self.heading = 0.0
        self.prev_heading = 0.0
        self.exit_heading = 0.0
        self.target_heading = 0.0
        self.look_for_line = False
        self.waypoints_done = False
        self.gps_exit_heading = self.get_parameter('/GpsExitHeading').value
        self.waypoint_count = 0

        # Make a timer object for calling the change state periodically
        self.timer = self.create_timer(self.get_parameter('/TimerRate').value, self.timer_callback)

    # Beginning of State Machine

    # Beginning of Major States

    # Line Following State
    def line_following_state(self):
        light_msg = LightCmd()
        light_msg.type = 'G'
        light_msg.on = False
        self.lights_pub.publish(light_msg)

        if self.waypoint_found:  # reached gps waypoint - switch to gps navigation
            self.waypoint_count += 1
            self.waypoint_found = False
            self.exit_heading = self.target_heading
            # self.state = STATE.ORIENT_TO_GPS
            # self.state_msg.data = STATE.ORIENT_TO_GPS
            # self.state_pub.publish(self.state_msg)
            # self.orient_to_gps_state()
            self.state = STATE.GPS_NAVIGATION
            self.state_msg.data = STATE.GPS_NAVIGATION
            self.state_pub.publish(self.state_msg)
            self.gps_navigation_state()

        elif self.obj_seen:  # object sighted - switch to obstacle avoidance
            # We check for an object second because if we have already hit the
            # GPS waypoint we want the robot to record that first.
            self.obj_seen = False
            self.state = STATE.LINE_TO_OBJECT
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)

            self.prev_heading = self.heading
            self.exit_heading = sub_angles(self.prev_heading, (1-2*int(self.follow_dir==DIRECTION.RIGHT))*self.exit_angle)

            self.get_logger().info(f"Current heading: {self.prev_heading}, exit heading: {self.exit_heading}")

            self.line_to_object_state()  # enter the transition state

    # Object Avoidance From Line Following State - trying to get back to line
    def object_avoidance_from_line_state(self):
        # self.get_logger().info("Object Avoidance From Line Following State")
        # Check for another object in front of the robot
        if self.waypoint_found:  # reached gps waypoint - switch to gps navigation
            self.waypoint_count += 1
            self.waypoint_found = False
            self.exit_heading = self.target_heading
            self.heading_restored = False
            self.state = STATE.OBJECT_AVOIDANCE_FROM_GPS
            self.state_msg.data = STATE.OBJECT_AVOIDANCE_FROM_GPS
            self.state_pub.publish(self.state_msg)
            self.object_avoidance_from_gps_state()

        elif self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_TO_OBJECT
            self.line_to_object_state()  # enter the transition state

        elif self.found_line and self.heading_restored:  # and self.look_for_line
            #light_msg = LightCmd()
            #light_msg.type = 'B'
            #light_msg.on = True
            #self.lights_pub.publish(light_msg)
            self.look_for_line = False
            self.found_line = False
            self.heading_restored = False
            self.state_msg.data = STATE.OBJECT_TO_LINE
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_TO_LINE
            self.object_to_line_state()  # enter the transition state

    # Object Avoidance From GPS Navigation State
    def object_avoidance_from_gps_state(self):
        # self.get_logger().info("Object Avoidance From GPS State")

        # Check for another object in front of the robot
        if self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = STATE.GPS_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_TO_OBJECT
            self.gps_to_object_state()  # enter the transition state

        elif self.heading_restored:  # Otherwise see if have a clear path to the waypoint
            self.heading_restored = False
            self.state_msg.data = STATE.GPS_NAVIGATION
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_NAVIGATION
            self.gps_navigation_state()  # enter the gps navigation state

        # might need a waypoint found here
        elif self.waypoint_found and self.waypoint_count == 4:
            self.waypoint_found = False
            self.state_msg.data = STATE.OBJECT_AVOIDANCE_FROM_LINE
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_AVOIDANCE_FROM_LINE
            self.object_avoidance_from_line_state()

    # GPS Navigation State
    def gps_navigation_state(self):
        # self.get_logger().info("GPS Navigation State")
        # After looking for an obstacle, see if we have arrived
        # turn light off so should blink at first waypoint
        # light_msg = LightCmd()
        # light_msg.type = 'G'
        # light_msg.on = False
        # self.lights_pub.publish(light_msg)
        # light_msg = LightCmd()
        # light_msg.type = 'B'
        # light_msg.on = False
        # self.lights_pub.publish(light_msg)

        if self.waypoint_found:
            self.waypoint_count += 1
            self.waypoint_found = False
            self.get_logger().info("WAYPOINT FOUND IN FSM!!")

            if self.waypoint_count == 4 or not self.get_parameter('/CrossRampInGps').value:
                # just take this step if not using nav across ramp
                self.state_msg.data = STATE.GPS_EXIT
                self.state_pub.publish(self.state_msg)
                self.state = STATE.GPS_EXIT
                self.exit_heading = self.gps_exit_heading
                self.gps_exit_state()
            else:
                # stay in gps state on to next object - redundant but for clarity
                self.exit_heading = self.target_heading
                self.state = STATE.ORIENT_TO_GPS
                self.state_msg.data = STATE.ORIENT_TO_GPS
                self.state_pub.publish(self.state_msg)
                self.orient_to_gps_state()

        # First look for a potential obstacle
        elif self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = STATE.GPS_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_TO_OBJECT

            self.prev_heading = self.heading
            self.exit_heading = self.target_heading

            self.get_logger().info(f"Current heading: {self.prev_heading}, exit heading: {self.exit_heading}")
            self.gps_to_object_state()

    # End of Major States

    # Beginning of Transition States
    # In these states the main controller (this class) is
    # the one controlling the wheels

    # Line Following to Object Avoidance Transition State
    def line_to_object_state(self):
        # self.get_logger().info("Line to Object Transition State")
        # Just keep turning until the object is not in front of us
        # hard control of the speeds with this command !!!
        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{self.TURN_SPEED}," \
                              f"{(-1 + 2*int(self.follow_dir==DIRECTION.LEFT)) * self.TURN_SPEED}"
        self.wheel_pub.publish(self.wheel_msg)

        self.get_logger().info("In line to object state publishing:")
        self.get_logger().info(f"{CODE.TRANSITION_CODE},{self.TURN_SPEED}," \
                              f"{(-1 + 2*int(self.follow_dir==DIRECTION.LEFT)) * self.TURN_SPEED}")

        if self.waypoint_found:
            self.waypoint_count += 1

            self.waypoint_found = False
            self.state = STATE.GPS_TO_OBJECT
            self.state_msg.data = STATE.GPS_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.gps_to_object_state()

        elif self.path_clear:
            self.path_clear = False
            self.obj_seen = False
            self.state_msg.data = STATE.OBJECT_AVOIDANCE_FROM_LINE
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_AVOIDANCE_FROM_LINE
            self.object_avoidance_from_line_state()

    # Object Avoidance to Line Following Transition State - is gps needed here?
    def object_to_line_state(self):
        # self.get_logger().info("Object to Line Transition State")


        # Gradual Turn
        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{self.SLIGHT_TURN}," \
                            f"{round((1-2*int(self.follow_dir==DIRECTION.RIGHT)) * (self.SLIGHT_TURN))}"
        self.wheel_pub.publish(self.wheel_msg)
        self.get_logger().info("In object to line state publishing:")
        self.get_logger().info(f"{CODE.TRANSITION_CODE},{self.SLIGHT_TURN}," \
                                    f"{round((1-2*int(self.follow_dir==DIRECTION.RIGHT)) * (self.SLIGHT_TURN))}")

        # Just keep turning until we are parallel with the line
        if self.aligned:

            self.aligned = False
            self.state_msg.data = STATE.LINE_FOLLOWING
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_FOLLOWING
            self.line_following_state()

        elif self.obj_seen:  # object sighted - switch to obstacle avoidance
            # We check for an object second because if we have already hit the
            # GPS waypoint we want the robot to record that first.
            self.obj_seen = False
            self.state = STATE.LINE_TO_OBJECT
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)

            self.line_to_object_state()  # enter the transition state


    # GPS Navigation to Object Avoidance Transition State
    def gps_to_object_state(self):
        # self.get_logger().info("GPS to Object Transition State")

        # Just keep turning until the object is not in front of us
        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{self.TURN_SPEED}," \
                              f"{(-1 + 2*int(self.follow_dir==DIRECTION.LEFT)) * self.TURN_SPEED}"
        self.wheel_pub.publish(self.wheel_msg)
        if self.path_clear:
            self.path_clear = False
            self.state_msg.data = STATE.OBJECT_AVOIDANCE_FROM_GPS
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_AVOIDANCE_FROM_GPS

            self.object_avoidance_from_gps_state()

        elif self.waypoint_found and self.waypoint_count == 4:
            self.waypoint_found = False
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_TO_OBJECT
            self.line_to_object_state()

    # Transition State to find the line after GPS Navigation
    def find_line_state(self):
        # self.get_logger().info("Find Line Transition State")
        # Just keep going until we find the line

        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{self.SLIGHT_TURN},{(-1+2*int(self.follow_dir))*self.SLIGHT_TURN*1/3}"
        self.wheel_pub.publish(self.wheel_msg)

        if self.found_line:
            self.found_line = False
            self.state_msg.data = STATE.LINE_ORIENT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_ORIENT
            self.line_orientation_state()  # Go to the next transition state

        elif self.obj_seen:
            self.obj_seen = False
            self.exit_heading = self.gps_exit_heading
            self.state = STATE.LINE_TO_OBJECT
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.line_to_object_state()

    # Transition State to Orient to the line direction
    def line_orientation_state(self):
        # self.get_logger().info("Line Orientation Transition State")

        # directly controls the motors
        # Just keep turning until we are parallel with the line
        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{self.SLIGHT_TURN}," \
                              f"{self.SLIGHT_TURN*(1-2*int(self.follow_dir==DIRECTION.RIGHT))}"
        self.wheel_pub.publish(self.wheel_msg)

        if self.aligned:
            self.aligned = False
            self.state_msg.data = STATE.LINE_FOLLOWING
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_FOLLOWING
            self.line_following_state()

    def orient_to_gps_state(self):
        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{10},{18*(-1+2*int(self.follow_dir==DIRECTION.RIGHT))}"
        self.wheel_pub.publish(self.wheel_msg)

        if self.heading_restored:
            self.heading_restored = False
            self.state = STATE.GPS_NAVIGATION
            self.state_msg.data = STATE.GPS_NAVIGATION
            self.state_pub.publish(self.state_msg)
            self.gps_navigation_state()  # enter the gps navigation state

        elif self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = STATE.GPS_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_TO_OBJECT

            self.prev_heading = self.heading
            self.exit_heading = self.target_heading
            # self.get_logger().info(f"Current heading: {self.prev_heading}, exit heading: {self.exit_heading}")
            self.gps_to_object_state()

    def gps_exit_state(self):

        self.wheel_msg.data = f"{CODE.TRANSITION_CODE},{8},{15*(-1+2*int(self.follow_dir==DIRECTION.RIGHT))}"
        self.wheel_pub.publish(self.wheel_msg)

        if self.heading_restored:
            self.heading_restored = False
            self.state = STATE.FIND_LINE
            self.state_msg.data = STATE.FIND_LINE
            self.state_pub.publish(self.state_msg)
            self.find_line_state()  # enter the gps navigation state

        elif self.obj_seen:
            self.obj_seen = False
            self.state = STATE.LINE_TO_OBJECT
            self.state_msg.data = STATE.LINE_TO_OBJECT
            self.state_pub.publish(self.state_msg)
            self.line_to_object_state()
    # End of Transition States

    # This function is essentially a big state machine handling transitions
    # between a number of different states in the system.
    def change_state(self):
        # self.get_logger().info(f"state {self.state}")
        if self.state == STATE.LINE_FOLLOWING:
            self.line_following_state()
        elif self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            self.object_avoidance_from_line_state()
        elif self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
            self.object_avoidance_from_gps_state()
        elif self.state == STATE.GPS_NAVIGATION:
            self.gps_navigation_state()
        elif self.state == STATE.LINE_TO_OBJECT:
            self.line_to_object_state()
        elif self.state == STATE.OBJECT_TO_LINE:
            self.object_to_line_state()
        elif self.state == STATE.GPS_TO_OBJECT:
            self.gps_to_object_state()
        elif self.state == STATE.FIND_LINE:
            self.find_line_state()
        elif self.state == STATE.LINE_ORIENT:
            self.line_orientation_state()
        elif self.state == STATE.ORIENT_TO_GPS:
            self.orient_to_gps_state()
        elif self.state == STATE.GPS_EXIT:
            self.gps_exit_state()
        else:
            self.get_logger().info("Error: Invalid State")

    # End of State Machine

    # Beginning of Callback Methods
    def heading_callback(self, heading_msg):
        self.heading = heading_msg.current_heading
        self.target_heading = heading_msg.target_heading
        # self.get_logger().info(f"Current heading in FSM: {self.heading}")
        if self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE and not self.heading_restored:
            direction_var = (-1+2*int(self.follow_dir==DIRECTION.RIGHT))
            obj_curr = self.heading*direction_var
            obj_exit = self.exit_heading*direction_var
            if sub_angles(obj_curr, obj_exit) >= 0:
                # in this case, we must have jumped the pi to -pi boundary
                # a left following case has a positive exit which has been flipped to negative
                # a right following case has a negative exit and a positive base heading
                # we need to confirm that both values are in the exit direction from 0
                self.get_logger().info(
                    f"Heading restored with heading {obj_curr * direction_var} and goal {obj_exit * direction_var}")
                self.heading_restored = True
        elif self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS and self.waypoint_count == 1:
            direction_var = (-1 + 2 * int(self.follow_dir == DIRECTION.RIGHT))
            self.exit_heading = self.target_heading
            gps_curr = self.heading*direction_var
            gps_exit = self.exit_heading*direction_var
            if sub_angles(gps_curr, gps_exit) >= 0 or sub_angles(gps_exit, gps_curr) < math.pi/6:
                self.get_logger().info(f"Heading restored with heading {gps_curr*direction_var}"
                                       f" and goal {gps_exit*direction_var}")
                self.heading_restored = True
            elif self.heading_restored:
                self.heading_restored = False
        elif self.state == STATE.ORIENT_TO_GPS or self.state == STATE.GPS_EXIT \
                or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
            if self.state == STATE.ORIENT_TO_GPS or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
                self.exit_heading = self.target_heading
            orient_curr = self.heading
            orient_exit = self.exit_heading
            if min(abs(sub_angles(orient_curr, orient_exit)), abs(sub_angles(orient_exit, orient_curr))) <= math.pi/12:
                self.get_logger().info(f"Heading restored with heading {orient_curr} and goal {orient_exit}")
                self.heading_restored = True
            elif self.heading_restored:
                self.heading_restored = False


    # Callback for information coming from the line following node
    def line_callback(self, line_event):

        # self.get_logger().info("Message from Line Following Node")

        # Get the lock before proceeding
        self.lock.acquire()
        try:
            if line_event.data == STATUS.FOUND_LINE and (
                    (self.heading_restored and self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE)
                    or self.state == STATE.FIND_LINE):
                self.get_logger().warning("FOUND LINE!!")
                # light_msg = LightCmd()
                # light_msg.type = 'B'
                # light_msg.on = True
                # self.lights_pub.publish(light_msg)
                self.found_line = True

            elif line_event.data == STATUS.ALIGNED \
                    and (self.state == STATE.OBJECT_TO_LINE or self.state==STATE.LINE_ORIENT):
                self.aligned = True
                light_msg = LightCmd()
                light_msg.type = 'G'
                light_msg.on = True
                self.get_logger().warning("ALIGNED TO LINE!!")
            else:
                pass
                # self.get_logger().info("UNKNOWN MESSAGE on line_events")
        finally:
            # Release the lock
            self.lock.release()

    # Callback for information coming from the GPS node
    def gps_callback(self, gps_event):
        self.get_logger().info(f"Message from Fusion Node {gps_event}")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            if gps_event.data == STATUS.WAYPOINT_FOUND:
                self.waypoint_found = True

                light_msg = LightCmd()
                light_msg.type = 'B'
                light_msg.on = True
                self.lights_pub.publish(light_msg)
                time.sleep(.10)
                light_msg.type = 'B'
                light_msg.on = False
                self.lights_pub.publish(light_msg)

            elif gps_event.data == STATUS.WAYPOINTS_DONE:
                self.waypoints_done = True
            else:
                self.get_logger().warning("Unknown Message")
        finally:
            # Release the lock
            self.lock.release()

    # Callback for information from the depth camera
    def lidar_callback(self, lidar_event):
        # self.get_logger().info("Message from LIDAR")

        # Get the lock before proceeding
        self.lock.acquire()
        try:
            if lidar_event.data == STATUS.OBJECT_SEEN and not self.waypoint_count == 2:
                self.obj_seen = True

                # # buzz if object seen
                light_msg = LightCmd()
                light_msg.type = 'Y'
                light_msg.on = True
                self.lights_pub.publish(light_msg)

            elif lidar_event.data == STATUS.PATH_CLEAR and self.state in self.transition_set:
                self.path_clear = True
                self.obj_seen = False
                light_msg = LightCmd()
                light_msg.type = 'Y'
                light_msg.on = False
                self.lights_pub.publish(light_msg)

        finally:
            # Release the lock
            self.lock.release()

    # Callback for the timer
    def timer_callback(self):
        # self.get_logger().info("Timer Callback")
        # self.get_logger().info(f"STATE: {self.state}")
        # Get the lock before proceeding
        self.lock.acquire()
        try:
            self.change_state()
            # self.get_logger().info(f"state: {self.state}")
        finally:
            # Release the lock
            self.lock.release()


def main(args=None):
    rclpy.init()
    main = MainRobot()
    time.sleep(3)
    # Get the lock so the Timer doesn't interfere
    main.lock.acquire()

    try:
        # Start the Robot
        main.lock.release()
        rclpy.spin(main)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
