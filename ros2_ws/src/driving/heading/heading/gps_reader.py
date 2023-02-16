#!/usr/bin/env python

################################
# AutoNav 2022 Competition Robot
# Package: gps
# File: gps_reader.py
# Purpose: publish the gps messages needed by the master fsm
# Date Modified: 3 June 2022
################################

import cmath
import csv
from dataclasses import dataclass
from geopy import distance
import numpy as np
import serial
import time
from utils.utils import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from custom_msgs.msg import HeadingStatus

@dataclass
class SensorMsg:
    latitude   : float
    longitude  : float
    altitude   : float
    num_sat    : int
    hdop       : float
    time       : float


class GPS(Node):
    def __init__(self):
        super().__init__("gps")
        self.declare_parameter('/WaypointLat1', 0.0)
        self.declare_parameter('/WaypointLon1', 0.0)
        self.declare_parameter('/WaypointLat2', 0.0)
        self.declare_parameter('/WaypointLon2', 0.0)
        self.declare_parameter('/WaypointLat3', 0.0)
        self.declare_parameter('/WaypointLon3', 0.0)
        self.declare_parameter('/WaypointLat4', 0.0)
        self.declare_parameter('/WaypointLon4', 0.0)
        self.declare_parameter('/GPSFollowGoal', 1.0)
        self.declare_parameter('/LineToGPSTrans', 5.0)
        self.declare_parameter('/Port', '/dev/ttyACM0')
        self.declare_parameter('/Debug', False)
        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/NorthPointFirst', False)
        self.declare_parameter('/RealCourse', False)
        self.declare_parameter('/PracticeWaypointLat1', 0.0)
        self.declare_parameter('/PracticeWaypointLon1', 0.0)
        self.declare_parameter('/PracticeWaypointLat2', 0.0)
        self.declare_parameter('/PracticeWaypointLon2', 0.0)
        self.declare_parameter('/PracticeWaypointLat3', 0.0)
        self.declare_parameter('/PracticeWaypointLon3', 0.0)
        self.declare_parameter('/PracticeWaypointLat4', 0.0)
        self.declare_parameter('/PracticeWaypointLon4', 0.0)

        self.DISTANCE_GOAL = self.get_parameter('/GPSFollowGoal').value
        self.line_to_gps = self.get_parameter('/LineToGPSTrans').value

        self.target_loc = []
        if self.get_parameter('/RealCourse').value:
            WP_LAT1 = self.get_parameter('/WaypointLat1').value
            WP_LON1 = self.get_parameter('/WaypointLon1').value
            WP_LAT2 = self.get_parameter('/WaypointLat2').value
            WP_LON2 = self.get_parameter('/WaypointLon2').value
            WP_LAT3 = self.get_parameter('/WaypointLat3').value
            WP_LON3 = self.get_parameter('/WaypointLon3').value
            WP_LAT4 = self.get_parameter('/WaypointLat4').value
            WP_LON4 = self.get_parameter('/WaypointLon4').value

            self.target_loc.append(complex(WP_LAT1, WP_LON1))
            self.target_loc.append(complex(WP_LAT2, WP_LON2))
            self.target_loc.append(complex(WP_LAT3, WP_LON3))
            self.target_loc.append(complex(WP_LAT4, WP_LON4))
        else:
            WP_LAT1 = self.get_parameter('/PracticeWaypointLat1').value
            WP_LON1 = self.get_parameter('/PracticeWaypointLon1').value

            # get middle
            WP_LAT2 = self.get_parameter('/PracticeWaypointLat2').value
            WP_LON2 = self.get_parameter('/PracticeWaypointLon2').value
            WP_LAT3 = self.get_parameter('/PracticeWaypointLat3').value
            WP_LON3 = self.get_parameter('/PracticeWaypointLon3').value

            WP_LAT4 = self.get_parameter('/PracticeWaypointLat4').value
            WP_LON4 = self.get_parameter('/PracticeWaypointLon4').value

            self.target_loc.append(complex(WP_LAT1, WP_LON1))
            self.target_loc.append(complex(WP_LAT2, WP_LON2))
            self.target_loc.append(complex(WP_LAT3, WP_LON3))
            self.target_loc.append(complex(WP_LAT4, WP_LON4))

        # flip if
        if not self.get_parameter('/NorthPointFirst').value:
            self.target_loc.reverse()

        # Publish new events that may change the overall state of the robot
        self.gps_event_pub = self.create_publisher(String, "gps_events", 10)
        self.heading_pub = self.create_publisher(HeadingStatus, 'gps_heading', 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)
        self.state = STATE.LINE_FOLLOWING

        # process GPS data at 2 Hz
        self.timer = self.create_timer(.45, self.process_gps_data)

        self.ser = serial.Serial(self.get_parameter('/Port').value, baudrate=115200)
        self.ser.readline()  # read one junk line to achieve line synchronization

        # useful constants
        self.LINE_MODE = {STATE.LINE_FOLLOWING, STATE.OBJECT_AVOIDANCE_FROM_LINE, STATE.OBJECT_TO_LINE,
                          STATE.LINE_TO_OBJECT}

        # position in gps rounds
        self.waypoint_itr = 0
        self.past_loc = self.take_reading()
        # self.target_loc.append(self.past_loc)  # set initial position as end goal
        self.moving_avg = np.zeros((5,), dtype=np.float32)
        self.moving_avg_idx = 0

        # needs to run once at beginning of script
        name = 'gps-log-' + str(round(time.time())) + '.csv'
        if self.get_parameter('/RealCourse').value:
            name = 'gps-log-real-' + str(round(time.time())) + '.csv'
        self.logfile = open(name, 'w')
        self.writer = csv.writer(self.logfile)


        self.done = False
        self.get_logger().info("GPS Node configured")

    # a 5 point moving average filter
    def filter_angle(self, new_val):
        self.moving_avg = np.roll(self.moving_avg, 1)
        self.moving_avg[0] = new_val
        return sum(self.moving_avg) / self.moving_avg.size

    # this function calculates the heading in radians from 2 points
    def calc_heading(self, past, curr):
        trajectory = curr - past
        return cmath.phase(trajectory)  # in radians [-pi, pi]  # this might be a problem

    # this function returns true if we are within the threshold of the
    # GPS waypoint
    def check_waypoint(self, curr):
        target_point = (self.target_loc[self.waypoint_itr%len(self.target_loc)].real, self.target_loc[self.waypoint_itr%len(self.target_loc)].imag)
        current_point = (curr.real, curr.imag)
        dist_meters = distance.distance(current_point, target_point).m
        self.get_logger().info(f"distance from waypoint: {dist_meters}")

        if self.state == STATE.GPS_NAVIGATION or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
            #dist_limit = 0.65
            dist_limit = 0.75
        else:
            #dist_limit = self.DISTANCE_GOAL
            dist_limit = 0.75

        if dist_meters <= dist_limit:
            msg = String()
            msg.data = STATUS.WAYPOINT_FOUND
            self.waypoint_itr = (self.waypoint_itr + 1) % len(self.target_loc)  # go to next waypoint goal
            self.gps_event_pub.publish(msg)
            self.get_logger().info(f"WAYPOINT FOUND - SWITCH POINTS to {self.target_loc[self.waypoint_itr]}")

            # if self.waypoint_itr >= len(self.target_loc):
            #     msg = String()
            #     msg.data = STATUS.WAYPOINTS_DONE
            #     self.gps_event_pub.publish(msg)
            #     self.get_logger().warning("FINISHED GPS")
        return dist_meters

    # this function takes a measurement and calculates all of the necessary
    # information from the location to send to the motor controller
    def process_gps_data(self):
        loc = self.take_reading()  # get the new reading
        # Check if we are at the waypoint
        distance = self.check_waypoint(loc)
        if self.get_parameter('/Debug').value:
            # pass
            self.get_logger().info(f"loc {loc}\n")
                                   # f"Desired location: {self.target_loc[self.waypoint_itr]}")

        # calculate our current heading and the heading we need to have and publish these
        curr_heading = self.calc_heading(self.past_loc, loc)
        target_heading = self.calc_heading(loc, self.target_loc[self.waypoint_itr])
        heading_msg = HeadingStatus()
        heading_msg.current_heading = curr_heading
        heading_msg.target_heading = target_heading
        heading_msg.distance = distance
        self.heading_pub.publish(heading_msg)

        # save off the location
        self.past_loc = loc

    # this function will read from the gps a single nmea sentence
    # and return a lat, long complex pair
    def take_reading(self):
        while True:
            try:
                line = self.ser.readline().decode()
                message = line.split(',')
                if message[0] == '$GNGGA':
                    lat = float(message[2]) / 100 * (-1 + 2 * int(message[3] == 'N'))
                    lon = float(message[4]) / 100 * (-1 + 2 * int(message[5] == 'E'))
                    self.get_logger().warning(f"FOUND GNGGA FIX {lat}, {lon}")
                    self.log_gps(message)
                    return complex(lat, lon)
                elif message[0] == "$GNRMC":
                    lat = float(message[3]) / 100 * (-1 + 2 * int(message[4] == 'N'))
                    lon = float(message[5]) / 100 * (-1 + 2 * int(message[6] == 'E'))
                    self.get_logger().warning(f"FOUND GNRMC FIX {lat}, {lon}")
                    return complex(lat, lon)
                # elif message[0] == "$GNGLL":
                #     lat = float(message[1]) / 100 * (-1 + 2 * int(message[2] == 'N'))
                #     lon = float(message[3]) / 100 * (-1 + 2 * int(message[4] == 'E'))
                #     # self.get_logger().warning(f"FOUND GNRMC FIX {lat}, {lon}")
                #     return complex(lat, lon)
            except Exception as e:
                # self.get_logger().warning(f"ERROR IN READING: {e}. Take robot outside")
                pass
                # time.sleep(1)


    def state_callback(self, new_state):
        # self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = new_state.data

    def log_gps(self, nmeastring):
        # call "log_gps(GNGGA_string)" each time new GPS data is received
        # nmealist = nmeastring.split(',')
        self.writer.writerow(np.concatenate(([time.time()],nmeastring)))

    def __del__(self):
        # self.get_logger().info("Deleting GPS Node")
        # self.get_logger().info("Closing GPS Connection")
        # needs to run once at end of script
        self.logfile.close()

        # close serial port
        self.ser.close()
        time.sleep(0.2)

        # Close the recording file
        # self.get_logger().info("Cleanup Finished")


def main(args=None):
    rclpy.init(args=args)
    gps = GPS()
    try:
        rclpy.spin(gps)
    except KeyboardInterrupt:
        gps.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

