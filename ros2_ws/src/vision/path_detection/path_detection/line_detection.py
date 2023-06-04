#!/usr/bin/env python11

################################
# Cedarville AutoNav 2022 Competition Robot
# Package: lines
# File: line_detection.py
# Purpose: controls line detection
# Author: Modified from 2020-21 autonav team code for ROS2
# Date Modified: 4 June 2023
################################

import cv2
import math
import numpy as np
from rclpy.node import Node
from utils.utils import *


class LineDetection():
    def __init__(self, buffersize, bufferfill, croptop, approach_croptop, cropbottom, cropside, maxwhite, minslope, linelength, linedistance, debug, use_yellow):
        # super().__init__('detection')
        self.history_idx = 0
        self.slope = None
        self.aligned = False
        self.found_line = False
        self.window_handle = []

        self.BUFF_SIZE = buffersize
        self.BUFF_FILL = bufferfill
        self.CROP_TOP = croptop
        self.APPROACH_CROP_TOP = approach_croptop
        self.CROP_BOTTOM = cropbottom
        self.CROP_SIDE = cropside
        self.MAX_WHITE = maxwhite
        self.MIN_SLOPE = minslope
        self.MIN_LINE_LENGTH = linelength
        self.LINE_DISTANCE = linedistance
        self.line_history = [0] * self.BUFF_SIZE

        self.debug = debug
        self.use_yellow = use_yellow

        self.distance = 0

    def image_callback(self, image, state):
        # Save Dimensions
        y, x = image.shape[0], image.shape[1]

        # Slice Edges
        if state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            image = image[int(y*self.APPROACH_CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]
        else:
            image = image[int(y*self.CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]

        # cv_display(image, 'Line Detection Color Image', self.window_handle)

        # Remove Shadows
        image = hsv_filter(image, use_white=not self.use_yellow)

        # Discard Oversaturated Images
        if np.count_nonzero(image) < image.shape[0]*image.shape[1]*self.MAX_WHITE:
            # Open Image
            morph = cv2.morphologyEx(image, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
            line, coords = self.determine_line(morph, state)
            if self.debug:
                if line:
                    x1, y1, x2, y2 = coords
                    morph = cv2.cvtColor(morph, cv2.COLOR_GRAY2RGB)
                    cv2.line(morph, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)
                cv_display(morph, 'Line Detection Opened Image', self.window_handle)
        else:
            # self.get_logger().warning("Oversaturated LineDetection Image")
            self.update_history(0)

        found_line = self.determine_state()
        aligned = self.determine_orientation()

        return found_line, aligned

    def determine_line(self, image, state):
        if state == "OBJECT_TO_LINE":
            lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH/2), maxLineGap=10)
        else:
            lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH), maxLineGap=10)

        if lines is not None:
            self.update_history(1)
            x1, y1, x2, y2 = lines[0][0]
            self.slope = self.get_slope(x1, y1, x2, y2)
            self.distance = self.get_distance(image.shape[1], image.shape[0], [x1, y1, x2, y2])
            # self.get_logger().info("LINE SLOPE: {} | LINE DISTANCE: {}".format(self.slope, self.distance))
            return True, [x1, y1, x2, y2]
        else:  self.update_history(0)

        return False, [0, 0, 0, 0]

    def get_slope(self, x1, y1, x2, y2):
        return float(abs(y2-y1)) / float(abs(x2-x1)+0.00001)

    def get_distance(self, y, x, coords):
        x1, y1, x2, y2 = coords
        line_center = [(x1 + x2) // 2, (y1 + y2) // 2]
        robot_center = [x // 2, y]
        dist_left = math.sqrt(pow(x1 - robot_center[0], 2) + pow(y1 - robot_center[1], 2))
        dist_right = math.sqrt(pow(x2 - robot_center[0], 2) + pow(y2 - robot_center[1], 2))
        dist_center = math.sqrt(pow(line_center[0] - robot_center[0], 2) + pow(line_center[1] - robot_center[1], 2))
        return min([dist_left, dist_right, dist_center]) / 300  # pixels to meters

    def determine_state(self):
        if self.line_history.count(1) >= self.BUFF_FILL * self.BUFF_SIZE and (self.distance <= self.LINE_DISTANCE):
            self.found_line = True
            return True
        return False

    def determine_orientation(self):
        # self.get_logger().info(f"Slope {self.slope}")

        if self.found_line and self.slope >= self.MIN_SLOPE:
            self.aligned = True
            return True
        return False

    def update_history(self, x):
        self.line_history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def reset(self):
        self.line_history = [0] * self.BUFF_SIZE
        self.found_line = False
        self.aligned = False
        self.history_idx = 0
        self.slope = None
