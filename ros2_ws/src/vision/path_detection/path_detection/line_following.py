#!/usr/bin/env python

################################
# Cedarville AutoNav 2022 Competition Robot
# Package: lines
# File: line_following.py
# Purpose: controls line detection
# Author: Modified from 2020-21 autonav team code for ROS2
# Date Modified: 20 May 2022
################################

import cv2
import numpy as np
from rclpy.node import Node
from utils.utils import *

class LineFollowing():
    def __init__(self, direction, use_yellow, debug):
        # super().__init__('following')

        # Class attributes
        self.MAX_DIST = 7777
        self.GOOD_DIST = 200
        self.distance = self.MAX_DIST
        self.max_white = -1
        self.max_start = 800
        self.no_line_count = 0
        self.run = "go"
        self.LINE_CODE = "LIN,"
        self.window_handle = []

        self.use_yellow = use_yellow

        self.FOLLOWING_DIR = direction
        self.BUFF_SIZE = 5
        self.THRESH_MIN = 250
        self.MAX_PIXEL = 255  # linethreshmax
        self.HEIGHT_START = 470.0
        self.HEIGHT_STEP = 50.0
        self.LINE_LOST_COUNT = 100
        self.LINE_FOLLOW_DIST = 200  ## line dist
        self.DEBUG_MODE = debug
        self.moving = [200] * self.BUFF_SIZE

    def filter_result(self):
        # 5 point moving average filter
        if self.distance != 7777:
            print(self.moving)
            self.moving = np.roll(self.moving, 1)
            self.moving[0] = self.distance

        filt_dist = sum(self.moving) / self.BUFF_SIZE
        return (filt_dist)

    def filter_image(self, image):
        # HSV filtering
        grey = hsv_filter(image, use_white=not self.use_yellow)

        # # REMOVE, UNNECESSARY DUE TO NEW HSV FILTERING
        # # Threshold a single portion of the image and then place that portion on a blank image
        # ret, grey = cv2.threshold(grey, self.THRESH_MIN, self.MAX_PIXEL, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Perform a morphological opening operation on the image with a rectangular structuring element
        element = cv2.getStructuringElement(cv2.MORPH_RECT,
                                            (int((7.0 / 1280.0) * grey.shape[0]), int((20.0 / 720.0) * grey.shape[1])))
        mask = cv2.morphologyEx(grey, cv2.MORPH_OPEN, element)

        # cv_display(mask, 'filter result', self.window_handle)
        return mask

    # returns the value in pixels that an assumed line is from the center of the image
    def follow_line(self, mask, begin, end, h, y):
        # Maximum values
        confidence = 100
        self.max_white = confidence

        # Width of the regin of interest
        # The divide and multiply allow this value to be used on any image size
        # normalize in relation to a 1080 by 720 image, then put in reference to the current image dimension
        w = int((self.HEIGHT_STEP / 720.0) * mask.shape[0])

        # Find the square region with the highest white pixel count
        for x in range(begin, end, 1):
            white_count = cv2.countNonZero(mask[y:y + h, x:x + w])
            if white_count > self.max_white:
                self.max_white = white_count
                self.max_start = x

        # Obtain pixel distance
        # check to see if the max pixel count is substantial
        if self.max_white > confidence:
            new_dist = (self.max_start - (mask.shape[1] / 2))
            # if there is a jump in the detected line position, just go straight
            self.no_line_count = 0
            if abs(new_dist - self.distance) <= 7 * w or self.distance == self.MAX_DIST:
                self.distance = new_dist
            else:
                self.distance = self.GOOD_DIST
            self.distance = new_dist

        else:  # if inconclusive  line found in the image
            # search horizontal boxes above our usual frame
            self.no_line_count += 1

        # report if we lost the line
        # if not self.run or self.no_line_count == self.LINE_LOST_COUNT:
        # self.distance = self.GOOD_DIST

        # 5 point moving average filter
        filt_dist = self.filter_result()

        # return the resulting distance
        return filt_dist

    def reset(self):
        self.distance = self.MAX_DIST
        self.no_line_count = 0
        # self.moving = array('i', [200] * self.BUFF_SIZE)

    def printResult(self, mask, original_image, y, h, line_dist):
        # Creates and overlays a green square on the image where we think the line is
        # start pixels and width of square
        w = int((self.HEIGHT_STEP / 720.0) * mask.shape[0])
        self.max_start = line_dist + (mask.shape[1] / 2)

        # Create overlay
        pixels = np.array(
            [[self.max_start, y], [self.max_start, y + h], [self.max_start + w, y + h], [self.max_start + w, y]],
            dtype=np.int32)
        overlay = cv2.fillConvexPoly(original_image, pixels, (43, self.MAX_PIXEL, 0))

        # Apply Overlay
        result = cv2.addWeighted(original_image, 1, overlay, 0.5, 0)

        # Display results
        cv_display(result, "result", self.window_handle)

    # This function takes an image, and returns the value of the distance (in pixels) that the line is
    # from the center of the image
    def image_callback(self, original_image):
        # Reference variables to help find portion of image
        x = 0
        y = int((self.HEIGHT_START / 720.0) * original_image.shape[0])
        w = original_image.shape[1]
        h = int((self.HEIGHT_STEP / 720.0) * original_image.shape[0])

        # Mask out portion of the image
        # Set region of interest based on left or right follow
        if self.FOLLOWING_DIR == 1:
            begin = int(0.5 * original_image.shape[1])
            end = original_image.shape[1]
        else:
            begin = 0
            end = int(0.5 * original_image.shape[1])

        ## region of interest image to help find specific portion of the image
        section = np.zeros(original_image.shape, np.uint8)
        section[y:y + h, begin:end] = original_image[y:y + h, begin:end]

        # filter the line
        mask = self.filter_image(original_image)

        # find the line
        line_dist = self.follow_line(mask, begin, end, h, y)

        # print result for debug
        if self.DEBUG_MODE:
            self.printResult(mask, original_image, y, h, line_dist)
            # cvDisplay(section, "Region of interest", self.window_handle)

        return abs(line_dist)
