################################
# AutoNav 2022 Competition Robot
# File: utils.py
# Purpose: store the different states used in the state machine
# Date Modified: 24 May 2022
################################
import cmath
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np


class STATE:
    LINE_FOLLOWING = 0
    OBJECT_AVOIDANCE_FROM_LINE = 1
    OBJECT_AVOIDANCE_FROM_GPS = 2
    GPS_NAVIGATION = 3
    LINE_TO_OBJECT = 4
    OBJECT_TO_LINE = 5
    GPS_TO_OBJECT = 6
    FIND_LINE = 7
    LINE_ORIENT = 8
    ORIENT_TO_GPS = 9
    GPS_EXIT = 10



class DIRECTION:
    LEFT = 0
    RIGHT = 1


class FollowMode:
    eeNone = 0
    eeLine = 1
    eeObject = 2
    eeGps = 3
    eeTransition = 4


# Messages that indicate a change of state is needed
class STATUS:
    PATH_CLEAR = "PATH_CLEAR"
    OBJECT_SEEN = "OBJECT_SEEN"
    WAYPOINT_FOUND = "WAYPOINT_FOUND"
    HEADING_RESTORED = "HEADING_RESTORED"
    FOUND_LINE = "FOUND_LINE"
    ALIGNED = "ALIGNED"
    WAYPOINTS_DONE = "WAYPOINTS_DONE"


# Messages that change the wheel controller's state
class CODE:
    # WHEELS_TRANSITION = "STR"
    # WHEELS_OBJECT_AVOIDANCE = "SOA"
    # WHEELS_LINE_FOLLOWING = "SLF"
    # WHEELS_GPS_NAV = "SGN"
    STOP_CODE = "STO"
    TRANSITION_CODE = "TRA"  # MESSAGES WITH THIS PREFIX BASICALLY MANUALLY CONTROL WHEELS
    LIN_SENDER = "LIN"
    OBJECT_SENDER = "OBJ"
    GPS_SENDER = "GPS"


def hsv_filter(image, use_white=True):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if not use_white:
        lower = np.array([18, 150, 100])
        upper = np.array([60, 255, 255])
    else:  # for white line following
        lower = np.array([0, 0, 200])
        upper = np.array([179, 55, 255])
    return cv2.inRange(hsv, lower, upper)


# Use cv_bridge() to convert the ROS image to OpenCV format
def bridge_image(ros_image, format):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, format)
        return cv_image
    except CvBridgeError as e:
        print("CvBridge could not convert images from ROS to OpenCV")


def bridge_image_pub(cv_image, format):
    try:
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, format)
        return ros_image
    except CvBridgeError as e:
        print("CvBridge could not convert images from OpenCV to ROS")


def cleanup():
    cv2.destroyAllWindows()


# Function used to display images in a way that allows them to be closed when not used.
def close_windows(handleArr):
    if len(handleArr) > 0:
        for handle in handleArr:
            cv2.destroyWindow(handle)
            handleArr.remove(handle)


# Closes unwanted windows
def cv_display(image, handle, handleArr):
    cv2.imshow(handle, image)
    cv2.waitKey(2)
    if handle not in handleArr:
        handleArr.append(handle)

def sub_angles(x, y):
    a = (x - y + 2 * cmath.pi) % (2 * cmath.pi)
    if a > cmath.pi:
        a -= 2 * cmath.pi
    return a

def dms_to_dmm(dms):
    minp = dms.split('.')
    degrees = math.trunc(float(dms))
    minutes = int(minp[1][:2])
    seconds = str(float(f'{minp[1][2:4]}.{minp[1][4:]}')/60)[2:]
    return float(f'{degrees}.{minutes}{seconds}')

def degrees_to_radians(degrees):
    heading = degrees * math.pi / 180.0
    if degrees > 180:
        heading -= math.pi * 2
    return heading
