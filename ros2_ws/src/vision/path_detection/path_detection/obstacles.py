################################
# AutoNav 2022 Competition Robot
# Package: rs2l_transform
# File: obstacles.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 24 May 2022
# To run: ros2 run path_detection obstacles
################################

# !/usr/bin/env python

from dataclasses import dataclass
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from utils.utils import *
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import cv2, csv
from numpy import asarray
from PIL import ImageOps
import time
from cv_bridge import CvBridge
from utils.utils import *

@dataclass
class Circle:
    xcenter: float
    ycenter: float
    radius: float


# given ax+bx+c=0 and center and radius of a circle, determine if they intersect and returns distance in meters from edge
def check_collision(a, b, c, x, y, radius):
    # Finding the distance of line from center
    dist = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))

    # return the distance from the circle edge (approximate) and whether it touches
    if radius < dist:
        return 0, False
    else:
        return math.sqrt((x - 190) * (x - 190) + (y - 452) * (y - 452)) / 150 - 0.3, True


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('obstacles')
        self.lidar_pub = self.create_publisher(LaserScan, '/laser_frame', 10)
        self.lidar_str_pub = self.create_publisher(String, '/mod_lidar', 10)
        self.lidar_wheel_distance_pub = self.create_publisher(String, "wheel_distance", 10)

        # Subscribe to the camera color image and unaltered laser scan
        #FIXME
        self.get_logger().info('*1*********************')
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.get_logger().info('*2*********************')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)
        self.state = STATE.LINE_FOLLOWING

        # lidar parameters
        self.declare_parameter("/LIDARTrimMin", 1.31)
        self.declare_parameter("/LIDARTrimMax", 4.97)
        self.declare_parameter("/ObstacleFOV", math.pi/6)
        self.declare_parameter("/ObstacleDetectDistance", 1.5)  # meters
        self.declare_parameter("/FollowingDirection", 1)

        # camera parameters
        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)
        self.declare_parameter("/PotholeBufferSize", 5)

        self.declare_parameter('/Debug', False)

        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value


        # camera/obstacle detection
        self.circles = []  # recent circle history
        self.window_handle = []
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True

        self.get_logger().info("Waiting for image/lidar topics...")

    def state_callback(self, new_state):
        # self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = new_state.data

    def get_c(self, i, scan):
        return -(190 * (452-math.cos(i * scan.angle_increment)) - 452 * (190-math.sin(i * scan.angle_increment)))

    def check_range(self, scan, min, max, max_distance):
        distances = 0
        count = 0
        for i in range(len(scan.ranges)):
            if max > i * scan.angle_increment > min and scan.ranges[i] is not None and scan.ranges[i] != math.inf\
                    and scan.ranges[i] < max_distance:
                distances += scan.ranges[i] * math.sin(i*scan.angle_increment)
                count += 1
        try:
            return distances / count
        except ZeroDivisionError:
            # self.get_logger().info("ZERO DIVISION ERROR")
            return max_distance + .75  # parameterize later

    # first portion nullifies all data behind the scanner after adjusting min and max to be 0
    # second portion adds potholes based on image data
    # third portion replaces obstacle in front and time of flight sensors
    def lidar_callback(self, scan):
        # adjust range to only include data in front of scanner\

        # self.get_logger().info(f"trim range: {scan.angle_max}, scan range: {scan.angle_min}")
        scan.angle_max += math.pi
        scan.angle_min += math.pi

        trim_min = self.get_parameter('/LIDARTrimMin').value
        trim_max = self.get_parameter('/LIDARTrimMax').value
        
        new_ranges = []
        new_intensities = []

        startOffset = int(scan.angle_min/scan.angle_increment)
        endOffset = int(scan.angle_max/scan.angle_increment)

        startTrim = int(trim_min/scan.angle_increment)
        endTrim = int(trim_max/scan.angle_increment)
        try:
            if len(scan.intensities) > 0:
                i = 0
                while i <= startOffset:
                    new_ranges.append(math.inf)
                    new_intensities.append(0.0)
                    i += 1
                while i <= startTrim:
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    new_intensities.append(scan.intensities[i - (startOffset + 1)])
                    i += 1
                while i <= endTrim:
                    new_ranges.append(math.inf)
                    new_intensities.append(0.0)
                    i += 1
                while i <= endOffset:
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    new_intensities.append(scan.intensities[i - (startOffset + 1)])
                    i += 1
                while i <= int(6.28/scan.angle_increment):
                    new_ranges.append(math.inf)
                    new_intensities.append(0.0)
                    i += 1
                scan.ranges = new_ranges
                scan.intensities = new_intensities
            else:
                i = 0
                while i <= startOffset:
                    new_ranges.append(math.inf)
                    i += 1
                while i <= startTrim:
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    i += 1
                while i <= endTrim:
                    new_ranges.append(math.inf)
                    i += 1
                while i <= endOffset:
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    i += 1
                while i <= int(6.28/scan.angle_increment):
                    new_ranges.append(math.inf)
                    i += 1
                scan.ranges = new_ranges
            scan.angle_min = 0.0
            scan.angle_max = 6.28
        except Exception:
            self.get_logger().info(f"ERROR: removing extraneous data broke ranges length: {len(scan.ranges)}, width: {width}")

	# uncommented by James Oct 22 for pothole evaluation
	# START
        # insert pothole additions to lidar here - can compensate with constants for the camera angle - REMOVED AT COMPETITION BECAUSE NO POTHOLES
        # for circle in self.circles:
        #      # front part of lidar scan 0 to pi/2 radians
        #      for i in range(len(scan.ranges)):
        #          if i < (len(scan.ranges) // 4) or i > len(scan.ranges) // 4 * 3:
        #              dist, hit = check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
        #                                          self.get_c(i, scan), circle.xcenter, circle.ycenter, circle.radius)
        #              if dist < scan.ranges[i] and hit:
        #                  scan.ranges[i] = dist
        #                  scan.intensities[i] = 47
        # print(". . . . . . . Made it to Finish . . . . .")
	# FINISH
	# T1 rolled over; pothole touching the line; no change
	# T2 rolled over; pothole a foot away from line; no change
	# T3 no roll over sudden left turn with beep; pothole a foot away from line; no change
	# T4 rolled over; pothole a foot away; no change
	# T5 rolled over; pothole a foot away; no change
	# T6 rolled over; pothole 2 feet away; no change
        self.lidar_pub.publish(scan)

        # scan in the range in front of robot to check for obstacles
        msg = String()
        msg.data = STATUS.PATH_CLEAR
        count1 = 0
        if self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            follow_dist = self.get_parameter("/ObstacleDetectDistance").value *3/4
        else:
            follow_dist = self.get_parameter("/ObstacleDetectDistance").value
        for i in range(len(scan.ranges)):
            if i * scan.angle_increment < self.get_parameter('/ObstacleFOV').value/2 \
                    or i * scan.angle_increment > 2 * math.pi - self.get_parameter('/ObstacleFOV').value / 2:
                if scan.ranges[i] < follow_dist: #self.get_parameter("/ObstacleDetectDistance").value:
                    if count1 > 1: # get at least two points of obstacle in front to trigger found
                        msg.data = STATUS.OBJECT_SEEN
                    count1+=1

        if msg.data == STATUS.PATH_CLEAR:
            self.update_history(0)
        else:
            self.update_history(1)

        if np.count_nonzero(self.history) >= 0.6 * self.BUFF_SIZE:
            if self.get_parameter('/Debug').value:
                self.get_logger().info("OBJECT_SEEN")
            self.path_clear = False
        elif np.count_nonzero(self.history) <= (1 - .6) * self.BUFF_SIZE and not self.path_clear:
            if self.get_parameter('/Debug').value:
                self.get_logger().info("PATH_CLEAR")
            self.path_clear = True
        self.lidar_str_pub.publish(msg)

        # publish the wheel distance from the obstacle based on following direction
        distance_msg = String()
        try:
            if self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
                if self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT:
                    distance_msg.data = "OBJ," + str(self.check_range(scan, 70*math.pi/180, 84*math.pi/180, 2.0))
                    # self.get_logger().info("Publishing from obstacles.py:")
                    # self.get_logger().info(f"Distance message data: {distance_msg}")
                    self.lidar_wheel_distance_pub.publish(distance_msg)
                elif self.get_parameter('/FollowingDirection').value == DIRECTION.RIGHT:
                    distance_msg.data = "OBJ," + str(self.check_range(scan, 276*math.pi/180, 290*math.pi/180, 2.0))
                    # self.get_logger().info("Publishing from obstacles.py:")
                    # self.get_logger().info(f"Distance message data: {distance_msg}")
                    self.lidar_wheel_distance_pub.publish(distance_msg)

        except Exception as e:
            self.get_logger().warning(f"ERROR with TOF Following direction : {e}")

    # receives camera image and parses potholes into history
    def image_callback(self, image):
        self.get_logger().info('*4*********************')
        image = bridge_image(image, "bgr8")
        # slice edges
        y, x = image.shape[0], image.shape[1]
        image = image[int(y * self.get_parameter('/LineDetectCropTop').value):-int(y * self.get_parameter('/LineDetectCropBottom').value),
                int(x * self.get_parameter('/LineDetectCropSide').value):-int(x * self.get_parameter('/LineDetectCropSide').value)]

        # Apply HSV Filter
        #gray = hsv_filter(image)
        #morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # look for certain type blobs aka potholes - hone these with other obstacles
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True  # area of acceptable blob
        params.minArea = 8000
        params.maxArea = 30000
        params.filterByCircularity = True  # square has circularity of like 78%
        params.minCircularity = .75
        params.maxCircularity = 1
        params.filterByConvexity = True  # more convexity is closer to circle
        params.minConvexity = .85
        params.maxConvexity = 1
        params.filterByInertia = True
        params.minInertiaRatio = .45
        params.maxInertiaRatio = 1
        params.filterByColor = False


        #***************************************************************

        self.get_logger().info('**********************')
        t1 = time.time()
        kernel = np.ones((5,5),np.uint8)
        ksize = (5,5)

        #image = Image.open('3m.jpg')
        #image = cv2.imread('3m.jpg')

        bridge = CvBridge()
        #cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        # Convert the image to grayscale
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Invert the image (black becomes white, white becomes black)
        img_inv = abs(img_gray - 255)

        # Color the non-black spots more white
        factor = 1.5  # Change this factor to adjust the degree of whitening
        img_white = ImageOps.autocontrast(from_array(img_inv), cutoff=0, ignore=255).point(lambda i: i*factor)

        numpydata = np.array(img_white)

        #removing other componets
        (thresh, blackAndWhiteImage) = cv2.threshold(numpydata, 127, 255, cv2.THRESH_BINARY)

        # HSV filtering
        #grey = hsv_filter(image, use_white=True)
        dilation = cv2.dilate(blackAndWhiteImage,kernel,iterations = 10)
        erosion = cv2.erode(dilation,kernel,iterations = 10)

        closing = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel)


        # Apply HoughCircles to detect circles
        circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT_ALT, 1, 1, param1=100, param2=0.1, minRadius=100, maxRadius=0)
        color = (0, 255, 0)
        markerType = cv2.MARKER_CROSS
        markerSize = 90
        thickness = 50

        im_rgb = cv2.cvtColor(closing, cv2.COLOR_BGR2RGB)
        t2=time.time()
        # Draw detected circles on the original image
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            i =0
            for (x, y, r) in circles:
                cv2.circle(im_rgb, (x, y), r, (0, 0, 0), 2)
                print(x, y, r)
                i=i+1
                cv2.drawMarker(im_rgb, (x, y), color, markerType, markerSize, thickness)

        t3=time.time()
        print('time before loop')
        print(t2-t1)
        print('time after loop')
        print(t3-t1)
        print("loop took:")
        print(t3-t2)
        cv2.imwrite("originN.png", im_rgb)
        image = bridge_image(im_rgb, "bgr8")
        morph = im_rgb

        #*************************************************************

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(morph)  # find the blobs meeting the parameters
        self.circles = []
        #for hole in keypoints:
        #    self.circles.insert(0, Circle(hole.pt[0], hole.pt[1], hole.size//2))

        if self.get_parameter('/Debug').value:
            blobs = cv2.drawKeypoints(morph, keypoints, np.zeros((1, 1)), (0, 255, 0),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #cv_display(blobs, 'Potholes', self.window_handle)

    def update_history(self, x):
        self.history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def reset(self):
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.path_clear = True


def main(args=None):
    rclpy.init(args=args)

    transform = TransformPublisher()

    try:
        rclpy.spin(transform)
    except KeyboardInterrupt:
        # Destroy the node explicitly (optional)
        transform.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
