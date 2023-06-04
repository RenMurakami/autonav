################################
# AutoNav 2023 Competition Robot
# Package: rs2l_transform
# File: obstacles.py
# Purpose: detect potholes and obstacles in the front 180 degrees
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
from custom_msgs.msg import LightCmd

import cv2
from PIL import ImageOps
from PIL import Image as im
import time
from cv_bridge import CvBridge


@dataclass
class Circle:
    xcenter: float
    ycenter: float
    radius: float


# this funciton deals with pothole design - we have not changed/updated the parms for the physical lidar change
# given ax+bx+c=0 and center and radius of a circle, determine if they intersect and returns distance in meters from edge
def check_collision(a, b, c, x, y, radius):
    # Finding the distance of line from center
    dist = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))

    # return the distance from the circle edge (approximate) and whether it touches
    if radius < dist:
        return 0, False
    else:
        # need to know what these consts are for (need to be update?? 1/28/23)
        return math.sqrt((x - 190) * (x - 190) + (y - 452) * (y - 452)) / 150 - 0.3, True


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('obstacles')
        self.lidar_pub = self.create_publisher(LaserScan, '/laser_frame', 10)
        self.lidar_str_pub = self.create_publisher(String, '/mod_lidar', 10)
        self.lidar_wheel_distance_pub = self.create_publisher(String, "wheel_distance", 10)
        self.lights_pub = self.create_publisher(LightCmd, "light_events", 10)

        # Subscribe to the camera color image and unaltered laser scan
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.image_pub = self.create_publisher(String,"pothole_events",10)
        self.get_logger().info('Pothole set up')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)
        self.state = STATE.LINE_FOLLOWING

        # lidar parameters
        self.declare_parameter("/LIDARTrimMin", 2.8808)         # radians = 165.0575 degrees
        self.declare_parameter("/LIDARTrimMax", 0.2576)         # radians = 14.7 degrees
        self.declare_parameter("/ObstacleFOV", math.pi/6)       #
        self.declare_parameter("/ObstacleDetectDistance", 1.5)  # meters = 4.9213 ft  # may need to fine tune param to detect farther out
        self.declare_parameter("/ObstacleToPlainDistance", 2.0)
        self.declare_parameter("/ObstacleNoiseMinDist", 0.3)
        self.declare_parameter("/FollowingDirection", 1)        # meters

        # camera parameters
        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)

        self.declare_parameter("/PotholeDetectCropTop", 0.4)
        self.declare_parameter("/PotholeDetectCropBottom", 0.2)
        self.declare_parameter("/PotholeDetectCropSide", 0.2)
        self.declare_parameter("/PotholeBufferSize", 5)

        self.declare_parameter('/Debug', False)

        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value

        # camera/obstacle detection
        self.circles = []  # recent circle history
        self.window_handle = []
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True
        self.pothole_found = False

        # Publish events that could change the robot state
        self.event_pub = self.create_publisher(String, "pothole_events", 10)

        self.get_logger().info("Waiting for image/lidar topics...")

    def state_callback(self, new_state):
        # self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = new_state.data

    # this funciton deals with pothole design - we have not changed/update parms due to the lidar physical change
    def get_c(self, i, scan):
        return -(190 * (452 - math.cos(i * scan.angle_increment)) - 452 * (
                    190 - math.sin(i * scan.angle_increment)))  # is this center of obstacle or pothole??

    # calculates avg distance from the obstacle to the front plan of the robot
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

    # new def to squash bad values and to detect and paint a plane on each barricade/barrel
    def lidar_ObjToPlane(self, scan):

        # identify and save points for possible legs and barrels
        threshold = 0.08 # was 0.08 # a param - max distance between points to check if it's apart of same obj
        objPoints = []
        objCorners = []
        pointLeftMax = 0
        pointRightMax = 0
        legsFound = []
        legLRCorners = []
        legMinDist = []
        barrelsFound = []
        barrelLRCorners = []
        barrelMinDists = []
        minDist = 10 # can param

        windowMax = self.get_parameter("/ObstacleToPlainDistance").value
        windowMin = self.get_parameter("/ObstacleNoiseMinDist").value

        # finds the point clusters in window and determines if it is a leg or barrel
        for point in range(len(scan.ranges)):
            # get rid of noise values that are too close/far
            if (scan.ranges[point] > windowMax) or (scan.ranges[point] < windowMin):
                scan.ranges[point] = inf
            else:
                # uses a radial cordinate system
                #objPoints.append((point, scan.ranges[point])) # may be able to get ride of
                #minDist = min(minDist, scan.ranges[point]) # may be able to get ride of

                # uses a cartesian coordinate system
                objPoints.append(point)
                minDist = min(minDist, (scan.ranges[point] * math.sin(point * scan.angle_increment)))
                # NEED TO FIX
                """
                if ((abs(scan.ranges[point] - scan.ranges[point + 1]) > threshold) and (abs(scan.ranges[point] - scan.ranges[point + 2]) > threshold)):
                    # handles legs
                    if 1 < len(objPoints) < 14: #  can param if needed
                        legsFound.append(objPoints)
                        legLRCorners = [legsFound[0][0][0], point] # assuming only one barricade in view at any time instant
                        legMinDist = minDist
                        objPoints = []
                    # handles barrels
                    elif 14 <= len(objPoints):
                        barrelsFound.append(objPoints)
                        barrelLRCorners.append([objPoints[0][0],objPoints[-1][0]])
                        barrelMinDists.append(minDist)
                        minDist = 3
                        objPoints = []
                else:
                """
        # need to keep track of which barrel we are on
        #barrelCount = len(barrelsFound)
        #self.get_logger().info(f"Barrels: {barrelCount}, corners: {barrelLRCorners}")
        #legCount = len(legsFound) # assuming only one barricade in view for this function
        #self.get_logger().info(f"Legs: {legCount}, corners: {legLRCorners}")
        #self.get_logger().info(f"Min BarrelDist: {barrelMinDists}, Min LegDist: {legMinDist}")
        #barrel = 0
        padding = 2 # can param
        #isBarrel = False

        #self.get_logger().info(f"Point: {point}, objP L: {objPoints[0]}")
        #self.get_logger().info(f"Point: {point}, objP R: {objPoints[-1]}")

        # added if to compensate for not catching every single barrel orientation
        """ # NEW
        if ((barrelCount != 0) and (legCount != 0)):
            #OLD
            # replace the points in the scan based off of the corners making a tangential plane to the obj
            for scanPoint in range(len(scan.ranges)):
                # if it is a barricade
                if (0 < legCount < 4): # changed from 3 to 4 # may be able to get of legLRCorners check
                    if (self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT) and legLRCorners[0] <= scanPoint:
                        scan.ranges[scanPoint] = legMinDist
                    elif (self.get_parameter('/FollowingDirection').value == DIRECTION.RIGHT) and scanPoint <= legLRCorners[1]:
                        scan.ranges[scanPoint] = legMinDist
                elif (legCount >= 4) and (legLRCorners[0]<= scanPoint <= legLRCorners[1]):
                    scan.ranges[scanPoint] = legMinDist
                # if it is 1+ barrel(s)
                if barrelLRCorners and (barrelLRCorners[barrel][0] - padding <= scanPoint <= barrelLRCorners[barrel][1] + padding):
                    scan.ranges[scanPoint] = barrelMinDists[barrel]
                    isBarrel = True
                elif (isBarrel == True) and ((barrel + 1) < barrelCount):
                    barrel = barrel + 1
                else:
                    isBarrel = False
            # OLD
            # NEW
            else:
                self.get_logger().info(f"Didn't find barrel or barricade")
                for scanPoint in range(len(scan.ranges)):
                    if (self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT) and pointMin - padding <= scanPoint:
                        scan.ranges[scanPoint] = minDist
                    elif (self.get_parameter('/FollowingDirection').value == DIRECTION.RIGHT) and scanPoint <= pointMax + padding:
                        scan.ranges[scanPoint] = minDist
            """
        if len(objPoints) != 0:
            for scanPoint in range(len(scan.ranges)):
                if (scan.ranges[point] > windowMax) or (scan.ranges[point] < windowMin):
                    if not (self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT) and scanPoint <= objPoints[-1]:
                        scan.ranges[scanPoint] = minDist
                    elif (self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT) and scanPoint >= objPoints[0]:
                        scan.ranges[scanPoint] = minDist
            self.get_logger().info(f"MinDist: {round(minDist, 5)}")
            self.get_logger().info(f"Min: {objPoints[0]}")
            self.get_logger().info(f"ObjP Size: {len(objPoints)}")
            self.get_logger().info(f"Max: {objPoints[-1]}")
        else:
            return

    # first portion nullifies all data behind the scanner after adjusting min and max to be 0
    # second portion adds potholes based on image data
    # third portion replaces obstacle in front and time of flight sensors

    # this funciton does the following:
    # 1. Shifts the angle and records scans that are in front of the robot
    # 2. Check/detect if there are obstacles/objects in front and in the FOV
    # 3. Publish the wheel distance to the obstacle based off the current line following direction
    def lidar_callback(self, scan):
        # self.get_logger().info(f"angle min: {scan.angle_min} \n angle max: {scan.angle_max} \n angle increment: {scan.angle_increment}")

        scan.angle_max += math.pi  # change the range from -pi -> pi to 0 -> 2 pi for array indexing
        scan.angle_min += math.pi

        new_ranges = []
        new_intensities = []

        startOffset = int(
            scan.angle_min / scan.angle_increment)  # where lidar started recording the scan in relation to min angle
        fullScan = int(math.pi / scan.angle_increment)
        try:
            if len(scan.intensities) > 0:
                i = 0
                while 0 <= i < startOffset:  # 0 is where the physical scan starts - don't record scan that are behind
                    i += 1
                    # self.get_logger().info(f"0 -> start off: 0 <= {i} < {startOffset}\n ")
                while startOffset <= i < fullScan:  # keep values in front of the robot
                    # self.get_logger().info(f"start off -> in front: start off <= {i} < {inFront}\n ")
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    new_intensities.append(scan.intensities[i - (startOffset + 1)])
                    # self.get_logger().info(f"Intensity")
                    i += 1
                scan.ranges = new_ranges
                scan.intensities = new_intensities
            else:  # same thing above just without intensities
                i = 0
                while 0 <= i < startOffset:
                    i += 1
                while startOffset <= i < fullScan:
                    new_ranges.append(scan.ranges[i - (startOffset + 1)])
                    i += 1
                scan.ranges = new_ranges
            scan.angle_min = 0.0  # radians = 0 degrees
            scan.angle_max = math.pi  # radians = 180 degrees
        except Exception:
            self.get_logger().info(
                f"ERROR: removing extraneous data broke ranges length: {len(scan.ranges)}, width: {width}")

        """
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
        """
        self.lidar_pub.publish(scan)

        # scan in the range in front of robot to check for obstacles
        msg = String()
        msg.data = STATUS.PATH_CLEAR
        count1 = 0
        if self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            follow_dist = self.get_parameter("/ObstacleDetectDistance").value *3/4
        else:
            follow_dist = self.get_parameter("/ObstacleDetectDistance").value


        half_FOV = self.get_parameter('/ObstacleFOV').value / 2
        right_FOV = (math.pi / 2) - half_FOV
        left_FOV = (math.pi / 2) + half_FOV
        for i in range(len(scan.ranges)):
            if right_FOV < i * scan.angle_increment < left_FOV:
                if scan.ranges[i] < follow_dist:
                    if count1 > 1:  # get at least two points of obstacle in front to trigger found
                        msg.data = STATUS.OBJECT_SEEN
                    count1 += 1
        # didn't see anything in front of the robot
        if msg.data == STATUS.PATH_CLEAR:
            self.update_history(0)
            # light_msg = LightCmd()
            # light_msg.type = 'G'
            # light_msg.on = True
            # self.lights_pub.publish(light_msg)

        else:
            self.update_history(1)
            # light_msg = LightCmd()
            # light_msg.type = 'G'
            # light_msg.on = False
            # self.lights_pub.publish(light_msg)

        if np.count_nonzero(self.history) >= 0.6 * self.BUFF_SIZE:
            if self.get_parameter('/Debug').value:
                self.get_logger().info("OBJECT_SEEN")
            self.path_clear = False
            msg.data = STATUS.OBJECT_SEEN
        elif np.count_nonzero(self.history) <= (1 - .6) * self.BUFF_SIZE and not self.path_clear:
            if self.get_parameter('/Debug').value:
                self.get_logger().info("PATH_CLEAR")
            self.path_clear = True
            msg.data = STATUS.PATH_CLEAR

        self.lidar_str_pub.publish(msg)

        # publish the wheel distance from the obstacle based on following direction
        distance_msg = String()
        try:
            if self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE or self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
                if self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT:
                    distance_msg.data = "OBJ," + str(
                        self.check_range(scan, 160 * math.pi / 180, 174 * math.pi / 180, 2.0))
                    # self.get_logger().info("Publishing from obstacles.py:")
                    # self.get_logger().info(f"Distance message data: {distance_msg}")
                    self.lidar_wheel_distance_pub.publish(distance_msg)
                elif self.get_parameter('/FollowingDirection').value == DIRECTION.RIGHT:
                    distance_msg.data = "OBJ," + str(self.check_range(scan, 6 * math.pi / 180, 20 * math.pi / 180, 2.0))
                    # self.get_logger().info("Publishing from obstacles.py:")
                    # self.get_logger().info(f"Distance message data: {distance_msg}")
                    self.lidar_wheel_distance_pub.publish(distance_msg)

        except Exception as e:
            self.get_logger().warning(f"ERROR with TOF Following direction : {e}")

    # will need to refactor when make a pothole solution
    # receives camera image and parses potholes into history
    def image_callback(self, image):
        image = bridge_image(image, "bgr8")
        # slice edges
        y, x = image.shape[0], image.shape[1]
        image = image[int(y * self.get_parameter('/PotholeDetectCropTop').value):-int(
            y * self.get_parameter('/PotholeDetectCropBottom').value),
                int(x * self.get_parameter('/PotholeDetectCropSide').value):-int(
                    x * self.get_parameter('/PotholeDetectCropSide').value)]

        # Apply HSV Filter
        gray = hsv_filter(image)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

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
        # *************************************************************
        t1 = time.time()
        kernel = np.ones((5, 5), np.uint8)
        ksize = (5, 5)

        # Convert the image to grayscale
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Invert the image (black becomes white, white becomes black)
        img_inv = np.array(ImageOps.invert(im.fromarray(img_gray)))

        # Color the non-black spots more white
        factor = 1.5  # Change this factor to adjust the degree of whitening
        img_white = ImageOps.autocontrast(im.fromarray(img_inv), cutoff=0, ignore=255).point(lambda i: i * factor)
        numpydata = np.array(img_white)

        # removing other componets
        (thresh, blackAndWhiteImage) = cv2.threshold(numpydata, 127, 255, cv2.THRESH_BINARY)

        # HSV filtering
        dilation = cv2.dilate(blackAndWhiteImage, kernel, iterations=2)
        erosion = cv2.erode(dilation, kernel, iterations=2)
        # closing = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel)

        # Apply HoughCircles to detect circles
        circles = cv2.HoughCircles(erosion, cv2.HOUGH_GRADIENT_ALT, 1, 1, param1=100, param2=0.1, minRadius=60,
                                   maxRadius=0)
        color = (0, 255, 0)
        markerType = cv2.MARKER_CROSS
        markerSize = 20
        thickness = 10

        im_rgb = cv2.cvtColor(erosion, cv2.COLOR_BGR2RGB)
        im_rgb_withMarker = im_rgb
        #t2 = time.time()

        # Draw detected circles on the original image
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            self.pothole_found=True
            msg = String()
            msg.data = STATUS.POTHOLE_FOUND
            self.image_pub.publish(msg)
            for (x, y, r) in circles:
                cv2.circle(im_rgb, (x, y), r, (0, 0, 0), 2)
                # self.get_logger().info((x, y, r))
                cv2.drawMarker(im_rgb_withMarker, (x, y), color, markerType, markerSize, thickness)
        else:
            self.pothole_found = False

        if self.pothole_found:
            if self.get_parameter('/Debug').value:
                self.get_logger().info("POTHOLE_FOUND")

        t3 = time.time()
        morph = im_rgb_withMarker
        # *************************************************************
        """
        Detection method blobs is used in 2021 to 2022 team.
        """
        """
        detector = cv2.SimpleBlobDetector_create(params) 
        keypoints = detector.detect(morph)  # find the blobs meeting the parameters
        self.circles = []
        for hole in keypoints:
            self.circles.insert(0, Circle(hole.pt[0], hole.pt[1], hole.size // 2))

        if self.get_parameter('/Debug').value:
            blobs = cv2.drawKeypoints(morph, keypoints, np.zeros((1, 1)), (0, 255, 0),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #cv_display(blobs, 'Potholes', self.window_handle)
        """
        # cv_display(morph, 'Potholes', self.window_handle)
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