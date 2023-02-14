# Cedarville University AutoNav Competition Robot 2023
This repository contains the code used for the Cedarville AutoNav 2023 robot. The robot is run through ROS2 nodes and takes a behavioral approach through a state machine.

## Quick Start
1. Set the GPS waypoint locations in the yaml. These will assume to be first an exit point from lines, then an entrance, and repeat.
   Set these by running the dms_to_dmm.py script to convert the judge provided values to our Degrees Minutes Minutes Decimal format.
2. Run `python3 start_motors.py`. You will have to start the motors by enabling the motors (turn off estop) when prompted.
2. Enter the GPS heading given by the phone compass. Change this value in the params.yaml in degrees. Set the other parameters as appropriate. Then start by either following step 7 or steps 3-6
3. Source ROS2 `. /opt/ros/<ros2_distro>/setup.bash`
4. Navigate to `cd ros2_ws` and build with `colcon build`
5. Source the workspace in all relevant terminals with `. install/setup.bash`
6. Run `ros2 launch robot_launch robot.launch.py`. If this doesn't run or throws errors, view the robot.launch.py file to modify the port parameters.
7. Alternatively to steps 3-6, run `./vision.sh` and `./control.sh` in the ros2_ws in separate terminals.
8. (Optional) if you want to see the laser scan, run the following commands in new terminals:
- `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world laser_frame`
- `rviz2 ./install/rplidar_ros/share/rplidar_ros/rviz/rplidar.rviz`

### Important Parameters (found at top of params.yaml)
- /RealCourse:                True if on actual course, false if practice
- /FollowingDirection:        1  is right, 0 is left
- /UseYellow:                 False if following white lines
- /InitialHeading:            154.0  set before build to degrees based on iphone compass
- /NorthPointFirst:           set depending on direction going in course
- /GpsExitHeading:            comment shows what value to set based on going north south, this will have to be tweaked at location for return to line
- /CrossRampInGps:            True for cross in GPS or False for line following - line following untested
- /RepeatGps:                 False for real course, True to stay in GPS mode and repeat the waypoints indefinitely - used for testing/debug
- /LineDist:                  distance in meters to follow from line

## Packages and Nodes
### Master FSM
The master fsm node resides in the master_fsm package. This controls the conditions for switching between states as well as sends commands to the motor controllers. 
It can be run individually from the launch file with the command `ros2 run master_fsm fsm -p <parameter sets>`. Parameters available include:
- `/StartState` which can be selected from the STATE enum in the utils package.
- `/DefaultSpeed` which ranges from 0 to 64 but for stability has best results near 15.
- `/FollowingDirection` which indicates whether the robot should left or right line follow and is of value 0 or 1
- `/TimerRate` which sets how frequently the state change is checked in seconds.

### Utils
The utils package is not run directly but used by many of the nodes. It contains the state enums, various useful constants and functions, and custom messages.

### Custom Messages
The custom_msgs package is also not run directly but is used by the nodes. It messages are primarily for the encoders and GPS fusion.

### Launch
The robot_launch package contains the launch machine for the entire robot set of packages. Parameters can be set in the robot.launch.py file. To use defaults from the test machine, run `ros2 launch robot_launch robot.launch.py`

### GPS
The GPS node `gps_publisher` belongs to the `heading` package which monitors the NMEA stream from the ZED F9P GPS and publishes the location data to `gps_events` and `gps_heading` where the fusion node and the master state machine use these to maintain heading. 
- `/WaypointLat1` which sets the latitude of the first waypoint. These go up to four points.
- `/WaypointLon1` which sets the longitude of the waypoints, changing number for points
- `/ExitAngle`
- `/GPSFollowGoal` for how close to get to the point before considering it an arrival
- `/LineToGPSTrans`
- `/Port` which is the port the GPS is on 

### Teensy/Encoders
The encoders publish to the `encoder_data` topic. They can track the distance the wheels travel to the centimeter and report the change since last message. This node is extremely important it actually sends the motor messages to the wheels and also manages the lights. This was merged with the motor controller to save resources and time. Additional params for separating speeds have been added
- `/TeensyEncodersPort`
- `/TeensyBaudrate`
- `TeensyUpdateDelay` - sets the time between collection data from the encoders
- `/Debug`
- `FollowingDirection` - generic parameter
- `/LineDist` - manages desired distance from line
- `/SideObjectDist` - manages desired distance from object
- `/DefaultSpeed` - sets to simple speed when clear path or straight. This is being modified to include varied speed for different states - OUTDATED
- `/BoostIncrease'`
- `/BoostCountThreshold`
- `/LineBoostMargin`
- `/GPSBoostMargin`
- `/Port`


### Fusion for Heading
The fusion node publishes to the wheels and can publish its own `gps_event` messages. It basically weights the encoder and GPS provided headings to maintain on course.
- `/InitialHeading'` sets the initial angle relative to lat/long
- `/EncoderWeight` which weights the encoder versus the gps heading
- `/Debug`

### Lines
The lines node belongs to the path_detection package. It detects and maintains line following. The line parameters modify the camera cropping, color to follow, and following direction.
- `/LineDetectionCropTop` which cuts off a fraction of the top of the image
- `/LineDetectionCropBottom`
- `/LineDetectionCropSide` which crops both sides
- `/FollowingDirection` which indicates left and right wall following
- `/UseYellow` which allows yellow line following instead of white for outdoor parking lot tests
- `/Debug` which enables some of the debug output

### Obstacle Detection
The obstacles node belongs to the path_detection package. It is responsible for mapping vision camera potholes onto the LIDAR scan and publishing this obstacle data. Its parameters are similar to the lines node with some additional obstacle limits.
- `/LIDARTrimMin` which sets the left edge on the unit circle of where to cut off the scan for the robot body
- `/LIDARTrimMax` which sets the max angle of which to cut off
- `/ObstacleFOV` which sets the area in front of the robot to consider a dangerous obstacle
- `/PotholeDetectCropTop`
- `/PotholeDetectCropBottom`
- `/PotholeDetectCropSide`
- `/PotholeBufferSize` which maintains the history size used for the pothole conclusive detection
- `/ObstacleDetectDistance` which says how many meters at which to detect obstacles
- `/Debug` to display debug data

### Realsense Camera
The RealSense camera is run through a provided package. It reports on the `/camera/color/image_raw` topic which is received by the vision nodes. If desiring to run it outside of the master launch file, run `ros2 launch realsense2_camera rs_launch.py`

### RPLidar
The RPLIDAR also is controlled through a provided package. It spins and publishes the LIDAR scans. You may have to set the port with the parameters:
- `serial_port` which is by default `/dev/ttyUSB0`

### Data Logger
The data_logger node can subscribe to any topic and save the data to a CSV. To run the data_logger node, run `./log_data.sh`. Right now choosing which topics to subscribe to happens by editing the node itself. Potential future work would be to create a section in the parameters file to have a cleaner topic selection interface.

## Helpful Tools/Tips
- If wanting to use ROS1 versions of packages or rosbags see the helpful ros1 to ros2 bridge at [https://github.com/ros2/ros1_bridge](RosBridge)
- To change parameters while running see `ros2 param set <node> <parameter> <parameter_value>
- Some of these parameters may have been moved into the general parameter section.
- ROSbags have been taken on the actual course and can be found in the bags folder. These contain the camera, lidar, GPS, and other sensor inputs that can be run through modified systems. They may be on a separate drive as they are very large.
- GPS points on actual course may be slightly off. We did some shifting of the waypoints that would vary by run
- Units throughout are mostly in meters, exceptions being some of the line node reported data.
- We strongly recommend redoing the ROS messages for state and motor commands so they are not stupid string messages that were inherited
- Buy a new non SmartDriveDuo motor controller as this one was finnicky and randomly glitched with overcurrent
- Add rubber shielding around corners
- Shift payload to sit between the wheels so jolts don't tip it
- Get more batteries and a better interchangeablity so can charge cycle and test cycle at same time
- The heading package is technically more of the driving package as it controls the motors too.
- See quick start for ways to start the nodes

## How the Robot Tried to Kill Us and Itself
- poor manual fine control at low speeds - running us over out the gate
- can't see white line in hallway with white walls - drove directly at wall and pinned to wall
- slippage on ramp of encoder given heading - direct suicide off ramp attempt
- following obstacles too closely - tail spun and hit us
- connecting the controller (which requires the joystick to be in the 6:00 position) without the e-stop on - fully engaging the motors and punching a hole through the wall

## [Complete Video](https://drive.google.com/file/d/1BnwrCk68dNjJJOYNAAvbYWItyapaKm5W/view?usp=sharing)
