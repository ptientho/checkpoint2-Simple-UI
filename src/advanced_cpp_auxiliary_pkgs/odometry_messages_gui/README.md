# The odometry_messages_gui package

Author: Roberto Zegers  
Date: February 2023

## Description

This package creates a minimal graphical user interface featuring a window and text. 
When a ROS message of type `nav_msgs/Odometry` is received, the gui displays the incomming odometry data inside the window.


## Usage

- Clone to your ROS workspace, for instance into `~/catkin_ws/src`  
- Compile and source: `cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Make sure `roscore` is running
- Execute: `rosrun odometry_messages_gui odometry_messages_gui_node`  

## Manual testing

- This will publish message of type `nav_msgs/Odometry` with a position of x: 1.0, y: 2.0, z: 3.0 and no rotation on the `/odom` topic at a rate of 1 Hz:  
```
rostopic pub -r 1 /odom nav_msgs/Odometry '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "odom"}, child_frame_id: "base_link", pose: {pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

## License
- BSD-3-Clause
- CVUI Library: Copyright (c) 2016 Fernando Bevilacqua. Licensed under the MIT license.

## Dependencies
- ROS Noetic
- OpenCV
- [CVUI](https://github.com/Dovyski/cvui)