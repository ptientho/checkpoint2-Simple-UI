# The show_received_messages package

Author: Roberto Zegers  
Date: February 2023

## Description

This package creates a minimal graphical user interface featuring a window and text. 
When a ROS message is received, the gui displays the incomming data inside the window.


## Usage

- Clone to your ROS workspace, for instance into `~/catkin_ws/src`  
- Compile and source: `cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Make sure `roscore` is running
- Execute: `rosrun show_received_messages show_received_messages_node`  

## License
- BSD-3-Clause
- CVUI Library: Copyright (c) 2016 Fernando Bevilacqua. Licensed under the MIT license.

## Dependencies
- ROS Noetic
- OpenCV
- [CVUI](https://github.com/Dovyski/cvui)