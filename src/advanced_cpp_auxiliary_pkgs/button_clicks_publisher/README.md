# The button_clicks_publisher package

Author: Roberto Zegers  
Date: February 2023

## Descrition

This package creates a graphical user interface featuring a button and text.
When the button is clicked, the code increments a counter and publishes a
message with the current count to a ROS topic.

## Usage

- Clone to your ROS workspace, for instance into `~/catkin_ws/src`  
- Compile and source: `cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Execute: `rosrun button_clicks_publisher simple_button_clicks_publisher_node`  

## License
- BSD-3-Clause
- CVUI Library: Copyright (c) 2016 Fernando Bevilacqua. Licensed under the MIT license.

## Acknowledgement
- I was inspired by this post: [cvui: A GUI lib built on top of OpenCV drawing primitives](https://learnopencv.com/cvui-gui-lib-built-on-top-of-opencv-drawing-primitives/)  