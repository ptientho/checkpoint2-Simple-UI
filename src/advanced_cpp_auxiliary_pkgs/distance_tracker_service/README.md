# The distance_tracker_service package

- Author: Roberto Zegers

## Description

This is a ROS node that subscribes to a message of type `nav_msgs/Odometry` 
and uses the incoming data to calculate the distance traveled which is 
stored on a member variable. The node also has a service server of type 
`std_srvs/Trigger.h` which sends a message with the distance traveled each
time a request is received.

## Usage
- Clone to your ROS workspace, for instance into `~/catkin_ws/src`  
- Compile and source: `cd ~/catkin_ws; catkin_make; source devel/setup.bash`  
- Make sure a robot simulation is running  
- Execute: `rosrun distance_tracker_service distance_tracker_service`  


## Manual testing
- Open a new terminal window and execute:  
```
rosservice call /get_distance "{}"
```
The output should be similar to the following:  
```
success: True
message: "7.52"
```

## License
- BSD-3-Clause

## Dependencies
- ROS Noetic
