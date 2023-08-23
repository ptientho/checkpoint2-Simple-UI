#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include <string>

RobotInfo::RobotInfo(ros::NodeHandle *node_handle, std::string robot_desp,
                     std::string serial_number, std::string ip_addr,
                     std::string firmware_version)
    : nh(node_handle), robot_description(robot_desp),
      serial_number(serial_number), ip_address(ip_addr),
      firmware_version(firmware_version) {

  // ROS_INFO("robot_description: %s", robot_description.c_str());

  init_publisher();
}

void RobotInfo::init_publisher() {

  ros_pub =
      nh->advertise<robotinfo_msgs::RobotInfo10Fields>("/robot_info", 1000);
  ROS_INFO("Enabled robot_info publisher");
}

void RobotInfo::publish_data() {

  robotinfo_msgs::RobotInfo10Fields data;
  /*
data.data_field_01 = "robot_description: " + robot_description;
data.data_field_02 = "serial_number: " + serial_number;
data.data_field_03 = "ip_address: " + ip_address;
data.data_field_04 = "firmware_version: " + firmware_version;
  */

  data.data_field_01 = robot_description;
  data.data_field_02 = serial_number;
  data.data_field_03 = ip_address;
  data.data_field_04 = firmware_version;

  ros_pub.publish(data);
}