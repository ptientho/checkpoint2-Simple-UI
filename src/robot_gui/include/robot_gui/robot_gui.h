#pragma once
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <string>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
//#include <opencv2/opencv.hpp>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"

class RobotCVUI {

public:
  RobotCVUI();
  void run();
  void info_callback(const robotinfo_msgs::RobotInfo10Fields &info);
  void odom_callback(const nav_msgs::Odometry &odom);

private:
  const std::string WINDOW_NAME = "MY WINDOW";
  ros::NodeHandle nh;
  std::string sub_topic;
  std::string pub_topic;
  ros::Subscriber sub;
  ros::Subscriber sub_odom;
  ros::Publisher pub;
  geometry_msgs::Twist vel;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;

  std::string robot_description;
  std::string serial_number;
  std::string ip_address;
  std::string firmware_version;
  std::string maximum_payload;
  std::string hydraulic_oil_pressure;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_temperature;
  float pos_x;
  float pos_y;
  float pos_z;

  ros::ServiceClient dist_tracker;
  std::string distance_travelled = "";
};