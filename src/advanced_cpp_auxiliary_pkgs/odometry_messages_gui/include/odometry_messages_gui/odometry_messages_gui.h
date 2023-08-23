#pragma once

#define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "odometry_messages_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSOdomSubscriber {
public:
  CVUIROSOdomSubscriber();

  void run();

private:
  ros::Subscriber sub_;
  nav_msgs::Odometry data;
  std::string topic_name;
  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};