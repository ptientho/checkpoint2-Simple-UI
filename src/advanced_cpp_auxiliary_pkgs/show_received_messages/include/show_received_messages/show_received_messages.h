#pragma once

#define CVUI_IMPLEMENTATION
#include "show_received_messages/cvui.h"
#include "std_msgs/Float64.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSSubscriber {
public:
  CVUIROSSubscriber();

  void run();

private:
  ros::Subscriber sub_;
  std_msgs::Float64 data;
  std::string topic_name;
  void msgCallback(const std_msgs::Float64::ConstPtr &msg);
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};