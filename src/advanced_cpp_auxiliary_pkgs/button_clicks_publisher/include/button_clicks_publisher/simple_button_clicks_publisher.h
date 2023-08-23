#pragma once

#define CVUI_IMPLEMENTATION
#include "button_clicks_publisher/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

class CVUIROSPublisher {
public:
  CVUIROSPublisher();

  void run();

private:
  ros::Publisher pub_;
  const std::string WINDOW_NAME = "CVUI ROS BUTTON!";
};