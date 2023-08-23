//#include "robot_gui/cvui.h"
#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "Robot_gui");

  RobotCVUI my_gui;
  my_gui.run();

  return 0;
}