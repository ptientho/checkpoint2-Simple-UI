#include "odometry_messages_gui/odometry_messages_gui.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_messages_gui");
  CVUIROSOdomSubscriber odometry_messages_gui;
  odometry_messages_gui.run();
  return 0;
}