#pragma once
#include "robot_info/hydraulic_system_monitor.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include <string>

class RobotInfo {

private:
  ros::NodeHandle *nh;
  void init_publisher();

protected:
  std::string robot_description;
  std::string serial_number;
  std::string ip_address;
  std::string firmware_version;
  ros::Publisher ros_pub;

public:
  RobotInfo(ros::NodeHandle *node_handle, std::string robot_desp,
            std::string serial_number, std::string ip_addr,
            std::string firmware_version);
  virtual void publish_data();
};

class AGVRobotInfo : public RobotInfo {

private:
  std::string maximum_payload;
  HydraulicSystemMonitor h_monitor;

public:
  AGVRobotInfo(ros::NodeHandle *node_handle, std::string robot_desp,
               std::string serial_number, std::string ip_addr,
               std::string firmware_version, std::string load,
               const HydraulicSystemMonitor &h_monitor);

  virtual void publish_data() override;
};
