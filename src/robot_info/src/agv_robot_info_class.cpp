#include "robot_info/hydraulic_system_monitor.h"
#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <string>

AGVRobotInfo::AGVRobotInfo(ros::NodeHandle *node_handle, std::string robot_desp,
                           std::string serial_number, std::string ip_addr,
                           std::string firmware_version, std::string load,
                           const HydraulicSystemMonitor &h_monitor)
    : RobotInfo::RobotInfo(node_handle, robot_desp, serial_number, ip_addr,
                           firmware_version),
      maximum_payload(load), h_monitor(h_monitor) {}

void AGVRobotInfo::publish_data() {

  robotinfo_msgs::RobotInfo10Fields data;
  /*
data.data_field_01 = "robot_description: " + robot_description;
data.data_field_02 = "serial_number: " + serial_number;
data.data_field_03 = "ip_address: " + ip_address;
data.data_field_04 = "firmware_version: " + firmware_version;
data.data_field_05 = "maximum_payload: " + maximum_payload;
  */
  data.data_field_01 = "robot_description: " + robot_description;
  data.data_field_02 = "serial_number: " + serial_number;
  data.data_field_03 = "ip_address: " + ip_address;
  data.data_field_04 = "firmware_version: " + firmware_version;
  data.data_field_05 = "maximum_payload: " + maximum_payload;
  data.data_field_06 =
      "hydraulic_oil_temperature: " + h_monitor.get_oil_temperature();
  data.data_field_07 =
      "hydraulic_oil_tank_fill_vevel: " + h_monitor.get_oil_tank_fill_level();
  data.data_field_08 =
      "hydraulic_oil_pressure: " + h_monitor.get_oil_pressure();

  ros_pub.publish(data);
}