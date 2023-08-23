#include "robot_info/hydraulic_system_monitor.h"
#include "robot_info/robot_info_class.h"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "AGVRobotManager");
  ros::NodeHandle nh;

  HydraulicSystemMonitor hydraulic_system_monitor("45C", "100%", "25 bar");
  AGVRobotInfo robot_info1(&nh, "Mir100", "567A359", "169.254.5.180", "3.5.8",
                           "100 Kg", hydraulic_system_monitor);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    robot_info1.publish_data();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}