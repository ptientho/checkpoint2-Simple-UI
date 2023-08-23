#include "robot_info/robot_info_class.h"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "RobotManager");
  ros::NodeHandle nh;

  RobotInfo robot_info1(&nh, "a", "1111", "172.168.1.1", "v_01");

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    robot_info1.publish_data();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}