#include "robot_gui/robot_gui.h"
//#include "robot_gui/cvui.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"

RobotCVUI::RobotCVUI() {

  // call robot_info node
  sub_topic = "/robot_info";
  pub_topic = "/cmd_vel";
  sub = nh.subscribe(sub_topic, 1000, &RobotCVUI::info_callback, this);
  pub = nh.advertise<geometry_msgs::Twist>(pub_topic, 2);
  sub_odom = nh.subscribe("/odom", 10, &RobotCVUI::odom_callback, this);

  vel.linear.x = 0;
  vel.angular.z = 0;
  pub.publish(vel);

  dist_tracker = nh.serviceClient<std_srvs::Trigger>("/get_distance");
  ROS_INFO("GUI Initialized");
}

void RobotCVUI::info_callback(const robotinfo_msgs::RobotInfo10Fields &info) {
  // ROS_INFO("callback enabled");
  // ros::Rate loop_rate(10);
  robot_description = info.data_field_01;
  serial_number = info.data_field_02;
  ip_address = info.data_field_03;
  firmware_version = info.data_field_04;
  maximum_payload = info.data_field_05;
  hydraulic_oil_temperature = info.data_field_06;
  hydraulic_oil_tank_fill_level = info.data_field_07;
  hydraulic_oil_pressure = info.data_field_08;
  // ROS_INFO("message received: %s", robot_description.c_str());
  // loop_rate.sleep();
}

void RobotCVUI::odom_callback(const nav_msgs::Odometry &odom) {
  pos_x = odom.pose.pose.position.x;
  pos_y = odom.pose.pose.position.y;
  pos_z = odom.pose.pose.position.y;
}

void RobotCVUI::run() {

  cvui::init(WINDOW_NAME);

  cv::Mat frame = cv::Mat(cv::Size(400, 800), CV_8UC3);

  while (ros::ok()) {

    frame = cv::Scalar(49, 52, 49);

    // Info window
    cvui::window(frame, 5, 5, 390, 210, "Info");
    cvui::printf(frame, 9, 25, 0.4, 0x00ff00, robot_description.c_str());
    cvui::printf(frame, 9, 50, 0.4, 0x00ff00, serial_number.c_str());
    cvui::printf(frame, 9, 75, 0.4, 0x00ff00, ip_address.c_str());
    cvui::printf(frame, 9, 100, 0.4, 0x00ff00, firmware_version.c_str());
    cvui::printf(frame, 9, 125, 0.4, 0x00ff00, maximum_payload.c_str());
    cvui::printf(frame, 9, 150, 0.4, 0x00ff00,
                 hydraulic_oil_temperature.c_str());
    cvui::printf(frame, 9, 175, 0.4, 0x00ff00,
                 hydraulic_oil_tank_fill_level.c_str());
    cvui::printf(frame, 9, 200, 0.4, 0x00ff00, hydraulic_oil_pressure.c_str());

    if (cvui::button(frame, 135, 230, 130, 70, "Forward")) {

      vel.linear.x += linear_velocity_step;
      pub.publish(vel);
    }

    if (cvui::button(frame, 135, 305, 130, 70, "Stop")) {

      vel.linear.x = 0;
      vel.angular.z = 0;
      pub.publish(vel);
    }

    if (cvui::button(frame, 0, 305, 130, 70, "Left")) {

      vel.angular.z += angular_velocity_step;
      pub.publish(vel);
    }

    if (cvui::button(frame, 270, 305, 130, 70, "Right")) {

      vel.angular.z -= angular_velocity_step;
      pub.publish(vel);
    }

    if (cvui::button(frame, 135, 380, 130, 70, "Backward")) {

      vel.linear.x -= linear_velocity_step;
      pub.publish(vel);
    }

    cvui::window(frame, 0, 460, 190, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 50, 485, 0.4, 0xff0000, "%.02f m/sec", vel.linear.x);

    cvui::window(frame, 197, 460, 190, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 250, 485, 0.4, 0xff0000, "%.02f rad/sec",
                 vel.angular.z);

    cvui::text(frame, 5, 505, "Estimated robot position based off odometry");
    cvui::window(frame, 0, 525, 130, 90, "X");
    cvui::printf(frame, 10, 590, 0.7, 0xff0000, "%f", pos_x);

    cvui::window(frame, 135, 525, 130, 90, "Y");
    cvui::printf(frame, 135, 590, 0.7, 0xff0000, "%f", pos_y);

    cvui::window(frame, 270, 525, 130, 90, "Z");
    cvui::printf(frame, 270, 590, 0.7, 0xff0000, "%f", pos_z);

    cvui::text(frame, 5, 620, "Distance travelled");

    if (cvui::button(frame, 0, 640, 130, 90, "Call")) {

      std_srvs::TriggerResponse res;
      std_srvs::TriggerRequest req;
      if (dist_tracker.call(req, res)) {
        // ROS_INFO("Service called: %s", res.success ? "success" : "fail");
        distance_travelled = res.message;
      } else {
        ROS_ERROR("Failed to call service");
      }
    }

    cvui::window(frame, 135, 640, 255, 90, "Distance in meters:");
    cvui::printf(frame, 300, 700, 0.8, 0xff0000, "%s",
                 distance_travelled.c_str());

    cvui::update();
    cv::imshow(WINDOW_NAME, frame);
    ros::spinOnce();
    if (cv::waitKey(20) == 27) {
      break;
    }
  }
}
