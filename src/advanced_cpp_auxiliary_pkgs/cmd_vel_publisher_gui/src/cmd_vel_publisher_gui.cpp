/*
The CVUIROSCmdVelPublisher class creates a graphical user interface featuring
five buttons to move the robot forward, back, left, right and stop.
The program then publishes to cmd_vel a message of type Twist.

Author: Roberto Zegers
Date: February 2023
License: BSD-3-Clause
*/

#define CVUI_IMPLEMENTATION
#include "cmd_vel_publisher_gui/cvui.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();

  void run();

private:
  ros::Publisher twist_pub_;
  // Create Twist message
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  const std::string WINDOW_NAME = "CVUI ROS TELEOP";
};

CVUIROSCmdVelPublisher::CVUIROSCmdVelPublisher() {
  // Initialize ROS node
  ros::NodeHandle nh;
  twist_topic_name = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);
}

void CVUIROSCmdVelPublisher::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Clear the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Show a button at position x = 100, y = 20
    if (cvui::button(frame, 100, 20, " Forward ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 50
    if (cvui::button(frame, 100, 50, "   Stop  ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 30, y = 50
    if (cvui::button(frame, 30, 50, " Left ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 195, y = 50
    if (cvui::button(frame, 195, 50, " Right ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 80
    if (cvui::button(frame, 100, 80, "Backward")) {
      // The button was clicked,update the Twist message
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 20, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 45, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);

    // Create window at (320 60) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 60, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 85, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cmd_vel_teleop_gui");
  CVUIROSCmdVelPublisher button_clicks_publisher;
  button_clicks_publisher.run();
  return 0;
}
