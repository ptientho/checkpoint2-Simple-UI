/*
The CVUIROSOdomSubscriber class creates a graphical user interface featuring a
window and text. When a ROS message of type 'nav_msgs/Odometry.h' is received,
the gui displays the incomming x,z,y position inside the window.

To use this class, include the odometry_messages_gui.h header file in your
main program file and create an instance of the CVUIROSOdomSubscriber class. You
would then call the 'run' function on the instance to start the program.

Author: Roberto Zegers
Date: February 2023
License: BSD-3-Clause
*/

#include "odometry_messages_gui/odometry_messages_gui.h"

CVUIROSOdomSubscriber::CVUIROSOdomSubscriber() {
  // Initialize ROS node
  ros::NodeHandle nh;
  topic_name = "odom";
  sub_ = nh.subscribe<nav_msgs::Odometry>(
      topic_name, 2, &CVUIROSOdomSubscriber::msgCallback, this);
}

void CVUIROSOdomSubscriber::msgCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  data = *msg;
  ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void CVUIROSOdomSubscriber::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Clear the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 40, 20, 250, 40, "Topic: " + topic_name);

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, 45, 45, 0.4, 0xff0000,
                 "Data received: [%0.2f, %0.2f, %0.2f]",
                 data.pose.pose.position.x, data.pose.pose.position.y,
                 data.pose.pose.position.z);

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}