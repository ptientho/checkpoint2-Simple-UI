/*
The CVUIROSPublisher class creates a graphical user interface featuring a button
and text. When the button is clicked, the code increments a counter and
publishes a message with the current count to a ROS topic.

To use this class, include the simple_button_clicks_publisher.h header file in
your main program file and create an instance of the CVUIROSPublisher class. You
would then call the 'run' function on the instance to start the program.

Author: Roberto Zegers
Date: February 2023
License: BSD-3-Clause
*/

#include "button_clicks_publisher/simple_button_clicks_publisher.h"

CVUIROSPublisher::CVUIROSPublisher() {
  // Initialize ROS node
  ros::NodeHandle nh;
  pub_ = nh.advertise<std_msgs::String>("cvui_button_clicks", 10);
}

void CVUIROSPublisher::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);
  int count = 0;

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Show a button at position x = 40, y = 80
    if (cvui::button(frame, 40, 80, "Click me!")) {
      // The button was clicked, so let's increment our counter and publish a
      // message.
      count++;
      std_msgs::String msg;
      msg.data = "Button clicked " + std::to_string(count) + " times.";
      pub_.publish(msg);
    }

    // Create window at (220, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 220, 20, 250, 40, "Click Count");

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, 225, 45, 0.4, 0xff0000, "Button click count: %d",
                 count);

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
