/*
This package creates a graphical user interface featuring a button and a window.
When the button is clicked, a ROS service call of type `std_srvs/Trigger.h` is
sent to the service server and the response message is printed inside the
window.
*/
#define CVUI_IMPLEMENTATION
#include "trigger_msg_service_client_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_srvs/Trigger.h> // Service message type

class CVUIROSTriggerMsgServiceClient {
public:
  CVUIROSTriggerMsgServiceClient(const std::string &srv_name);

  void run();

private:
  ros::NodeHandle nh;
  ros::ServiceClient service_client;
  // Create a service request
  std_srvs::Trigger srv_req;
  std::string service_name;
  std::string last_service_call_msg;
  int service_call_counter = 0;
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SERVICE CLIENT";
};

CVUIROSTriggerMsgServiceClient::CVUIROSTriggerMsgServiceClient(
    const std::string &srv_name) {
  // Initialize ROS node
  ros::NodeHandle nh;
  // Create a service client that sends requests of type std_srvs/Trigger
  service_client = nh.serviceClient<std_srvs::Trigger>(srv_name);
  service_name = srv_name;
}

void CVUIROSTriggerMsgServiceClient::run() {
  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (40, 20) with size 460x80 (width x height) and title
    cvui::window(frame, 40, 20, 420, 40, "Service: " + service_name);

    // Call the service
    if (cvui::button(frame, 45, 80, "Call Service")) {
      // Send the request and wait for a response
      if (service_client.call(srv_req)) {
        // Print the response message and return true
        ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        // set latest service call status
        last_service_call_msg = srv_req.response.message;
        service_call_counter++;
      } else {
        last_service_call_msg = "Service call failed.";
        service_call_counter = 0;
      }
    }

    // Display the last response inside the window
    if (not last_service_call_msg.empty()) {
      cvui::printf(frame, 45, 45, 0.4, 0xff0000, "%s",
                   last_service_call_msg.c_str());
    }

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "trigger_msg_srv_client_gui");
  // Replace with the name of the service you want to call
  CVUIROSTriggerMsgServiceClient simple_srv_client_gui(
      "minimal_trigger_msg_service");
  simple_srv_client_gui.run();
  return 0;
}
