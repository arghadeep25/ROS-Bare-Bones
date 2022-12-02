/************************************
* @file ros_barebones_nodelet.cpp
* @details ROS Bare Bones Nodelet
* @author Arghadeep Mazumder
* @version 1.0.0
* @copyright -
************************************/

#include <iostream>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <utils.hpp>

namespace ros_bare_bones {
class ROSBareBonesNodelet : public nodelet::Nodelet {

public:
  // Default Constructor
  ROSBareBonesNodelet() = default;
public:
  // Default Destructor
  ~ROSBareBonesNodelet() override = default;

private:
  /**
   * @brief init function
   */
  void onInit() override {
    // ROS nodelet info with function name
    NODELET_INFO("ROSBareBonesNodelet - %s", __FUNCTION__);
    message_callback();
  }

private:
  /**
   * @brief Message Callback function
   */
  void message_callback() {
    // ROS nodelet info with function name
    NODELET_INFO("ROSBareBonesNodelet - %s", __FUNCTION__);
    // Setting the frequency at which message will be displayed
    ros::Rate rate(2);
    // Infinite Loop where the message will be printed
    while(ros::ok()){
      // Printing the message
      MESSAGE("Counter: ",counter);
      // Using ROS sleep function
      rate.sleep();
      // Incrementing the counter
      counter++;
    }
  }

private:
  int counter = 0;
};
PLUGINLIB_EXPORT_CLASS(ros_bare_bones::ROSBareBonesNodelet, nodelet::Nodelet)
} // namespace ros_bare_bones