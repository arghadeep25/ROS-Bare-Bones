#include <iostream>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <utils.hpp>

namespace ros_bare_bones {
class ROSBareBonesNodelet : public nodelet::Nodelet {

public:
  ROSBareBonesNodelet() = default;
  ~ROSBareBonesNodelet() override = default;

private:
  /**
   *
   */
  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    pub = nh.advertise<std_msgs::String>("chatter", 1000);
  }

private:
  void message_callback() {
    std_msgs::String message;
    std::stringstream sstream;
    sstream << "hello world";
    message.data = sstream.str();
    ROS_INFO("%s", message.data.c_str());
    pub.publish(message);
  }

private:
  // ROS Node Handler
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  // Subscriber and Publisher
  ros::Subscriber sub;
  ros::Publisher pub;
};
PLUGINLIB_EXPORT_CLASS(ros_bare_bones::ROSBareBonesNodelet, nodelet::Nodelet)
} // namespace ros_bare_bones