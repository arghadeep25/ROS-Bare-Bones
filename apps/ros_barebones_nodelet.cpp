/************************************
 * @file ros_barebones_nodelet.cpp
 * @details ROS Bare Bones Nodelet
 * @author Arghadeep Mazumder
 * @version 1.0.0
 * @copyright -
 ************************************/

#include <iostream>
#include <random>

#include <utils.hpp>

#include <nodelet/nodelet.h>
#include <pcl/filters/voxel_grid.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

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
    nh = getNodeHandle();
    points_callback();
  }

private:
  /**
   * @brief Points Callback function
   */
  void points_callback() {
    // ROS nodelet info with function name
    NODELET_INFO("ROSBareBonesNodelet - %s", __FUNCTION__);
    sizeOfCloud = nh.param<int>("point_cloud_size", 8000);
    centreX = nh.param<float>("centreX", 0.0);
    centreY = nh.param<float>("centreY", 0.0);
    frequency = nh.param<int>("frequency", 10);

    sensor_msgs::PointCloud cloud;
    cloud.points.resize(sizeOfCloud);

    cloud.header.frame_id = "base_link";
    cloud.header.stamp = ros::Time::now();

    auto keypoints_publisher =
        nh.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

    ros::Rate rate(10);

    while (ros::ok()) {
      MESSAGE("Streaming: ", counter);
      getRandomPointCloud(cloud, centreX, centreY);
      cloud.header.stamp = ros::Time::now();
      keypoints_publisher.publish(cloud);
      ros::spinOnce();
      rate.sleep();
      counter++;
    }
  }

private:
  /**
   * @details Random Point Cloud Generator
   * @param pc
   * @param centerX
   * @param centerY
   * @param sizeOfCloud
   */
  void getRandomPointCloud(sensor_msgs::PointCloud &pc, float centerX,
                           float centerY) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> distX(centerX, 2.);
    std::normal_distribution<> distY(centerY, 2.);

    for (auto &point : pc.points) {
      double xValue = distX(gen);
      double yValue = distY(gen);
      point.x = xValue;
      point.y = yValue;
      point.z = std::exp(-((xValue * xValue) + (yValue * yValue)) / 4.);
    }
    sensor_msgs::ChannelFloat32 depth_channel;
    depth_channel.name = "distance";
    for (auto &point : pc.points) {
      depth_channel.values.push_back(
          point.z); // or set to a random value if you like
    }
    // add channel to point cloud
    pc.channels.push_back(depth_channel);
  }

private:
  // Point Cloud Size
  int sizeOfCloud;
  // Point Cloud Centre
  float centreX;
  float centreY;
  // Frequency
  int frequency;
  // Counter
  int counter = 0;
  // ROS Node Handler
  ros::NodeHandle nh;
  // Publisher and Subscriber
  ros::Publisher pcd_pub;
};
PLUGINLIB_EXPORT_CLASS(ros_bare_bones::ROSBareBonesNodelet, nodelet::Nodelet)
} // namespace ros_bare_bones