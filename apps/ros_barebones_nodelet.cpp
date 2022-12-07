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
#include <sensor_msgs/PointCloud2.h>

#include <utils.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>

namespace ros_bare_bones {
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
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
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();
    points_callback();
    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcdfile", 5, true);
    pcd_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &ROSBareBonesNodelet::pub_once_cb, this, true, true);
  }

private:
  /**
   * @brief Points Callback function
   */
  void points_callback() {
    // ROS nodelet info with function name
    NODELET_INFO("ROSBareBonesNodelet - %s", __FUNCTION__);
    std::string pcd_file = private_nh.param<std::string>("pcd_file", "");
    pcdfile.reset(new PointCloud());
    pcl::io::loadPCDFile(pcd_file, *pcdfile);
    pcdfile->header.frame_id = "pcdfile";
    if (pcdfile->size() > 0)
      std::cout << "PCD File Received of Size: " << pcdfile->size()
                << std::endl;
    else
      std::cout << "Didn't receive PCD File" << std::endl;
  }

  void pub_once_cb(const ros::WallTimerEvent& event) {
    pcd_pub.publish(pcdfile);
  }

private:
  // ROS Node Handler
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  // Publisher and Subscriber
  ros::Publisher pcd_pub;
  ros::Subscriber m_sub;
  // ROS Wall Timer
  ros::WallTimer pcd_pub_timer;

  // PCD
  PointCloud::Ptr pcdfile;
};
PLUGINLIB_EXPORT_CLASS(ros_bare_bones::ROSBareBonesNodelet, nodelet::Nodelet)
} // namespace ros_bare_bones