/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Created On   : 2018-12-07
* Copyright    :
* Descriptoin  : read rosbag and write all data according to KITTI dataset format
* References   : http://www.cvlibs.net/datasets/kitti/eval_object.php
======================================================================*/
#ifndef CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_
#define CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_

// C++
#include <iostream>
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
// Boost
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/thread/thread.hpp>

#include <ros_kitti_writer/messages_sync.h>
class KittiWriter {
public:
  // Default constructor
  KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh);
  virtual ~KittiWriter();

  void process();
private:
  void createFormatFolders();


  //! Save image02 and corresponding timestamp
  void saveImage02(const sensor_msgs::Image::ConstPtr & image);

  //! Save lidar and corresponding timestamps
  void saveVelodyne(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  // Subscriber
  ros::Subscriber image_sub_;

  // Image and cloud synchronizer
  sensors_fusion::MessagesSync imageCloudSync_;

  // Multi thread
  boost::thread* processthread_;
  bool processthreadfinished_;

  // Class members
  boost::filesystem::path image_02_dir_path_, image_03_dir_path_;
  boost::filesystem::path timestamp_image02_path_, timestamp_image03_path_;
  boost::filesystem::path velo_dir_path_;

  // Whole counter
  unsigned long int count_;
};

#endif /* CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_ */
