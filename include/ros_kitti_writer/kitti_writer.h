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

#include <ros/ros.h>
class KittiWriter {
public:
  // Default constructor
  KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh);
  virtual ~KittiWriter();
};

#endif /* CATKIN_WS_TEST_SRC_ROS_KITTI_WRITER_INCLUDE_ROS_KITTI_WRITER_KITTI_WRITER_H_ */
