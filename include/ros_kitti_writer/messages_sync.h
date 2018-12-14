/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年11月24日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_UTILS_MESSAGES_SYNC_H_
#define SRC_DYNAMIC_OBJECT_SRC_UTILS_MESSAGES_SYNC_H_

//C++
#include <string>
#include <map>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace sensors_fusion {

/**
 * @brief     Class for synchronizing sensor_msg::Image and sensor_msgs::PointCloud2.
 */
class MessagesSync {
public:
  typedef  std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::PointCloud2ConstPtr> SyncImageCloudPair;

  // Cons
//  MessagesSync(){};
  MessagesSync(ros::NodeHandle nh, std::string camera_topic_name, std::string lidar_topic_name);
  MessagesSync(ros::NodeHandle nh, std::string camera1_topic_name, std::string camera2_topic_name,
               std::string lidar_topic_name);
  virtual ~MessagesSync();

  SyncImageCloudPair getSyncMessages();

private:
  void cameraLidarCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg);

private:
  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  // 消息过滤器订阅相机和激光雷达点云话题
  typedef message_filters::Subscriber<sensor_msgs::Image> subCameraImage;
  subCameraImage subCamera1Image_;//订阅图像消息
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarData;
  subLidarData subLidarData_;//订阅激光雷达消息

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;


  // returns
  SyncImageCloudPair syncMessages_;
  bool flag;
};



/**
 * @brief     Class for synchronizing sensor_msg::Image and sensor_msgs::PointCloud2.
 */
class StereoMessagesSync {
public:
  typedef struct SynchronizedMessages {
    sensor_msgs::ImageConstPtr image1_ptr;
    sensor_msgs::ImageConstPtr image2_ptr;
    sensor_msgs::PointCloud2ConstPtr cloud_ptr;
  }SynchronizedMessages;

  // Cons
//  MessagesSync(){};
  StereoMessagesSync(ros::NodeHandle nh, std::string camera1_topic_name, std::string camera2_topic_name,
               std::string lidar_topic_name);
  virtual ~StereoMessagesSync();

  SynchronizedMessages getSyncMessages();

private:
  void stereocameraLidarCallback(const sensor_msgs::ImageConstPtr& image1_msg,
                               const sensor_msgs::ImageConstPtr& image2_msg,
                               const sensor_msgs::PointCloud2ConstPtr& lidar_msg);
private:
  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  // 消息过滤器订阅相机和激光雷达点云话题
  typedef message_filters::Subscriber<sensor_msgs::Image> subCameraImage;
  subCameraImage subCamera1Image_;//订阅图像消息
  subCameraImage subCamera2Image_;//订阅图像消息
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarData;
  subLidarData subLidarData_;//订阅激光雷达消息

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync;

  // returns
  SynchronizedMessages syncMessages2_;
  bool flag;
};


} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_UTILS_MESSAGES_SYNC_H_ */
