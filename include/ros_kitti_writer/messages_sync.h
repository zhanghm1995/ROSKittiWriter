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
#include <queue>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace sensors_fusion {


typedef struct SynchronizedMessages_
{
    sensor_msgs::ImageConstPtr image1_ptr;
    sensor_msgs::ImageConstPtr image2_ptr;
    sensor_msgs::PointCloud2ConstPtr cloud_ptr;
}SynchronizedMessages;

/**
 * @brief     Class for synchronizing sensor_msg::Image and sensor_msgs::PointCloud2.
 */
class ImageCloudMessagesSync
{
public:
  // Cons
  ImageCloudMessagesSync(ros::NodeHandle nh, std::string camera1_topic_name, std::string lidar_topic_name);
  virtual ~ImageCloudMessagesSync();

  SynchronizedMessages getSyncMessages();

private:
  void cameraLidarCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& lidar_msg);
private:
  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  // 消息过滤器订阅相机和激光雷达点云话题
  typedef message_filters::Subscriber<sensor_msgs::Image> subCameraImage;
  subCameraImage subCamera1Image_;//订阅图像消息
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarData;
  subLidarData subLidarData_;//订阅激光雷达消息

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync;

  // Using queue to storing messages to avoid losing messages
  std::queue<SynchronizedMessages> messages_queue_;
};



/**
 * @brief     Class for synchronizing stereo sensor_msg::Image and sensor_msgs::PointCloud2.
 */
class StereoMessagesSync {
public:
  // Cons
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

  // Using queue to storing messages to avoid losing messages
  std::queue<SynchronizedMessages> messages_queue_;
};


} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_UTILS_MESSAGES_SYNC_H_ */
