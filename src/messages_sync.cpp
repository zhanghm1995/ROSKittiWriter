/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年11月24日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include <ros_kitti_writer/messages_sync.h>
//C++
#include <iomanip>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
//#include "boost/asio.hpp"
namespace sensors_fusion {

////////////////////////////////MessagesSync////////////////////////////////////////////////////////
ImageCloudMessagesSync::ImageCloudMessagesSync(ros::NodeHandle nh, std::string camera_topic_name, std::string lidar_topic_name):
    nodeHandle_(nh),
    subCamera1Image_(nh, camera_topic_name, 20),
    subLidarData_(nh,lidar_topic_name, 10),
    sync(MySyncPolicy(10), subCamera1Image_, subLidarData_)
{
  ROS_INFO("lidar and camera data synchronizing started");
  sync.registerCallback(boost::bind(&ImageCloudMessagesSync::cameraLidarCallback, this,_1, _2));
}

bool ImageCloudMessagesSync::is_valid()
{
  if (messages_queue_.empty())
    return false;
  else
    return true;
}

sensors_fusion::SynchronizedMessages ImageCloudMessagesSync::getSyncMessages()
{
  if(!messages_queue_.empty()) {
    sensors_fusion::SynchronizedMessages res = messages_queue_.front();
    messages_queue_.pop();
    return res;
  }
  else
    return SynchronizedMessages();
}


void ImageCloudMessagesSync::cameraLidarCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
  ROS_INFO_THROTTLE(5, "Sync_Callback");
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  // 2018-11-28 add below code for Warning "Failed to find match for field 'intensity'."
  // https://answers.ros.org/question/173396/losing-intensity-data-when-converting-between-sensor_msgspointcloud2-and-pclpointcloudt/
  cloudMsg->fields[3].name = "intensity";
  SynchronizedMessages result;
  result.image1_ptr = image_msg;
  result.cloud_ptr = cloudMsg;

  messages_queue_.push(result);
}

ImageCloudMessagesSync::~ImageCloudMessagesSync() {
}


/////////////////////////////////StereoMessagesSync//////////////////////////////////////////////////////////

StereoMessagesSync::StereoMessagesSync(ros::NodeHandle nh, std::string camera1_topic_name,
                                       std::string camera2_topic_name,
                                       std::string lidar_topic_name):
nodeHandle_(nh),
subCamera1Image_(nh, camera1_topic_name, 20),
subCamera2Image_(nh, camera2_topic_name, 20),
subLidarData_(nh,lidar_topic_name, 10),
sync(MySyncPolicy(10), subCamera1Image_, subCamera2Image_, subLidarData_)
{
  ROS_INFO("lidar and stereo camera data synchronizing started");
  sync.registerCallback(boost::bind(&StereoMessagesSync::stereocameraLidarCallback, this,_1, _2, _3));
}

StereoMessagesSync::~StereoMessagesSync() {
}


void StereoMessagesSync::stereocameraLidarCallback(const sensor_msgs::ImageConstPtr& image1_msg,
                               const sensor_msgs::ImageConstPtr& image2_msg,
                               const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
  ROS_INFO_THROTTLE(5, "Sync_Callback");
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  cloudMsg->fields[3].name = "intensity";
  SynchronizedMessages result;
  result.image1_ptr = image1_msg;
  result.image2_ptr = image2_msg;
  result.cloud_ptr = cloudMsg;

  messages_queue_.push(result);
}

bool StereoMessagesSync::is_valid()
{
  if (messages_queue_.empty())
    return false;
  else
    return true;
}

sensors_fusion::SynchronizedMessages  StereoMessagesSync::getSyncMessages()
{
  if(!messages_queue_.empty()) {
    sensors_fusion::SynchronizedMessages res = messages_queue_.front();
    messages_queue_.pop();
    return res;
  }
  else
    return SynchronizedMessages();
}

} /* namespace sensors_fusion */
