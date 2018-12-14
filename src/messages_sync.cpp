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
MessagesSync::MessagesSync(ros::NodeHandle nh, std::string camera_topic_name, std::string lidar_topic_name):
    nodeHandle_(nh),
    subCamera1Image_(nh, camera_topic_name, 2),
    subLidarData_(nh,lidar_topic_name, 2),
    flag(false),
    sync(MySyncPolicy(10), subCamera1Image_, subLidarData_)
{
  ROS_INFO("lidar and camera data synchronizing started");
  sync.registerCallback(boost::bind(&MessagesSync::cameraLidarCallback, this,_1, _2));
}

MessagesSync::SyncImageCloudPair MessagesSync::getSyncMessages()
{
  if(!flag)
    return SyncImageCloudPair();
  else {
    flag = false;
    return syncMessages_;
  }
}




void MessagesSync::cameraLidarCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
  ROS_INFO_THROTTLE(5, "Sync_Callback");
  // 2018-11-28 add below code for Warning "Failed to find match for field 'intensity'."
  // https://answers.ros.org/question/173396/losing-intensity-data-when-converting-between-sensor_msgspointcloud2-and-pclpointcloudt/
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  cloudMsg->fields[3].name = "intensity";
  syncMessages_ = SyncImageCloudPair(image_msg, cloudMsg);
  flag = true;
}



MessagesSync::~MessagesSync() {
}


/////////////////////////////////StereoMessagesSync//////////////////////////////////////////////////////////

StereoMessagesSync::StereoMessagesSync(ros::NodeHandle nh, std::string camera1_topic_name,
                                       std::string camera2_topic_name,
                                       std::string lidar_topic_name):
nodeHandle_(nh),
subCamera1Image_(nh, camera1_topic_name, 2),
subLidarData_(nh,lidar_topic_name, 2),
flag(false),
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
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  cloudMsg->fields[3].name = "intensity";
  SynchronizedMessages result;
  result.image1_ptr = image1_msg;
  result.image2_ptr = image2_msg;
  result.cloud_ptr = cloudMsg;
  syncMessages2_ = result;
  flag = true;
}

StereoMessagesSync::SynchronizedMessages  StereoMessagesSync::getSyncMessages()
{
  if(!flag)
    return SynchronizedMessages();
  else {
    flag = false;
    return syncMessages2_;
  }
}

} /* namespace sensors_fusion */
