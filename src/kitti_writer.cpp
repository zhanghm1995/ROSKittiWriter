/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月7日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include <ros_kitti_writer/kitti_writer.h>
// C++
#include <chrono>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdio>
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
// Opencv
#include <opencv2/opencv.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

static string folder_image_02 = "image_02/";
static string folder_image_03 = "image_03/";
static string folder_velodyne_points = "velodyne_points/";
static string format_image = "%|010|.png";
static string formate_velo = "%|010|.bin";

static string toDateTime(uint64_t ros_time)
{
  // Convert to chrono nanoseconds type
  chrono::nanoseconds tp(ros_time), tt_nano(ros_time);

  // Get nanoseconds part
  tp -= chrono::duration_cast<chrono::seconds>(tp);

  //to_time_t方法把时间点转化为time_t
  time_t tt =  std::time_t(chrono::duration_cast<chrono::seconds>(chrono::nanoseconds(tt_nano)).count());
  //把time_t转化为tm
  tm *localTm =  localtime(&tt);

  //tm格式化输出
  char buffer[20];
  char format[] = "%Y-%m-%d %H:%M:%S";
  strftime(buffer, sizeof(buffer), format, localTm);

  string nanosec_format = "%|010|";
  stringstream ss;
  ss<<buffer<<"."<<(boost::format(nanosec_format) % tp.count()).str();
  return ss.str();
}

KittiWriter::KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh):
count_(0),
imageCloudSync_(nh, "/darknet_ros/image_with_bboxes", "/kitti_player/hdl64e"),
processthread_(NULL),
processthreadfinished_(false)
{
  createFormatFolders();
  processthread_ = new boost::thread(boost::bind(&KittiWriter::process,this));
}

KittiWriter::~KittiWriter() {
  processthreadfinished_ = true;
  processthread_->join();
}

void KittiWriter::process()
{
  // main loop
  while(!processthreadfinished_&&ros::ok()) {
    // Get synchronized image with bboxes and cloud data
    sensors_fusion::MessagesSync::SyncImageCloudPair imagePair = imageCloudSync_.getSyncMessages();
    if((imagePair.first == nullptr) || (imagePair.second == nullptr)) {
      ROS_ERROR_THROTTLE(1,"Waiting for image and lidar topics!!!");
      continue;
    }

    // process image
    saveImage02(imagePair.first);

    // Preprocess point cloud
    saveVelodyne(imagePair.second);
    ++ count_;
  }

}

void KittiWriter::createFormatFolders()
{
  string root_directory = "/home/zhanghm/Test/catkin_ws_test/";

  // Create image 02 and 03 folder
  image_02_dir_path_ = boost::filesystem::path(root_directory)
                       / folder_image_02
                       / "data";
  if(!boost::filesystem::exists(image_02_dir_path_)) {
    boost::filesystem::create_directories(image_02_dir_path_);
  }
  timestamp_image02_path_ = boost::filesystem::path(root_directory)
                            / folder_image_02
                            / "timestamps.txt";

  image_03_dir_path_ = boost::filesystem::path(root_directory)
                       / folder_image_03
                       / "data";
  if(!boost::filesystem::exists(image_03_dir_path_)) {
    boost::filesystem::create_directories(image_03_dir_path_);
  }
  timestamp_image03_path_ = boost::filesystem::path(root_directory)
                            / folder_image_03
                            / "timestamps.txt";
  // Create velodyne_points folder
  velo_dir_path_ = boost::filesystem::path(root_directory)
                  / folder_velodyne_points
                  / "data";
  if(!boost::filesystem::exists(velo_dir_path_)) {
    boost::filesystem::create_directories(velo_dir_path_);
  }
}

void KittiWriter::saveImage02(const sensor_msgs::Image::ConstPtr & image)
{
  // Convert image detection grid to cv mat
  cv_bridge::CvImagePtr cv_det_grid_ptr;
  try{
    cv_det_grid_ptr = cv_bridge::toCvCopy(image,
        sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat raw_image = cv_det_grid_ptr->image;
 ROS_WARN_STREAM("save image "<<count_);
  // Get image name
  boost::filesystem::path image_02_file_path = image_02_dir_path_
      /(boost::format(format_image)%count_).str();
  string image_02_file_name = image_02_file_path.string();

  // 1) Save image
  cv::imwrite(image_02_file_name, raw_image);

  // 2) Save timestamps
  fstream filestr;
  filestr.open (timestamp_image02_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = image->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
}

void KittiWriter::saveVelodyne(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  // Convert to pcl cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*cloud));
  cloudMsg->fields[3].name = "intensity";
  pcl::fromROSMsg(*cloudMsg, *pcl_in);

  // cout
  size_t num = pcl_in->size();

  // Get cloud name
  boost::filesystem::path velodyne_file_path = velo_dir_path_
      /(boost::format(formate_velo)%count_).str();
  string velodyne_file_name = velodyne_file_path.string();

  // Begin save data
  FILE* stream = fopen(velodyne_file_name.c_str(), "wb");
  if(stream == NULL) {
    cout<<"error open "<<velodyne_file_name<<endl;
    return ;
  }
  float* data = (float*)malloc(4*num*sizeof(float));
  float* px = data + 0;
  float* py = data + 1;
  float* pz = data + 2;
  float* pI = data + 3;

  for(int i = 0; i < num; ++i) {
    *px = (float)pcl_in->points[i].x;
    *py = (float)pcl_in->points[i].y;
    *pz = (float)pcl_in->points[i].z;
    *pI = (float)pcl_in->points[i].intensity;
    px += 4;
    py += 4;
    pz += 4;
    pI += 4;
  }
  fwrite(data, sizeof(float), 4*num, stream);
  fclose(stream);

  // Save timestamps
  fstream filestr;
  filestr.open (timestamp_image02_path_.string().c_str(), fstream::out|fstream::app);
  uint64_t ros_tt = cloud->header.stamp.toNSec();
  string date = toDateTime(ros_tt);
  filestr<<date<<std::endl;
  filestr.close();
}
