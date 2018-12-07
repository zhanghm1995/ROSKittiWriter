/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月7日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include <ros_kitti_writer/kitti_writer.h>

#include <chrono>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
using namespace std;

static string folder_image_02 = "image_02/data";
static string folder_image_03 = "image_03/data";
static string folder_velodyne_points = "velodyne_points/data";

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

KittiWriter::KittiWriter(ros::NodeHandle nh, ros::NodeHandle private_nh)
{


}

KittiWriter::~KittiWriter() {
  // TODO Auto-generated destructor stub
}

void KittiWriter::createFormatFolder()
{
  string root_directory = "2018-12-08";

  // Create image 02 and 03 folder
  boost::filesystem::path image_file_path = boost::filesystem::path(root_directory)
  / folder_image_02;
  if(!boost::filesystem::exists(image_file_path)) {
    boost::filesystem::create_directories(image_file_path);
  }
  image_file_path = boost::filesystem::path(root_directory)
  / folder_image_03;
  if(!boost::filesystem::exists(image_file_path)) {
    boost::filesystem::create_directories(image_file_path);
  }

  // Create velodyne_points folder
  boost::filesystem::path velo_file_path = boost::filesystem::path(root_directory)
  / folder_velodyne_points;
  if(!boost::filesystem::exists(velo_file_path)) {
    boost::filesystem::create_directories(velo_file_path);
  }
}
