
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>
using namespace std;
using namespace cv;

string toDateTime(uint64_t ros_time)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_subscriber");
  ros::NodeHandle node;
  std_msgs::Header header;
  ros::Rate rate(10);
  while(ros::ok()) {
    chrono::system_clock::time_point nowTp = chrono::system_clock::now();
    header.stamp = ros::Time::now();
   //  chrono::time_point<chrono::system_clock, chrono::nanoseconds> tp = chrono::time_point_cast<chrono::nanoseconds>(nowTp);
    chrono::system_clock::duration tp = nowTp.time_since_epoch();
    cout<<"chr "<<setprecision(20)<<tp.count()<<endl;

    //to_time_t方法把时间点转化为time_t
    time_t tt =  std::time_t(chrono::duration_cast<chrono::seconds>(tp).count());

    //使用ctime转化为字符串
    cout << ctime(&tt);

    uint64_t ros_tt = header.stamp.toNSec();
    string date = toDateTime(ros_tt);
    cout<<date<<endl;
    cout<<"======"<<endl;
    rate.sleep();
  }

  ros::spin();
  return 0;
}
