// Copyright (C) 2018  Haiming Zhang<zhanghm_1995@qq.com>,
// Intelligent Vehicle Research Center, Beijing Institute of Technology

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.


#include <ros/ros.h>
#include <ros_kitti_writer/kitti_writer.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "sensor_setup_node");
    KittiWriter kitti_writer(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}

