ROS_KITTI_Writer
==================
A ROS package for synchronizing different sensors data from **lidar**,**camera**,**GPS/IMU** and saving them according to [KITTI Raw Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) format.

## Overview
Due to the popularity of the [KITTI Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php), there are many tools to parse or visualize the **KITTI Dataset**.  And it is convinient for us to  focu on our own **perception** algorithm developement. 

Therefore, how about saving our own data according to **KITTI Dataset** format? If so, we can extend the **KITTI Dataset** by use the existing tools to parse and visualize our own capturing data.

This repository aims to subscribe offline `rosbag` files and save the correspoing data into **KITTI Raw Dataset**.
## Features
- Synchronizing different messages
- Create KITTI Raw Dataset format folders automatically
- Save timestamps to nanoseconds

## Usages
Please use `roslaunch` file to run the code, and before you execute, please ensure the `directory` argument has been set to a valid path in your computer.
```
roslaunch ros_kitti_writer kitti_writer_standalone.launch

```
## TODO
- [ ]  add save oxts data
- [ ] improve `saveVelodyne` function
- [ ] add calibration related files

