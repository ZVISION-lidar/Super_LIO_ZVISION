
## Overview

<p align="center">
  <img src="docs/super_lio_zvision_nz5_mapping_demo.gif" width="95%">
</p>

**Key Features:  Efficient · Robust · Cross-Platform Compatible · Supports Both ROS1/ROS2 Versions**

Super-LIO is a robust and efficient LiDAR–Inertial Odometry (LIO) system designed for real-time and large-scale autonomous navigation. It introduces a compact and structured mapping strategy that enables predictable correspondence search and stable state estimation. The system is validated through extensive real-world experiments and comparisons with state-of-the-art methods, which demonstrates that Super-LIO not only achieves **excellent accuracy** but also maintains **lower resource consumption** and realizes a nearly **1.2–4× higher real-time processing speed**⚡.



## Quickly Run with Zvision_lidar

**For ROS2 Users**: Please switch to the **main** branch and follow the instructions at [ros2 branch](https://github.com/ZVISION-lidar/Super_LIO_ZVISION/)

### Requirements

Ubuntu 20(18).04 · C++20 · ROS noetic(melodic) · Eigen · PCL 

### Dependencies

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### Build & Run
```bash
git clone -b ROS1 https://github.com/ZVISION-lidar/Super_LIO_ZVISION.git
cd Super_LIO_ZVISION
catkin_make

source install/setup.bash
roslaunch super_lio zvision_nz1.launch

```
