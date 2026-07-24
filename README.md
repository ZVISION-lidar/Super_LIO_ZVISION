
## Overview

## Quickly Run with Zvision_lidar

**For ROS1 Users**: Please switch to the **ros1** branch and follow the instructions at [ros1 branch](https://github.com/ZVISION-lidar/Super_LIO_ZVISION/tree/ros1)

### Requirements

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL 

### Dependencies

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### Build & Run
```bash
git clone https://github.com/ZVISION-lidar/Super_LIO_ZVISION.git
cd Super-LIO
colcon build

source install/setup.bash
ros2 launch super_lio zvision.py

```

#### 🔁 Relocalization Mode
Super-LIO supports relocalization using a pre-built map, allowing the system to resume localization from a saved map without restarting the mapping process.
This mode is useful for long-term deployment, repeated missions, or recovery after tracking loss.

Before running relocalization, please make sure that:
- A map has been previously saved to disk.

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```


# Super_LIO_ZVISION_LOCALIZATION-ROS2

基于先验地图的重定位功能


## 1. Requirements

### 1.1 Ubuntu  ROS2

推荐环境：

```text
Ubuntu 22.04
ROS2 Humble
```

### 1.2 System Requirements

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
sudo apt install libpcl-dev libeigen3-dev
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install ros-humble-tf2-ros ros-humble-sensor-msgs-py
```

### 1.3 Python Requirements

```bash
pip3 install --user numpy==1.24.4 scipy==1.10.1 scikit-learn
pip3 install --user open3d==0.18.0
```

## 2. 编译

将本包放入 ROS2 工作空间的 `src` 目录：

```bash
mkdir -p ~/FAST_LIO_ZVISION_LOCALIZATION-ROS2_ws/src
cd ~/FAST_LIO_ZVISION_LOCALIZATION-ROS2_ws/src
cp -r /path/to/FAST_LIO_ZVISION_LOCALIZATION-ROS2-main .
cd ..
colcon build
source install/setup.bash
```


## 3. 建图

建图仍使用 FAST-LIO 原来的 mapping launch：

```bash
ros2 launch fast_lio_zvision_localization mapping_zvision_nz5.launch.py
```

其他型号：

```bash
ros2 launch fast_lio_zvision_localization mapping_zvision_nz1.launch.py
ros2 launch fast_lio_zvision_localization mapping_zvision_nz3.launch.py
ros2 launch fast_lio_zvision_localization mapping_zvision_nz5_mt.launch.py
```


## 4. 重定位

指定先验 PCD 地图启动：

```bash
ros2 launch fast_lio_zvision_localization localization_zvision_nz5.launch.py map:=/path/to/map.pcd
```

播放 rosbag 时，launch 需要开启仿真时间，bag 播放也需要发布 `/clock`：

```bash
ros2 launch fast_lio_zvision_localization localization_zvision_nz5.launch.py map:=/path/to/map.pcd use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

命令行发布初始位姿：

```bash
ros2 run fast_lio_zvision_localization publish_initial_pose.py x y z yaw pitch roll
```

姿态角为弧度制。示例：

```bash
ros2 run fast_lio_zvision_localization publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0
```

也可以在 RViz2 中使用 `2D Pose Estimate`，它会向 `/initialpose` 发布初始位姿。


## 5. 主要话题和坐标系

坐标系：

```text
map          先验地图坐标系
camera_init  FAST-LIO 本体的局部世界坐标系
body         FAST-LIO 机体坐标系
```

主要话题：

```text
/map                 map_publisher.py 发布的先验 PCD 地图
/cloud_registered    FAST-LIO 输出的 camera_init 坐标系点云
/Odometry            FAST-LIO 里程计，表示 camera_init -> body
/map_to_odom         低频 ICP 重定位结果，表示 map -> camera_init
/localization        融合后的 map 坐标系全局里程计
/cur_scan_in_map     当前帧点云可视化
/submap              当前 FOV 裁剪出的先验地图子图可视化
```

初始化成功后的 TF 树：

```text
map -> camera_init -> body
```


## 6. 关键参数

重定位参数位于 `config/localization.yaml`：

```text
localization.map_voxel_size
localization.scan_voxel_size
localization.frequency
localization.fitness_threshold
localization.fov
localization.fov_far
transform_fusion.publish_frequency
```

先验地图路径可以在 launch 中直接修改：

```bash
ros2 launch fast_lio_zvision_localization localization_zvision_nz5.launch.py map:=/home/wwb/DATA/MAP/map1.pcd
```


## 7. 注意事项

`Not match!!!!` 表示当前 ICP 的 fitness score 小于 `localization.fitness_threshold`，因此这一次全局修正不会被采用。警告中打印的矩阵和分数用于分析，不会自动作为最终位姿应用。

如果播放 rosbag 时 RViz2 出现 TF extrapolation，优先检查：

1. launch 是否设置 `use_sim_time:=true`；
2. `ros2 bag play` 是否加了 `--clock`；
3. 可视化话题和 TF 是否使用同一套 bag 时间。
