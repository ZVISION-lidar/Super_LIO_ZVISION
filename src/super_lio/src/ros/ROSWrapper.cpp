
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 8);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // ================= IMU vibration suppression filter =================
  // [默认关闭] yaml 里不写这两个参数 = 不滤波.
  // 滤波会引入相位延迟, 且与 FastLIVO2 的裸 IMU 直推行为不一致;
  // 需要开启时在 yaml 里显式设 lio.sensor.imu_notch_enable / imu_lp_enable: true.
  node.declare_parameter<bool>("lio.sensor.imu_notch_enable", false);
  node.get_parameter("lio.sensor.imu_notch_enable", g_imu_notch_enable);

  node.declare_parameter<bool>("lio.sensor.imu_lp_enable", false);
  node.get_parameter("lio.sensor.imu_lp_enable", g_imu_lp_enable);

  node.declare_parameter<double>("lio.sensor.imu_filt_fs", 197.98);
  node.get_parameter("lio.sensor.imu_filt_fs", g_imu_filt_fs);

  node.declare_parameter<std::vector<double>>(
      "lio.sensor.imu_notch_f0s", std::vector<double>({43.42, 19.61}));
  node.get_parameter("lio.sensor.imu_notch_f0s", g_imu_notch_f0s);

  node.declare_parameter<double>("lio.sensor.imu_notch_q", 8.0);
  node.get_parameter("lio.sensor.imu_notch_q", g_imu_notch_q);

  node.declare_parameter<double>("lio.sensor.imu_lp_fc", 15.0);
  node.get_parameter("lio.sensor.imu_lp_fc", g_imu_lp_fc);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);
  node.declare_parameter<int>("lio.output.imu_odom_pub_divider", 1);
  node.get_parameter("lio.output.imu_odom_pub_divider", g_imu_odom_pub_divider);
  if(g_imu_odom_pub_divider < 1) g_imu_odom_pub_divider = 1;

  LOG(INFO) << GREEN << " ---> [Param] output/imu_odom_pub_divider: "
            << g_imu_odom_pub_divider << " (1 = publish at the IMU rate)"
            << RESET;

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  // ================= [LOOP-CLOSURE] 回环检测 + 位姿图优化 =================
  node.declare_parameter<bool>("lio.loop_closure.enable", false);
  node.get_parameter("lio.loop_closure.enable", g_loop_closure_enable);

  node.declare_parameter<double>("lio.loop_closure.frequency", 2.0);
  node.get_parameter("lio.loop_closure.frequency", g_loop_closure_frequency);

  // 关键帧判定阈值 (位移与角度同时满足才建关键帧)
  node.declare_parameter<double>("lio.loop_closure.keyframe_dist", 1.0);
  node.get_parameter("lio.loop_closure.keyframe_dist", g_keyframe_dist);
  node.declare_parameter<double>("lio.loop_closure.keyframe_angle", 0.2);
  node.get_parameter("lio.loop_closure.keyframe_angle", g_keyframe_angle);

  // Scan Context 描述子
  node.declare_parameter<int>("lio.loop_closure.sc_num_rings", 20);
  node.get_parameter("lio.loop_closure.sc_num_rings", g_sc_num_rings);
  node.declare_parameter<int>("lio.loop_closure.sc_num_sectors", 60);
  node.get_parameter("lio.loop_closure.sc_num_sectors", g_sc_num_sectors);
  node.declare_parameter<double>("lio.loop_closure.sc_max_radius", 80.0);
  node.get_parameter("lio.loop_closure.sc_max_radius", g_sc_max_radius);
  node.declare_parameter<double>("lio.loop_closure.sc_lidar_height", 2.0);
  node.get_parameter("lio.loop_closure.sc_lidar_height", g_sc_lidar_height);
  node.declare_parameter<int>("lio.loop_closure.sc_exclude_recent", 30);
  node.get_parameter("lio.loop_closure.sc_exclude_recent", g_sc_exclude_recent);
  node.declare_parameter<int>("lio.loop_closure.sc_num_candidates", 10);
  node.get_parameter("lio.loop_closure.sc_num_candidates", g_sc_num_candidates);
  node.declare_parameter<double>("lio.loop_closure.sc_search_ratio", 0.1);
  node.get_parameter("lio.loop_closure.sc_search_ratio", g_sc_search_ratio);
  node.declare_parameter<double>("lio.loop_closure.sc_distance_threshold", 0.15);
  node.get_parameter("lio.loop_closure.sc_distance_threshold", g_sc_distance_threshold);

  // ICP 几何验证
  //   icp_fitness_score_threshold 单位是 m^2 (PCL 的 fitness score 是
  //   平方距离的均值), 开方才是实际 RMS 对齐误差: 0.3 -> 0.55m.
  node.declare_parameter<int>("lio.loop_closure.icp_max_correspondence_distance", 15);
  node.get_parameter("lio.loop_closure.icp_max_correspondence_distance",
                     g_icp_max_correspondence_distance);
  node.declare_parameter<int>("lio.loop_closure.icp_max_iterations", 100);
  node.get_parameter("lio.loop_closure.icp_max_iterations", g_icp_max_iterations);
  node.declare_parameter<double>("lio.loop_closure.icp_fitness_score_threshold", 0.3);
  node.get_parameter("lio.loop_closure.icp_fitness_score_threshold",
                     g_icp_fitness_score_threshold);
  node.declare_parameter<int>("lio.loop_closure.icp_history_search_num", 5);
  node.get_parameter("lio.loop_closure.icp_history_search_num", g_icp_history_search_num);

  // GTSAM 位姿图噪声 (1-sigma)
  node.declare_parameter<double>("lio.loop_closure.pg_odom_rot_sigma", 0.001);
  node.get_parameter("lio.loop_closure.pg_odom_rot_sigma", g_pg_odom_rot_sigma);
  node.declare_parameter<double>("lio.loop_closure.pg_odom_trans_sigma", 0.01);
  node.get_parameter("lio.loop_closure.pg_odom_trans_sigma", g_pg_odom_trans_sigma);
  node.declare_parameter<double>("lio.loop_closure.pg_loop_rot_sigma", 0.005);
  node.get_parameter("lio.loop_closure.pg_loop_rot_sigma", g_pg_loop_rot_sigma);
  node.declare_parameter<double>("lio.loop_closure.pg_loop_trans_sigma", 0.05);
  node.get_parameter("lio.loop_closure.pg_loop_trans_sigma", g_pg_loop_trans_sigma);
  node.declare_parameter<double>("lio.loop_closure.pg_prior_rot_sigma", 0.000001);
  node.get_parameter("lio.loop_closure.pg_prior_rot_sigma", g_pg_prior_rot_sigma);
  node.declare_parameter<double>("lio.loop_closure.pg_prior_trans_sigma", 0.000001);
  node.get_parameter("lio.loop_closure.pg_prior_trans_sigma", g_pg_prior_trans_sigma);
  // [DIAG] 位姿图残差诊断开关 (false = 关闭, 日常运行推荐)
  node.declare_parameter<bool>("lio.loop_closure.diag_enable", false);
  node.get_parameter("lio.loop_closure.diag_enable", g_pg_diag_enable);

  // 回环后是否把优化位姿回写到前端 ESKF
  node.declare_parameter<bool>("lio.loop_closure.writeback_enable", true);
  node.get_parameter("lio.loop_closure.writeback_enable", g_loop_writeback_enable);
  // 退出时是否用关键帧重拼接保存地图
  node.declare_parameter<bool>("lio.loop_closure.keyframe_map_enable", false);
  node.get_parameter("lio.loop_closure.keyframe_map_enable", g_loop_keyframe_map_enable);
  // 回环可视化 (只控制重负载部分: 点云/优化后地图)
  node.declare_parameter<bool>("lio.loop_closure.visual_enable", true);
  node.get_parameter("lio.loop_closure.visual_enable", g_loop_visual_enable);
  node.declare_parameter<int>("lio.loop_closure.visual_map_interval", 50);
  node.get_parameter("lio.loop_closure.visual_map_interval", g_loop_visual_map_interval);

  // 参数合法性检查 (防止 yaml 写错导致运行时异常)
  if (g_sc_exclude_recent < 1) g_sc_exclude_recent = 1;
  if (g_sc_num_candidates < 1) g_sc_num_candidates = 1;
  if (g_icp_history_search_num < 0) g_icp_history_search_num = 0;
  if (g_loop_closure_frequency <= 0.0) g_loop_closure_frequency = 1.0;
  if (g_loop_visual_map_interval < 1) g_loop_visual_map_interval = 1;

  LOG(INFO) << GREEN << " ---> [Param] loop_closure/enable: "
            << (g_loop_closure_enable ? "true" : "false")
            << ", frequency: " << g_loop_closure_frequency << " Hz"
            << ", diag_enable: " << (g_pg_diag_enable ? "true" : "false")
            << RESET;

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}


std::string lidarTypeToString(int type) {
  if (type < 1 || type > LID_TYPE_COUNT_VAL) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  // IMU vibration suppression filter params
  imu_notch_enable_ = g_imu_notch_enable;
  imu_lp_enable_    = g_imu_lp_enable;
  imu_filt_fs_      = g_imu_filt_fs;
  imu_notch_f0s_    = g_imu_notch_f0s;
  imu_notch_q_      = g_imu_notch_q;
  imu_lp_fc_        = g_imu_lp_fc;
  initFilters();

  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .best_effort()
                 .durability_volatile();

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .best_effort()
                   .durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 50);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 50);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", 10);

  // ============================================================
  // [LOOP-CLOSURE] 回环可视化
  // ============================================================
  // 回环节点 + 回环边 (一个 topic 里装两个 marker).
  pub_loop_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/lio/loop_closure_constraints", 10);

  // 优化后 / 优化前 的关键帧轨迹 (叠加显示, 一眼看出回环拉回了多少).
  pub_loop_path_ =
      this->create_publisher<nav_msgs::msg::Path>("/lio/loop_path", 10);
  pub_loop_path_before_ =
      this->create_publisher<nav_msgs::msg::Path>("/lio/loop_path_before", 10);

  // 优化后的全局地图 (关键帧重拼接) —— 开销大, 低频发.
  pub_loop_map_optimized_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/loop_map_optimized", 1);

  // ICP 匹配的点云 (只在检出新回环时发).
  pub_loop_icp_history_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/loop_icp_history_cloud", 1);
  pub_loop_icp_corrected_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/loop_icp_corrected_cloud", 1);
  pub_loop_icp_raw_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/loop_icp_raw_cloud", 1);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

// ============================================================
// IMU 振动抑制滤波器初始化
//   acc : Notch(多频点) -> BiquadLP(15Hz)
//   gyr : Notch(多频点)
//   Notch 精准消除旋翼振动主频, 通带几乎无相位延迟;
//   低通兜底宽带噪声与谐波。
// ============================================================
void ROSWrapper::initFilters(){
  const double fs = imu_filt_fs_;
  for(int i = 0; i < 3; ++i){
    notch_acc_[i].setFreqs(imu_notch_f0s_, fs, imu_notch_q_);
    notch_gyr_[i].setFreqs(imu_notch_f0s_, fs, imu_notch_q_);
    lp_acc_[i].setParams(imu_lp_fc_, fs);
  }
  imu_filt_initialized_ = true;

  std::string f0s_str;
  for (size_t i = 0; i < imu_notch_f0s_.size(); ++i) {
    if (i) f0s_str += ", ";
    f0s_str += std::to_string(imu_notch_f0s_[i]);
  }

  LOG(INFO) << GREEN << " ---> [ROSWrapper]: IMU filter. "
            << "notch=" << (imu_notch_enable_ ? "on" : "off")
            << " f0s=[" << f0s_str << "]Hz"
            << " Q=" << imu_notch_q_
            << " | lowpass=" << (imu_lp_enable_ ? "on" : "off")
            << " fc=" << imu_lp_fc_ << "Hz"
            << " | fs=" << fs << "Hz"
            << RESET;
}

// ============================================================
// 单帧 IMU 滤波
//   acc : Notch -> LowPass   (两者可独立开关)
//   gyr : Notch only         (低通会带来姿态滞后, 故不设)
// ============================================================
void ROSWrapper::filterIMU(const IMUData& in, IMUData& out){
  out = in;

  const bool use_notch = imu_notch_enable_;
  const bool use_lp    = imu_lp_enable_;

  if(!use_notch && !use_lp){
    out.acc = in.acc;
    out.gyr = in.gyr;
    return;
  }

  for(int i = 0; i < 3; ++i){
    double a = in.acc(i);
    double g = in.gyr(i);

    if(use_notch){
      a = notch_acc_[i].update(a);
      g = notch_gyr_[i].update(g);
    }
    if(use_lp){
      a = lp_acc_[i].update(a);
    }

    out.acc(i) = a;
    out.gyr(i) = g;
  }
}

void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                  msg->linear_acceleration.y,
                  msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);
    last_timestamp_imu_ = data.secs;
    // eskf_->Reset();   // todo:
    return;
  }

  // 首次进入时初始化滤波器系数
  if(!imu_filt_initialized_){
    initFilters();
  }

  // 滤波: acc -> Notch(多频点) + LP, gyr -> Notch(多频点)
  IMUData filtered_data;
  filterIMU(data, filtered_data);

  imu_buffer_.push_back(filtered_data);
  last_timestamp_imu_ = data.secs;

  // ============================================================
  // [ALIGN-FastLIVO2] 前向递推 (纯 IMU 推算)
  //   内部维护独立的递推 IMU 缓冲, 在 EKF 锚点后逐条重积分,
  //   积分步长恒为 ~5ms (与 FastLIVO2 的 prop_imu_once 行为一致).
  //   旧实现锚点后不做重积分, 会用单条 acc 外推一大段 dt,
  //   导致每个 LIO 帧 (10Hz) 向速度注入一个尖峰.
  // ============================================================
  DynamicState imu_state, robo_state;
  if(eskf_->ForwardPredict(filtered_data, imu_state, robo_state)){

    static int pub_counter = 0;
    const int pub_divider = g_imu_odom_pub_divider;
    if(++pub_counter < pub_divider) return;
    pub_counter = 0;

    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  // case LID_TYPE::ZVISION:
  // {
  //   // ZVISION point timestamp field is in seconds; use first valid point timestamp as reference.
  //   pcl::PointCloud<zvision_ros::Point> pl_orig;
  //   pcl::moveFromROSMsg(*msg, pl_orig);
  //   if (pl_orig.empty()) return;
  //   lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);

  //   double min_offset = std::numeric_limits<double>::max();
  //   double max_offset = std::numeric_limits<double>::lowest();

  //   for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
  //     auto& pt = pl_orig.points[i];
  //     if (!validPoint(pt.x, pt.y, pt.z)) continue;
  //     min_offset = std::min(min_offset, pt.timestamp);
  //     max_offset = std::max(max_offset, pt.timestamp);
  //   }

  //   lidar_data.start_time = min_offset;
  //   lidar_data.end_time = max_offset;

  //   for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
  //     auto& pt = pl_orig.points[i];
  //     if (!validPoint(pt.x, pt.y, pt.z)) continue;
  //     double offset_time = pt.timestamp - lidar_data.start_time;
  //     lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
  //   }
  //   break;
  // }
  case LID_TYPE::ZVISION:
  {
    pcl::PointCloud<zvision_ros::Point> pl_orig;
    pcl::moveFromROSMsg(*msg, pl_orig);
    if (pl_orig.empty()) return;
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);

    // ---- 第一遍：全量点算 min/max，不加 blind / maxrange / filter_rate ----
    double min_offset = std::numeric_limits<double>::max();
    double max_offset = std::numeric_limits<double>::lowest();

    for (std::size_t i = 0; i < pl_orig.size(); ++i) {
      const auto& pt = pl_orig.points[i];
      if (!std::isfinite(pt.timestamp)) continue;
      min_offset = std::min(min_offset, pt.timestamp);
      max_offset = std::max(max_offset, pt.timestamp);
    }

    if (min_offset > max_offset) {
      RCLCPP_WARN(rclcpp::get_logger("preprocess"),
                  "ZVISION: no valid timestamp in frame");
      return;
    }

    lidar_data.start_time = min_offset;
    lidar_data.end_time   = max_offset;

    // ---- 第二遍：按 filter_rate 采样 + validPoint + blind + maxrange ----
    for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
      auto& pt = pl_orig.points[i];

      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      if (!std::isfinite(pt.timestamp)) continue;

      // 距离平方，避免开根号
      const double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

      // blind < dist < maxrange
      if (dist_sq <= g_blind2) continue;       // 盲区内，丢弃
      if (dist_sq >= g_maxrange2) continue;    // 超出最远距离，丢弃

      const double offset_time = pt.timestamp - lidar_data.start_time;
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    break;
  }

  default:
    return;
  }
  
  lidar_buffer_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "imu";

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


// ============================================================
// [LOOP-CLOSURE] 发布回环边 + 优化后的关键帧轨迹
// ============================================================
void ROSWrapper::pub_loop_closure(const std::vector<BASIC::SE3>& keyframe_poses,
                                  const std::vector<LoopEdgeInfo>& edges){
  if(keyframe_poses.empty()) return;

  const builtin_interfaces::msg::Time stamp = this->now();

  // ---- 回环节点 (SPHERE_LIST) ----
  visualization_msgs::msg::Marker marker_node;
  marker_node.header.frame_id = "world";
  marker_node.header.stamp = stamp;
  marker_node.action = visualization_msgs::msg::Marker::ADD;
  marker_node.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_node.ns = "loop_nodes";
  marker_node.id = 0;
  marker_node.pose.orientation.w = 1;
  marker_node.scale.x = 0.3;
  marker_node.scale.y = 0.3;
  marker_node.scale.z = 0.3;
  marker_node.color.r = 0.0f;
  marker_node.color.g = 0.8f;
  marker_node.color.b = 1.0f;
  marker_node.color.a = 1.0f;

  // ---- 回环边 (LINE_LIST) ----
  visualization_msgs::msg::Marker marker_edge;
  marker_edge.header.frame_id = "world";
  marker_edge.header.stamp = stamp;
  marker_edge.action = visualization_msgs::msg::Marker::ADD;
  marker_edge.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge.ns = "loop_edges";
  marker_edge.id = 1;
  marker_edge.pose.orientation.w = 1;
  marker_edge.scale.x = 0.1;
  marker_edge.color.r = 0.9f;
  marker_edge.color.g = 0.9f;
  marker_edge.color.b = 0.0f;
  marker_edge.color.a = 1.0f;

  for(const auto& edge : edges){
    if(edge.from_index >= keyframe_poses.size() ||
       edge.to_index >= keyframe_poses.size()) continue;

    const BASIC::SE3& pose_from = keyframe_poses[edge.from_index];
    const BASIC::SE3& pose_to = keyframe_poses[edge.to_index];

    geometry_msgs::msg::Point p_from, p_to;
    p_from.x = pose_from.t_.x();
    p_from.y = pose_from.t_.y();
    p_from.z = pose_from.t_.z();
    p_to.x = pose_to.t_.x();
    p_to.y = pose_to.t_.y();
    p_to.z = pose_to.t_.z();

    marker_node.points.push_back(p_from);
    marker_node.points.push_back(p_to);
    marker_edge.points.push_back(p_from);
    marker_edge.points.push_back(p_to);
  }

  if(!marker_edge.points.empty()){
    // 发 MarkerArray (节点 + 边), 与 fastlio_sam 的
    // /fast_lio_sam/mapping/loop_closure_constraints 一致:
    //   一个 topic 里装两个 marker, RViz 用 MarkerArray 显示类订阅.
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker_node);
    marker_array.markers.push_back(marker_edge);
    pub_loop_markers_->publish(marker_array);
  }

  // ---- 优化后的关键帧轨迹 ----
  nav_msgs::msg::Path loop_path;
  loop_path.header.frame_id = "world";
  loop_path.header.stamp = stamp;
  loop_path.poses.reserve(keyframe_poses.size());
  for(const auto& pose : keyframe_poses){
    geometry_msgs::msg::PoseStamped ps;
    ps.header = loop_path.header;
    ps.pose.position.x = pose.t_.x();
    ps.pose.position.y = pose.t_.y();
    ps.pose.position.z = pose.t_.z();
    // 注意: M3::coeffs() 是矩阵展开, 不是四元数, 必须走 SO3::coeffs().
    const BASIC::V4 q = BASIC::SO3(pose.R_).coeffs();
    ps.pose.orientation.x = q[0];
    ps.pose.orientation.y = q[1];
    ps.pose.orientation.z = q[2];
    ps.pose.orientation.w = q[3];
    loop_path.poses.push_back(ps);
  }
  pub_loop_path_->publish(loop_path);
}


// ============================================================
// [LOOP-CLOSURE] 发布"优化前"的关键帧轨迹 (原始 ESKF 位姿)
//   与 pub_loop_closure() 发的"优化后"轨迹叠在一起, 就能直接看出
//   回环优化把漂移的轨迹拉回了多少.
// ============================================================
void ROSWrapper::pub_loop_path_before(const std::vector<BASIC::SE3>& odom_poses){
  if(odom_poses.empty()) return;

  nav_msgs::msg::Path path_before;
  path_before.header.frame_id = "world";
  path_before.header.stamp = this->now();
  path_before.poses.reserve(odom_poses.size());
  for(const auto& pose : odom_poses){
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_before.header;
    ps.pose.position.x = pose.t_.x();
    ps.pose.position.y = pose.t_.y();
    ps.pose.position.z = pose.t_.z();
    const BASIC::V4 q = BASIC::SO3(pose.R_).coeffs();
    ps.pose.orientation.x = q[0];
    ps.pose.orientation.y = q[1];
    ps.pose.orientation.z = q[2];
    ps.pose.orientation.w = q[3];
    path_before.poses.push_back(ps);
  }
  pub_loop_path_before_->publish(path_before);
}


// ============================================================
// [LOOP-CLOSURE] 发布优化后的全局地图 (关键帧重拼接)
//   对应 fastlio_sam 的 fast_lio_sam/mapping/map_global_optimized
// ============================================================
void ROSWrapper::pub_loop_map_optimized(const BASIC::CloudPtr& cloud){
  if(!cloud || cloud->empty()) return;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = "world";
  msg.header.stamp = this->now();
  pub_loop_map_optimized_->publish(msg);
}


// ============================================================
// [LOOP-CLOSURE] 发布 ICP 匹配的点云
//   history_cloud  : 历史 submap (世界系)
//   corrected_cloud: 当前帧经 ICP 修正后的点云 (世界系)
//   raw_cloud      : 当前帧修正前的点云 (世界系)
// 对应 fastlio_sam 的:
//   fast_lio_sam/mapping/icp_loop_closure_history_cloud
//   fast_lio_sam/mapping/icp_loop_closure_corrected_cloud
// ============================================================
void ROSWrapper::pub_loop_icp_clouds(
    const pcl::PointCloud<pcl::PointXYZI>& history_cloud,
    const pcl::PointCloud<pcl::PointXYZI>& corrected_cloud,
    const pcl::PointCloud<pcl::PointXYZI>& raw_cloud){
  const builtin_interfaces::msg::Time stamp = this->now();

  if(!history_cloud.empty()){
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(history_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;
    pub_loop_icp_history_->publish(msg);
  }

  if(!corrected_cloud.empty()){
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(corrected_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;
    pub_loop_icp_corrected_->publish(msg);
  }

  // raw_cloud (修正前) 发到独立 topic.
  // 这样在 RViz 里把 corrected + raw 都加上, 直接看到 ICP 把当前帧搬了多远.
  if(!raw_cloud.empty()){
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(raw_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;
    pub_loop_icp_raw_->publish(msg);
  }
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              0.2;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


} // namespace END.