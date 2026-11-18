

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;

  /// IMU vibration suppression filter
  /// [默认关闭] 滤波会引入相位延迟, 且与 FastLIVO2 的裸 IMU 直推行为不一致.
  ///   需要开启时在 yaml 里显式设置 imu_notch_enable / imu_lp_enable: true.
  bool   g_imu_notch_enable = false;
  bool   g_imu_lp_enable   = false;
  double g_imu_filt_fs     = 197.98;
  std::vector<double> g_imu_notch_f0s = {43.42, 19.61};
  double g_imu_notch_q     = 8.0;
  double g_imu_lp_fc       = 15.0;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  double g_kf_quit_eps;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;
  int  g_imu_odom_pub_divider = 1;   // 1: publish at the IMU rate

  /// for planner
  bool g_planner_enable;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// ============================================================
  /// [LOOP-CLOSURE] 回环检测 + 位姿图优化
  /// ============================================================
  bool   g_loop_closure_enable = false;
  double g_loop_closure_frequency = 2.0;

  double g_keyframe_dist = 1.0;
  double g_keyframe_angle = 0.2;

  int    g_sc_num_rings = 20;
  int    g_sc_num_sectors = 60;
  double g_sc_max_radius = 80.0;
  double g_sc_lidar_height = 2.0;
  int    g_sc_exclude_recent = 30;
  int    g_sc_num_candidates = 10;
  double g_sc_search_ratio = 0.1;
  double g_sc_distance_threshold = 0.15;

  int    g_icp_max_correspondence_distance = 15;
  int    g_icp_max_iterations = 100;
  double g_icp_fitness_score_threshold = 0.6;
  int    g_icp_history_search_num = 5;

  // GTSAM 噪声 (1-sigma).
  // 里程计因子比回环因子更可信, 所以 sigma 更小.
  double g_pg_odom_rot_sigma = 1e-3;
  double g_pg_odom_trans_sigma = 1e-2;
  double g_pg_loop_rot_sigma = 5e-3;
  double g_pg_loop_trans_sigma = 5e-2;
  // 先验因子把节点 0 锚死在初始里程计系, sigma 要极小.
  double g_pg_prior_rot_sigma = 1e-6;
  double g_pg_prior_trans_sigma = 1e-6;

  bool   g_loop_writeback_enable = true;
  bool   g_loop_keyframe_map_enable = true;

}