/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16 
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  /// IMU vibration suppression filter
  extern bool   g_imu_notch_enable;   ///< notch (rotor vibration) switch
  extern bool   g_imu_lp_enable;      ///< low-pass (broadband noise) switch
  extern double g_imu_filt_fs;        ///< IMU sample rate (Hz), used for coefficient design
  extern std::vector<double> g_imu_notch_f0s;  ///< notch center frequencies (Hz), empty = off
  extern double g_imu_notch_q;        ///< notch Q (bandwidth = f0 / Q)
  extern double g_imu_lp_fc;          ///< low-pass cutoff frequency (Hz)

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;
  /// Decimation factor of the high-frequency IMU/robot odometry topics.
  /// 1 = publish on every IMU sample, 2 = publish every other sample, ...
  extern int g_imu_odom_pub_divider;

  /// for planner
  extern bool g_planner_enable;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// ============================================================
  /// [LOOP-CLOSURE] 回环检测 + 位姿图优化
  /// ============================================================
  /// 总开关. false 时不启动回环线程, 行为与原来完全一致.
  extern bool g_loop_closure_enable;
  /// 回环检测线程频率 (Hz)
  extern double g_loop_closure_frequency;

  /// 关键帧判定阈值
  extern double g_keyframe_dist;      ///< 位移阈值 (m)
  extern double g_keyframe_angle;     ///< 角度阈值 (rad)

  /// Scan Context 参数
  extern int    g_sc_num_rings;       ///< 环数 (径向)
  extern int    g_sc_num_sectors;     ///< 扇区数 (角向)
  extern double g_sc_max_radius;      ///< 最大半径 (m)
  extern double g_sc_lidar_height;    ///< 雷达安装高度补偿 (m)
  extern int    g_sc_exclude_recent;  ///< 排除最近 N 个关键帧
  extern int    g_sc_num_candidates;  ///< 候选数量
  extern double g_sc_search_ratio;    ///< 角度搜索范围比例
  extern double g_sc_distance_threshold; ///< 描述子距离阈值

  /// ICP 几何验证参数
  extern int    g_icp_max_correspondence_distance;  ///< 最大对应点距离
  extern int    g_icp_max_iterations;
  extern double g_icp_fitness_score_threshold;     ///< 匹配质量阈值 (越小越好)
  extern int    g_icp_history_search_num;          ///< 历史 submap 前后各取几个关键帧

  /// GTSAM 位姿图优化参数 (噪声 1-sigma)
  extern double g_pg_odom_rot_sigma;
  extern double g_pg_odom_trans_sigma;
  extern double g_pg_loop_rot_sigma;
  extern double g_pg_loop_trans_sigma;
  extern double g_pg_prior_rot_sigma;
  extern double g_pg_prior_trans_sigma;

  /// 回环优化后是否把优化位姿回写到 ESKF (前端里程计对齐)
  extern bool   g_loop_writeback_enable;
  /// 回环优化后是否用关键帧重拼接地图 (SaveMapByKeyframe)
  extern bool   g_loop_keyframe_map_enable;

}

#endif
