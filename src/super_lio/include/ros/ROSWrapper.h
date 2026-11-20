
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <cmath>
#include <execution>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <pcl_conversions/pcl_conversions.h>

/// msgs
#include "super_lio/msg/cloud_pose.hpp"
#include "super_lio/msg/cloud_pose2.hpp"

/// ZVISION LiDAR point structure (NZ1 / EZ6 series)
/// MUST match ROS PointCloud2 field offsets exactly:
///   x:0(float32) y:4(float32) z:8(float32) intensity:16(uint8)
///   ring:18(uint16) timestamp:24(float64)  point_step=32
namespace zvision_ros
{
#pragma pack(push, 1)
struct Point
{
    PCL_ADD_POINT4D;                  // x:0 y:4 z:8
    uint8_t  intensity;               // offset:16
    uint8_t  pad0;                    // offset:17 (padding)
    uint16_t ring;                    // offset:18
    double   timestamp;                // offset:24
};
#pragma pack(pop)
}  // namespace zvision_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(zvision_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, BASIC::CloudPtr& point_cloud);

/// 一条回环边 (用于可视化).
struct LoopEdgeInfo {
  std::size_t from_index = 0;
  std::size_t to_index = 0;
  double fitness_score = 0.0;
};

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper(){};
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}

  void clear(){
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
    resetFilters();
  }

  /// Reset IMU filter states (call on re-initialization)
  void resetFilters(){
    for (int i = 0; i < 3; ++i) {
      notch_acc_[i].reset();
      notch_gyr_[i].reset();
      lp_acc_[i].reset();
    }
    imu_filt_initialized_ = false;
  }

  void pub_odom(const NavState&);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc, 
                           const NavState& state);
  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);  
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  // ============================================================
  // [LOOP-CLOSURE] 回环可视化
  // ============================================================
  /// 发布回环 marker (节点+边) 与优化后的关键帧轨迹.
  void pub_loop_closure(const std::vector<BASIC::SE3>& keyframe_poses,
                        const std::vector<LoopEdgeInfo>& edges);
  /// 发布"优化前"的关键帧轨迹 (原始 ESKF 位姿), 与优化后的对比.
  void pub_loop_path_before(const std::vector<BASIC::SE3>& odom_poses);
  /// 发布优化后的全局地图 (关键帧重拼接).
  void pub_loop_map_optimized(const BASIC::CloudPtr& cloud);
  /// 发布 ICP 匹配的点云 (历史 submap / 修正后 / 修正前).
  void pub_loop_icp_clouds(
      const pcl::PointCloud<pcl::PointXYZI>& history_cloud,
      const pcl::PointCloud<pcl::PointXYZI>& corrected_cloud,
      const pcl::PointCloud<pcl::PointXYZI>& raw_cloud);

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupParams();
  void setupIO();

  // ============================================================
  // IMU 振动抑制滤波器
  //   acc : Notch(多频点级联) -> LowPass
  //   gyr : Notch(多频点级联)
  //   Notch 精准消除旋翼振动主频且几乎不引入相位延迟；
  //   低通兜底宽带噪声与谐波。
  // ============================================================
  void initFilters();
  void filterIMU(const IMUData& in, IMUData& out);

  // ============================================================
  // 滤波器结构体
  // ============================================================

  /// 二阶 Butterworth 低通（双线性变换，Q = 0.7071）
  struct BiquadLP {
    double b0, b1, b2, a1, a2;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    /// 默认构造（直通 1），之后由 setParams 重新配置
    BiquadLP() { b0 = 1.0; b1 = b2 = a1 = a2 = 0.0; }

    BiquadLP(double fc, double fs, double Q = 0.7071) {
      const double w0 = 2.0 * M_PI * fc / fs;
      const double cosw0 = std::cos(w0);
      const double sinw0 = std::sin(w0);
      const double alpha = sinw0 / (2.0 * Q);
      const double a0 = 1.0 + alpha;
      b0 = (1.0 - cosw0) / 2.0 / a0;
      b1 = (1.0 - cosw0) / a0;
      b2 = b0;
      a1 = -2.0 * cosw0 / a0;
      a2 = (1.0 - alpha) / a0;
    }

    /// 重新配置系数（保留当前状态）
    void setParams(double fc, double fs, double Q = 0.7071) {
      const double nx1 = x1, nx2 = x2, ny1 = y1, ny2 = y2;
      *this = BiquadLP(fc, fs, Q);
      x1 = nx1; x2 = nx2; y1 = ny1; y2 = ny2;
    }

    inline double update(double x) {
      const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
      x2 = x1; x1 = x;
      y2 = y1; y1 = y;
      return y;
    }

    void reset() { x1 = x2 = y1 = y2 = 0.0; }
  };

  /// 二阶陷波器（Notch）
  struct Notch {
    double b0, b1, b2, a1, a2;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

    /// 默认构造（直通 1）
    Notch() { b0 = 1.0; b1 = b2 = a1 = a2 = 0.0; }

    Notch(double f0, double fs, double Q = 15.0) {
      const double w0 = 2.0 * M_PI * f0 / fs;
      const double alpha = std::sin(w0) / (2.0 * Q);
      const double cosw0 = std::cos(w0);
      const double a0 = 1.0 + alpha;
      b0 = 1.0 / a0;
      b1 = -2.0 * cosw0 / a0;
      b2 = 1.0 / a0;
      a1 = -2.0 * cosw0 / a0;
      a2 = (1.0 - alpha) / a0;
    }

    /// 重新配置系数（保留当前状态）
    void setParams(double f0, double fs, double Q = 15.0) {
      const double nx1 = x1, nx2 = x2, ny1 = y1, ny2 = y2;
      *this = Notch(f0, fs, Q);
      x1 = nx1; x2 = nx2; y1 = ny1; y2 = ny2;
    }

    inline double update(double x) {
      const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
      x2 = x1; x1 = x;
      y2 = y1; y1 = y;
      return y;
    }

    void reset() { x1 = x2 = y1 = y2 = 0.0; }
  };

  /// 多频点陷波器级联
  ///   无人机震动通常不止一个频率 (基频 + 多次谐波 + 机械共振),
  ///   单个陷波器只能挖掉一条谱线, 故支持级联多个 Notch。
  struct NotchCascade {
    static constexpr int kMax = 4;
    Notch nt[kMax];
    int n = 0;

    NotchCascade() {}

    /// 用频率列表配置, 跳过 f0 <= 0 的项
    void setFreqs(const std::vector<double>& freqs, double fs, double Q) {
      n = 0;
      for (double f : freqs) {
        if (f <= 0.0 || n >= kMax) continue;
        nt[n].setParams(f, fs, Q);
        ++n;
      }
    }

    inline double update(double x) {
      double y = x;
      for (int i = 0; i < n; ++i) y = nt[i].update(y);
      return y;
    }

    void reset() {
      for (int i = 0; i < kMax; ++i) nt[i].reset();
    }
  };

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;

  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;

  ESKF::Ptr eskf_{nullptr};

  // ============================================================
  // IMU 滤波器成员
  // ============================================================
  NotchCascade notch_acc_[3];
  NotchCascade notch_gyr_[3];
  BiquadLP lp_acc_[3];
  bool imu_filt_initialized_ = false;

  /// 滤波器参数（可由配置文件覆盖）
  /// [默认关闭] 滤波会引入相位延迟, 且与 FastLIVO2 的裸 IMU 直推行为不一致.
  bool imu_notch_enable_ = false;  ///< 陷波开关
  bool imu_lp_enable_ = false;     ///< 低通开关
  double imu_filt_fs_ = 197.98;   ///< IMU 采样率 (Hz)
  std::vector<double> imu_notch_f0s_;  ///< 陷波中心频率列表 (Hz)
  double imu_notch_q_ = 8.0;      ///< 陷波 Q 值 (带宽 = f0/Q)
  double imu_lp_fc_ = 15.0;       ///< 低通截止频率 (Hz)

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// output.
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;

  // ============================================================
  // [LOOP-CLOSURE] 回环可视化 publisher
  // ============================================================
  /// 回环节点 + 回环边 (MarkerArray)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_markers_;
  /// 优化后的关键帧轨迹
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_loop_path_;
  /// 优化前的关键帧轨迹 (原始 ESKF 位姿)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_loop_path_before_;
  /// 优化后的全局地图 (关键帧重拼接)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_loop_map_optimized_;
  /// ICP 匹配的点云
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_loop_icp_history_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_loop_icp_corrected_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_loop_icp_raw_;
};

} // namespace END.

#endif
