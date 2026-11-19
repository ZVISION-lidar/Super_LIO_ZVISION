#pragma once

#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

#include "basic/Manifold.h"
#include "lio/loop_closure/KeyframeManager.h"

namespace LI2Sup {

/// 一条回环约束.
struct LoopConstraint {
  bool valid = false;
  /// 当前关键帧索引 (较新)
  std::size_t from_index = 0;
  /// 匹配到的历史关键帧索引 (较旧)
  std::size_t to_index = 0;
  /// between(x_from, x_to) = x_from^{-1} * x_to, 直接作为 GTSAM BetweenFactor 的观测值.
  BASIC::SE3 T_from_to;
  /// ICP 匹配质量 (平均对应点距离, 越小越好)
  double fitness_score = 0.0;
  /// Scan Context 给出的航向差 (rad)
  double yaw_difference_rad = 0.0;
};

/// 回环检测器: Scan Context 候选检索 + ICP 几何验证.
///
/// 流程 (与 fastlio_sam 的 performLoopClosure 一致):
///   1. Scan Context 检索候选 (排除最近 N 帧);
///   2. 取当前关键帧单帧点云 (世界系) 作为 ICP 的 source;
///   3. 取历史关键帧前后若干帧拼成的 submap (世界系) 作为 ICP 的 target;
///   4. ICP 对齐, 未收敛或 fitness 超阈值则拒绝;
///   5. 由 ICP 结果导出相对位姿约束.
///
/// 坐标说明: super_lio 的关键帧点云保存在 IMU 系, 与 kf_->GetSE3() 同系,
/// 因此可以直接用位姿变换到世界系, 无需 lidar<->body 外参转换.
class LoopDetector {
public:
  struct Options {
    /// ICP 最大对应点距离 (m)
    int icp_max_correspondence_distance = 15;
    /// ICP 最大迭代次数
    int icp_max_iterations = 100;
    /// ICP 收敛阈值
    double icp_transformation_epsilon = 1e-6;
    double icp_euclidean_fitness_epsilon = 1e-6;
    /// ICP fitness score 阈值 (越小越严格)
    double icp_fitness_score_threshold = 0.6;
    /// 历史 submap 前后各取几个关键帧
    int history_search_num = 5;
  };

  LoopDetector() = default;

  void configure(const Options& options) { options_ = options; }
  const Options& options() const { return options_; }

  /// 执行一次回环检测.
  /// @param kfm  关键帧管理器 (提供 Scan Context 与点云)
  /// @param out 检测结果
  /// @return 找到并通过几何验证的回环时返回 true
  bool detect(const KeyframeManager& kfm, LoopConstraint& out);

  /// 已接受的历史回环对 (from -> to), 供可视化使用.
  const std::vector<std::pair<std::size_t, std::size_t>>& acceptedLoops() const {
    return accepted_loops_;
  }

  /// [可视化] 最近一次 ICP 匹配的点云 (世界系).
  ///
  /// 对应 fastlio_sam 的两个 topic:
  ///   fast_lio_sam/mapping/icp_loop_closure_history_cloud   <- history_cloud
  ///   fast_lio_sam/mapping/icp_loop_closure_corrected_cloud <- cur_cloud 变换后
  ///
  /// 用途: 在 RViz 里直观看到"当前帧被 ICP 拉到了历史帧的位置",
  ///       以及匹配用的历史 submap.
  struct IcpVisualization {
    /// 历史 submap (世界系, 降采样后).
    pcl::PointCloud<pcl::PointXYZI> history_cloud;
    /// 当前关键帧经 ICP 修正后的点云 (世界系, 降采样后).
    pcl::PointCloud<pcl::PointXYZI> corrected_cloud;
    /// 当前关键帧经 ICP 修正前的点云 (世界系, 降采样后).
    pcl::PointCloud<pcl::PointXYZI> raw_cloud;
    /// 本次匹配的索引对.
    std::size_t from_index = 0;
    std::size_t to_index = 0;
    double fitness_score = 0.0;
    bool valid = false;
    /// 检出序号 (每成功检出一次回环 +1).
    /// 用于判断"是不是新的回环", 从而只在新的回环出现时才发布可视化点云.
    std::size_t seq = 0;
  };

  /// 取最近一次 ICP 匹配的调试点云 (线程安全).
  ///
  /// 注意: detect() 在后台线程里写, 这里在主线程读, 必须加锁.
  IcpVisualization lastIcpVisualization() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return icp_viz_;
  }

private:
  /// 可视化数据的锁 (后台线程写, 主线程读).
  mutable std::mutex mutex_;
  Options options_;
  std::vector<std::pair<std::size_t, std::size_t>> accepted_loops_;
  /// 最近一次 ICP 匹配的点云 (可视化用).
  IcpVisualization icp_viz_;
  /// ICP 检出计数 (每成功一次 +1), 用于判断是否是新回环.
  std::size_t icp_viz_seq_ = 0;
};

} // namespace LI2Sup
