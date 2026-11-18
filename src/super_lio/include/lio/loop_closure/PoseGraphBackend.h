#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "basic/Manifold.h"

namespace LI2Sup {

/// GTSAM ISAM2 位姿图后端.
///
/// 坐标系约定: 所有位姿都是 T_world_body (imu/robot 系), 与 kf_->GetSE3() 同系.
/// 因此 super_lio 的关键帧点云 (imu 系) 可以直接用该位姿变换到世界系,
/// 不需要像 fastlio_sam 那样额外处理 lidar->body 外参.
///
/// 节点编号 = 关键帧索引. 节点 0 携带先验因子, 锚定整个图.
///
/// GTSAM 约定:
///   BetweenFactor(k1, k2, z) 约束  between(x_k1, x_k2) = z,
///   而 between(a, b) = a^{-1} * b.
///   所以里程计因子 z = T_prev^{-1} * T_curr,
///   回环因子   z = T_cur^{-1} * T_pre.
///
/// 线程模型: 本类不做加锁, 由调用方 (LoopClosureManager) 保证同一时刻只有一个线程访问.
class PoseGraphBackend {
public:
  using Ptr = std::shared_ptr<PoseGraphBackend>;

  struct Options {
    /// 里程计因子噪声 (1-sigma): 旋转 rad, 平移 m
    double odom_rot_sigma = 1e-3;
    double odom_trans_sigma = 1e-2;
    /// 回环因子默认噪声 (1-sigma)
    double loop_rot_sigma = 5e-3;
    double loop_trans_sigma = 5e-2;
    /// 先验因子噪声 (1-sigma)
    double prior_rot_sigma = 1e-6;
    double prior_trans_sigma = 1e-6;
  };

  PoseGraphBackend();
  ~PoseGraphBackend();

  void configure(const Options& options);
  const Options& options() const { return options_; }

  /// 添加先验因子 (锚定节点 0). 同一 id 重复调用会覆盖.
  void addPriorFactor(std::size_t id, const BASIC::SE3& pose);

  /// 添加里程计因子: 约束 T_{id-1->id} = pose_prev^{-1} * pose_curr.
  void addOdomFactor(std::size_t id, const BASIC::SE3& pose_prev,
                     const BASIC::SE3& pose_curr);

  /// 添加回环因子: 约束 between(x_id_from, x_id_to) = T_between.
  /// @param id_from   当前关键帧 (较新)
  /// @param id_to     匹配到的历史关键帧 (较旧)
  /// @param T_between T_{id_from->id_to} = pose_from^{-1} * pose_to
  /// @param rot_sigma   旋转噪声 (1-sigma), <= 0 时用默认值
  /// @param trans_sigma 平移噪声 (1-sigma), <= 0 时用默认值
  void addLoopFactor(std::size_t id_from, std::size_t id_to,
                     const BASIC::SE3& T_between,
                     double rot_sigma = -1.0, double trans_sigma = -1.0);

  /// 执行优化.
  /// @param poses_out  输出所有节点的最新估计 (与节点编号一一对应)
  /// @return 有解且节点数 > 0 时返回 true
  bool optimize(std::vector<BASIC::SE3>& poses_out);

  /// 取上一次优化得到的某个节点的位姿估计.
  /// @note 用于构造里程计因子: 上一关键帧应该用【优化后】的位姿,
  ///       否则回环后新旧里程计因子会不自洽.
  bool getEstimate(std::size_t id, BASIC::SE3& out) const;

  /// 当前是否还有待处理的因子.
  bool hasPendingFactors() const { return pending_factors_ > 0; }

  /// 因子图当前的残差 (chi2), 仅用于诊断.
  double currentResidual() const { return last_residual_; }

  std::size_t nodeCount() const { return node_count_; }

  /// 已接受的回环因子数量.
  std::size_t loopFactorCount() const { return loop_factor_count_; }

  /// 重置整个图 (重新开始).
  void reset();

private:
  static gtsam::Pose3 toGtsam(const BASIC::SE3& pose);
  static BASIC::SE3 fromGtsam(const gtsam::Pose3& pose);

  Options options_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  /// 完整因子图 (用于计算残差), 与 isam_ 同步累积.
  gtsam::NonlinearFactorGraph full_graph_;
  /// 尚未提交给 ISAM2 的因子.
  gtsam::NonlinearFactorGraph pending_graph_;
  /// 尚未提交给 ISAM2 的变量初值.
  gtsam::Values pending_initial_;
  int pending_factors_ = 0;
  /// 本次待提交因子中回环因子的个数.
  /// 回环因子需要更多迭代才能充分收敛, 纯里程计因子增量很小, 一次 update 即可.
  int pending_loop_factors_ = 0;
  double last_residual_ = std::numeric_limits<double>::infinity();
  std::size_t node_count_ = 0;
  std::size_t loop_factor_count_ = 0;
  /// 上一次优化得到的位姿估计 (索引 = 节点编号).
  std::vector<BASIC::SE3> estimates_;
};

} // namespace LI2Sup
