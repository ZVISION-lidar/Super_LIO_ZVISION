#pragma once

#include <atomic>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "basic/Manifold.h"
#include "lio/ESKF.h"
#include "lio/loop_closure/KeyframeManager.h"
#include "lio/loop_closure/LoopDetector.h"
#include "lio/loop_closure/PoseGraphBackend.h"
#include "lio/params.h"

namespace LI2Sup {

/// 回环检测 + 位姿图优化 的统一管理器.
///
/// ================================================================
/// 线程模型 (重要)
/// ================================================================
///   - 后台线程: 只做"回环检测" (Scan Context 检索 + ICP 几何验证).
///     这部分耗时长 (ICP 可能几十~几百 ms), 但它只读关键帧数据,
///     不碰 ESKF, 也不碰 ISAM2. 检测出的约束放进队列, 由主线程消费.
///
///   - 主线程: 消费约束 -> 加入位姿图 -> ISAM2 优化 -> 回写 ESKF.
///     优化与回写必须在主线程做, 否则会和 EKF 更新竞争,
///     且"回写修正量"的参考系一致性无法保证.
///
///   注意: 后台线程做 ICP 时【不持有】本类的 mutex_,
///         否则 ICP 的几百 ms 会直接卡住前端主线程.
///         KeyframeManager 有自己的锁, 快照取完立刻释放.
///
/// ================================================================
/// 坐标系与"锚点-增量"约定 (关键)
/// ================================================================
///   - 所有位姿都是 T_world_body (imu 系), 与 kf_->GetSE3() 同系.
///     因此关键帧点云 (imu 系) 直接用该位姿变换到世界系,
///     不需要 lidar<->body 外参转换.
///
///   - 位姿图的先验因子把节点 0 锚定在"初始里程计系" = 图的世界系 A.
///     先验噪声很小 (默认 1e-4), 所以整图不会整体漂移.
///
///   - 开启回写时, ESKF 的位姿始终被"钉"在图上:
///         ESKF 位姿 = X_ref * Δ
///     其中 X_ref 是参考节点 (最新关键帧) 在图内的估计值,
///     Δ 是参考节点之后累积的纯相对运动.
///
///     1) 新建关键帧 k 时 (主线程):
///          里程计因子 Z_k = X_{k-1}^{-1} * P_k
///        (X_{k-1} = 节点 k-1 的图内估计, P_k = 当前 ESKF 位姿)
///        因为 ESKF 在关键帧 k-1 处已被钉到 X_{k-1}, 所以 Z_k 恰好是
///        这段纯里程计运动, 不含任何回环修正.
///        优化后 X_k 出现偏移 -> corr = X_k * P_k^{-1}, 把 ESKF 重新钉到 X_k.
///
///     2) 回环优化后 (主线程):
///          corr = X_ref' * X_ref^{-1}      (X_ref' 为优化后的新值)
///        施加 T_new = corr * T_cur = X_ref' * Δ
///        即"参考节点被优化成什么样, 当前位姿就跟着变成什么样",
///        同时严格保留参考节点之后的相对运动.
///
///        多次回环天然可组合: 每次只用"本次变化量", 不会重复施加.
///
///   - 关闭回写时, ESKF 保持纯里程计, 图是独立的一条链.
///     此时关键帧判定与里程计因子全部使用【原始位姿】.
///
///   - 在线地图 (OctVoxMap) 不做重建 (与 fastlio_sam 一致).
///     地图一致性由"关键帧重拼接保存"保证 (见 SuperLIO::saveMap).
class LoopClosureManager {
public:
  using Ptr = std::shared_ptr<LoopClosureManager>;

  LoopClosureManager();
  ~LoopClosureManager();

  void configure();
  void start();
  void stop();

  /// ==============================================================
  /// 主线程接口
  /// ==============================================================

  /// 判断当前位姿是否需要成为关键帧, 若是则记录关键帧 + 加入里程计因子,
  /// 并立即做一次增量优化, 把 ESKF 重新钉到新的图估计值上.
  ///
  /// @param pose     当前位姿 (world <- imu)
  /// @param cloud    当前帧点云 (imu 系)
  /// @param out_corr 输出的回写修正量 (T_new = out_corr * T_cur)
  /// @return 本次新增了关键帧时返回 true
  bool maybeAddKeyframe(const BASIC::SE3& pose,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                        BASIC::SE3& out_corr);

  /// 消费后台线程检测到的回环约束, 执行位姿图优化, 并生成回写修正量.
  ///
  /// @param out_corr 输出的回写修正量 (T_new = out_corr * T_cur)
  /// @return 产生了新的优化结果 (需要回写) 时返回 true
  bool processLoops(BASIC::SE3& out_corr);

  /// ==============================================================
  /// 后台线程接口
  /// ==============================================================

  /// 执行一次回环检测, 检出则把约束放进队列.
  /// @return 本次是否检测到并接受了回环
  bool runOnce();

  /// ==============================================================
  /// 查询接口
  /// ==============================================================

  /// 关键帧管理器 (供地图保存使用).
  KeyframeManager::Ptr keyframeManager() const { return kfm_; }

  /// 优化后的关键帧位姿 (用于发布可视化轨迹).
  std::vector<BASIC::SE3> optimizedPoses() const;

  /// 已接受的回环边 (含 ICP fitness), 用于发布可视化边.
  struct LoopEdge {
    std::size_t from_index = 0;
    std::size_t to_index = 0;
    double fitness_score = 0.0;
  };
  std::vector<LoopEdge> loopEdges() const;

  /// 已接受的回环边 (只含索引对), 便于外部快速遍历.
  std::vector<std::pair<std::size_t, std::size_t>> loopPairs() const;

  /// 累计回环次数.
  std::size_t loopCount() const;

private:
  void loopThread();

  /// 修正量是否显著 (过滤微小数值抖动).
  static bool isCorrectionSignificant(const BASIC::SE3& corr);

  mutable std::mutex mutex_;

  KeyframeManager::Ptr kfm_;
  LoopDetector detector_;
  PoseGraphBackend::Ptr backend_;

  /// 后台线程检测到的回环约束队列 (主线程消费).
  std::vector<LoopConstraint> pending_constraints_;

  /// 已经建立过回环边的"当前关键帧"索引 (避免同一帧反复加边).
  std::vector<std::size_t> looped_from_;

  std::vector<LoopEdge> loop_edges_;
  std::size_t loop_count_ = 0;

  /// 参考节点: 最新关键帧 (与 ESKF 同步的锚点).
  std::size_t ref_index_ = std::numeric_limits<std::size_t>::max();
  /// 参考节点在上次同步时的图内位姿.
  BASIC::SE3 ref_graph_pose_;

  std::atomic<bool> running_{false};
  std::thread thread_;
};

} // namespace LI2Sup
