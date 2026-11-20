#include "lio/loop_closure/LoopClosureManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

#include "basic/logs.h"

namespace LI2Sup {

namespace {
/// 修正量小于该阈值就认为是恒等变换, 直接跳过 (避免数值抖动).
constexpr double kMinCorrectionTrans = 1e-6;
constexpr double kMinCorrectionRot = 1e-7;
} // namespace

LoopClosureManager::LoopClosureManager() {
  kfm_.reset(new KeyframeManager());
  backend_.reset(new PoseGraphBackend());
}

LoopClosureManager::~LoopClosureManager() {
  stop();
}

void LoopClosureManager::configure() {
  // ---- 关键帧管理器 ----
  KeyframeManager::Options kfm_options;
  kfm_options.keyframe_dist = g_keyframe_dist;
  kfm_options.keyframe_angle = g_keyframe_angle;
  kfm_options.scan_context.num_rings = g_sc_num_rings;
  kfm_options.scan_context.num_sectors = g_sc_num_sectors;
  kfm_options.scan_context.max_radius = g_sc_max_radius;
  kfm_options.scan_context.lidar_height = g_sc_lidar_height;
  kfm_options.scan_context.exclude_recent = g_sc_exclude_recent;
  kfm_options.scan_context.num_candidates = g_sc_num_candidates;
  kfm_options.scan_context.search_ratio = g_sc_search_ratio;
  kfm_options.scan_context.distance_threshold = g_sc_distance_threshold;
  kfm_->configure(kfm_options);

  // ---- 回环检测器 ----
  LoopDetector::Options detector_options;
  detector_options.icp_max_correspondence_distance =
      g_icp_max_correspondence_distance;
  detector_options.icp_max_iterations = g_icp_max_iterations;
  detector_options.icp_fitness_score_threshold =
      g_icp_fitness_score_threshold;
  detector_options.history_search_num = g_icp_history_search_num;
  detector_.configure(detector_options);

  // ---- 位姿图后端 ----
  PoseGraphBackend::Options backend_options;
  backend_options.odom_rot_sigma = g_pg_odom_rot_sigma;
  backend_options.odom_trans_sigma = g_pg_odom_trans_sigma;
  backend_options.loop_rot_sigma = g_pg_loop_rot_sigma;
  backend_options.loop_trans_sigma = g_pg_loop_trans_sigma;
  backend_options.prior_rot_sigma = g_pg_prior_rot_sigma;
  backend_options.prior_trans_sigma = g_pg_prior_trans_sigma;
  backend_->configure(backend_options);
}

void LoopClosureManager::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread(&LoopClosureManager::loopThread, this);
  LOG(INFO) << GREEN << " ---> [LoopClosure]: thread started, frequency = "
            << g_loop_closure_frequency << " Hz" << RESET;
}

void LoopClosureManager::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
  LOG(INFO) << GREEN << " ---> [LoopClosure]: thread stopped." << RESET;
}

// ============================================================
// 关键帧记录 (主线程)
// ============================================================
bool LoopClosureManager::maybeAddKeyframe(
    const BASIC::SE3& pose,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    BASIC::SE3& out_corr) {
  std::size_t index = 0;
  bool added = false;
  std::vector<BASIC::SE3> optimized;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 关键帧判定与插入必须在同一把锁内, 否则后台线程可能读到不一致的索引.
    //
    // 判定基准与里程计因子必须【保持一致】, 这是自洽性要求:
    //   writeback=true  -> 用图内估计值 (ESKF 已被钉在图上, 两者同源)
    //   writeback=false -> 用原始位姿   (前端保持纯里程计, 图是独立的链)
    //   两端混用会让图内链条与判定基准脱节.
    // (fastlio_sam 只有一条路径, 因为它永远都把 ESKF 钉在图上.)
    if (!kfm_->shouldCreateKeyframe(pose, g_loop_writeback_enable)) return false;

    index = kfm_->addKeyframe(pose, cloud);
    added = true;

    // 加入位姿图.
    if (index == 0) {
      // 第一个关键帧: 加先验因子, 锚定整个图.
      backend_->addPriorFactor(index, pose);
    } else {
      // 里程计因子: z = between(x_{k-1}, x_k) = T_{k-1}^{-1} * T_k.
      //
      // T_prev 的取法必须与上面的判定基准保持一致:
      //
      //   writeback=true:
      //     用【图内估计值】(节点 k-1 的当前优化结果).
      //     fastlio_sam 用的就是 cloudKeyPoses6D->back(), 也就是优化后的
      //     上一关键帧位姿, 语义完全一致:
      //         z = X_{k-1}^{opt} .between( P_k )
      //     此时 P_k 是当前 ESKF 位姿 (已被钉在图上), 所以 z 恰好是那段
      //     纯里程计运动, 不含回环修正.
      //
      //     【不能用】"关键帧建立时固定的原始位姿": 回环把节点 k-1 拉变形后,
      //     那个固定值就过期了, 会让图内节点 k 偏离前端,
      //     下一关键帧的修正量会变成 X'_{k-1} * X_{k-1}^{old}^{-1},
      //     等于把回环形变【施加第二遍】 —— 这正是"轨迹飞"的成因.
      //
      //   writeback=false:
      //     两端都用【原始位姿】, 保证图内是纯里程计链.
      BASIC::SE3 T_prev;
      if (g_loop_writeback_enable) {
        if (!backend_->getEstimate(index - 1, T_prev)) {
          // 图内还没有该节点的估计 (第一次优化前), 退回关键帧的原始位姿.
          const std::vector<BASIC::SE3> odom_poses = kfm_->getOdomPoses();
          T_prev = (index - 1 < odom_poses.size()) ? odom_poses[index - 1]
                                                    : BASIC::SE3();
        }
      } else {
        const std::vector<BASIC::SE3> odom_poses = kfm_->getOdomPoses();
        T_prev = (index - 1 < odom_poses.size()) ? odom_poses[index - 1]
                                                  : BASIC::SE3();
      }
      // 注意: 这里【不能】提前 return —— 否则节点 index 会成为一个
      // 没有任何因子连接的孤立变量, GTSAM 求解线性系统时必然抛出
      // IndeterminantLinearSystemException.
      backend_->addOdomFactor(index, T_prev, pose);
    }

    // 顺手跑一次增量优化, 让 ISAM2 的估计值保持最新 (增量更新很快).
    if (!backend_->optimize(optimized)) return true;
    kfm_->updatePoses(optimized);
  }

  // ============================================================
  // 生成回写修正量
  //
  //   不变式 (invariant):
  //       ESKF 当前位姿 = X_ref * Δ
  //   其中 X_ref 是参考节点 (最新关键帧) 在图内的估计值,
  //         Δ 是参考节点之后累积的纯相对运动.
  //
  //   情况 1: 新建关键帧 k (此刻 ESKF 位姿 = P_k)
  //     希望把参考点重置到节点 k, 即 ESKF 位姿变成 X_k (Δ 归零).
  //       corr = X_k * P_k^{-1}
  //     随后 ref_index_ = k, ref_graph_pose_ = X_k, 不变式重新成立.
  //
  //   情况 2: 回环优化后 (见 processLoops)
  //       corr = X_ref' * X_ref^{-1}
  //     ESKF 位姿 = X_ref' * Δ, 参考节点值更新为 X_ref'.
  //
  //   两种情况都是"用本次变化量", 所以多次回环天然可组合, 不会重复施加.
  // ============================================================
  if (g_loop_writeback_enable && index < optimized.size()) {
    const BASIC::SE3 x_new = optimized[index];
    const BASIC::SE3 corr = x_new * pose.inverse();
    if (isCorrectionSignificant(corr)) out_corr = corr;

    // 参考节点推进到最新关键帧.
    ref_index_ = index;
    ref_graph_pose_ = x_new;
  }

  return added;
}

// ============================================================
// 回环检测 (后台线程)
// ============================================================
bool LoopClosureManager::runOnce() {
  LoopConstraint constraint;

  // 注意: 这里【不能】持有 mutex_ 做 ICP, 否则会阻塞前端主线程.
  // detector_.detect() 内部通过 KeyframeManager 自己的锁取快照,
  // 快照取完立刻释放, 后面耗时的变换/ICP 都在锁外跑.
  if (!detector_.detect(*kfm_, constraint)) return false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 同一个"当前关键帧"不重复建立回环边.
    // (fastlio_sam 用 loopIndexContainer 做同样的去重.)
    if (std::find(looped_from_.begin(), looped_from_.end(),
                  constraint.from_index) != looped_from_.end()) {
      return false;
    }
    pending_constraints_.push_back(constraint);
  }

  LOG(INFO) << GREEN << " ---> [LoopClosure]: loop candidate detected, from "
            << constraint.from_index << " to " << constraint.to_index
            << ", fitness = " << constraint.fitness_score << RESET;
  return true;
}

// ============================================================
// 回环优化 (主线程)
// ============================================================
bool LoopClosureManager::processLoops(BASIC::SE3& out_corr) {
  std::vector<LoopConstraint> constraints;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_constraints_.empty()) return false;
    constraints.swap(pending_constraints_);
  }

  std::vector<BASIC::SE3> optimized;
  std::size_t used = 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const LoopConstraint& constraint : constraints) {
      if (!constraint.valid) continue;

      // 噪声由 ICP 的 fitness 决定.
      //
      // 注意: PCL 的 getFitnessScore() 返回的是【平均平方距离】,
      //       它本身就是一个"方差"量纲, 而不是标准差.
      //       fastlio_sam 就是直接拿它当 Variance 用的:
      //           Vector6 << noiseScore x6
      //       所以这里先取 sqrt 还原成 sigma, 再由 makeNoise 平方回 variance,
      //       最终送进 GTSAM 的数值与 fastlio_sam 完全一致.
      //       fitness 越大说明匹配越差 -> 约束越弱.
      const double variance = std::max(constraint.fitness_score, 1e-6);
      const double sigma = std::sqrt(variance);

      backend_->addLoopFactor(constraint.from_index, constraint.to_index,
                              constraint.T_from_to, sigma, sigma);
      loop_edges_.push_back({constraint.from_index, constraint.to_index,
                             constraint.fitness_score});
      ++loop_count_;
      ++used;
    }

    if (used == 0) return false;

    // ---- 优化 ----
    //
    // [DIAG] diag_enable = true 时打印残差诊断.
    //   只在【有回环】的优化里打印 —— 纯里程计增量优化每个关键帧都会跑,
    //   每次都打印会刷屏, 而且那些增量的残差本来就很小, 没有诊断价值.
    //   打印内容 = 一行 chi2 汇总 + 本次新增回环因子的明细.
    //   由参数 /lio/loop_closure/diag_enable 控制 (默认 false).
    if (!backend_->optimize(optimized, g_pg_diag_enable)) {
      // 优化失败 (GTSAM 抛异常或未收敛).
      //
      // 【不能】把约束重新入队重试:
      //   GTSAM 的 update() 是【先】把因子推进 nonlinearFactors_,
      //   【后】才做消元求解. 所以失败时这些回环因子其实已经进了图.
      //   如果重新入队再 addLoopFactor() 一次, 同一对节点之间就会有两份
      //   相同的 BetweenFactor -> 约束权重翻倍, 结果更糟.
      //
      // 所以这里的处理是: 放弃本批回环约束, 但保留图和关键帧,
      // 继续跑后面的关键帧 (fastlio_sam 在这里会直接崩, 我们更保守).
      // 回环的去重表照常登记, 避免同一帧被反复检测.
      for (const LoopConstraint& constraint : constraints) {
        if (!constraint.valid) continue;
        if (std::find(looped_from_.begin(), looped_from_.end(),
                      constraint.from_index) == looped_from_.end()) {
          looped_from_.push_back(constraint.from_index);
        }
      }
      LOG(WARNING) << YELLOW << " ---> [LoopClosure]: optimization failed, "
                   << "this batch of loops is dropped." << RESET;
      return false;
    }

    // ---- 回写关键帧位姿 (图内估计值) ----
    kfm_->updatePoses(optimized);

    // ---- 优化成功后才登记 looped_from_ (去重表) ----
    //   之前是在 runOnce() 里就登记的, 一旦后续优化失败, 这条回环就
    //   永远不会被重试 (而且约束已经出队, 彻底丢失).
    //   现在: 只在优化成功后才确认去重登记.
    for (const LoopConstraint& constraint : constraints) {
      if (!constraint.valid) continue;
      if (std::find(looped_from_.begin(), looped_from_.end(),
                    constraint.from_index) == looped_from_.end()) {
        looped_from_.push_back(constraint.from_index);
      }
    }
  }

  // ---- 生成回写修正量 (见 maybeAddKeyframe 里的推导) ----
  if (g_loop_writeback_enable && ref_index_ < optimized.size()) {
    const BASIC::SE3 x_new = optimized[ref_index_];
    const BASIC::SE3 corr = x_new * ref_graph_pose_.inverse();
    if (isCorrectionSignificant(corr)) out_corr = corr;
    ref_graph_pose_ = x_new;
  }

  LOG(INFO) << GREEN << " ---> [LoopClosure]: optimization done, loop count = "
            << loop_count_ << ", residual = " << backend_->currentResidual()
            << ", nodes = " << backend_->nodeCount() << RESET;
  return true;
}

// ============================================================
// 判断修正量是否显著 (避免微小数值抖动导致 ESKF 抖动)
// ============================================================
bool LoopClosureManager::isCorrectionSignificant(const BASIC::SE3& corr) {
  if (corr.t().norm() < kMinCorrectionTrans) {
    const double angle = corr.so3().log_vee().norm();
    if (angle < kMinCorrectionRot) return false;
  }
  return true;
}

void LoopClosureManager::loopThread() {
  const double period = 1.0 / std::max(0.01, g_loop_closure_frequency);
  while (running_.load()) {
    // 分片睡眠, 保证 stop() 能及时响应.
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(
                        static_cast<long>(period * 1000.0));
    while (running_.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!running_.load()) break;

    try {
      runOnce();
    } catch (const std::exception& e) {
      LOG(WARNING) << YELLOW << " ---> [LoopClosure] exception: " << e.what()
                   << RESET;
    }
  }
}

std::vector<BASIC::SE3> LoopClosureManager::optimizedPoses() const {
  return kfm_->getPoses();
}

std::vector<std::pair<std::size_t, std::size_t>>
LoopClosureManager::loopPairs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<std::size_t, std::size_t>> out;
  out.reserve(loop_edges_.size());
  for (const LoopEdge& edge : loop_edges_) {
    out.emplace_back(edge.from_index, edge.to_index);
  }
  return out;
}

std::vector<LoopClosureManager::LoopEdge> LoopClosureManager::loopEdges() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_edges_;
}

std::size_t LoopClosureManager::loopCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_count_;
}

} // namespace LI2Sup
