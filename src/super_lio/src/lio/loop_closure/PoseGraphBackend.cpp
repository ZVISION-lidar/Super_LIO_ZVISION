#include "lio/loop_closure/PoseGraphBackend.h"

#include <algorithm>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "basic/logs.h"

namespace LI2Sup {

namespace {
/// 构造对角噪声模型 (Rot3 三个在前, Point3 三个在后).
gtsam::noiseModel::Diagonal::shared_ptr makeNoise(double rot_sigma,
                                                  double trans_sigma) {
  gtsam::Vector6 v;
  v << rot_sigma * rot_sigma, rot_sigma * rot_sigma, rot_sigma * rot_sigma,
       trans_sigma * trans_sigma, trans_sigma * trans_sigma,
       trans_sigma * trans_sigma;
  return gtsam::noiseModel::Diagonal::Variances(v);
}
} // namespace

PoseGraphBackend::PoseGraphBackend() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam_.reset(new gtsam::ISAM2(params));
}

PoseGraphBackend::~PoseGraphBackend() = default;

void PoseGraphBackend::configure(const Options& options) {
  options_ = options;
}

gtsam::Pose3 PoseGraphBackend::toGtsam(const BASIC::SE3& pose) {
  // BASIC::M3 (float) -> double 旋转矩阵 -> gtsam::Rot3
  //
  // [BUG FIX] 必须先做正交化再交给 GTSAM.
  //
  //   原因: GTSAM 4.2 编译时若定义了 GTSAM_USE_QUATERNIONS,
  //         Rot3(Matrix3) 走 Eigen 的 "矩阵 -> 四元数" 构造.
  //         Eigen 的这个构造【假设输入矩阵正交, 不做归一化】.
  //         如果传进去的 R 有微小正交性误差 (float 只有 ~7 位有效数字),
  //         得到的四元数就不是单位的.
  //         随后 GTSAM 的 Logmap/Expmap 会产生 NaN,
  //         线性系统求解失败 -> IndeterminantLinearSystemException.
  //
  //   做法: 用四元数中转, 强制归一化后再转回 double 矩阵.
  //         这样无论输入的 float 矩阵正交性如何, GTSAM 拿到的永远是
  //         严格正交的旋转.
  //
  //   fastlio_sam 没有这个问题: 它用 Rot3::RzRyRx(roll, pitch, yaw)
  //   从欧拉角构造, 天生正交.
  Eigen::Quaterniond q(pose.R_.cast<double>());
  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();

  return gtsam::Pose3(gtsam::Rot3(R),
                      gtsam::Point3(static_cast<double>(pose.t_.x()),
                                    static_cast<double>(pose.t_.y()),
                                    static_cast<double>(pose.t_.z())));
}

BASIC::SE3 PoseGraphBackend::fromGtsam(const gtsam::Pose3& pose) {
  BASIC::SE3 out;
  // GTSAM 的结果本来是正交的, 但转成 float 后会有 ~1e-7 的误差.
  // 这里做一次正交化, 保证回写给 ESKF 的姿态不会引入正交性漂移.
  // (ESKF 侧也做了同样的处理, 这里是第二道防线.)
  Eigen::Quaterniond q(pose.rotation().matrix());
  q.normalize();
  out.R_ = q.toRotationMatrix().cast<BASIC::scalar>();
  out.t_ = pose.translation().cast<BASIC::scalar>();
  out.T_ = Eigen::Matrix4f::Identity();
  out.T_.block<3, 3>(0, 0) = out.R_;
  out.T_.block<3, 1>(0, 3) = out.t_;
  return out;
}

void PoseGraphBackend::addPriorFactor(std::size_t id,
                                      const BASIC::SE3& pose) {
  const auto noise =
      makeNoise(options_.prior_rot_sigma, options_.prior_trans_sigma);
  pending_graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      id, toGtsam(pose), noise);
  full_graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      id, toGtsam(pose), noise);
  pending_initial_.insert<gtsam::Pose3>(id, toGtsam(pose));
  ++pending_factors_;
  if (id + 1 > node_count_) node_count_ = id + 1;
}

void PoseGraphBackend::addOdomFactor(std::size_t id,
                                     const BASIC::SE3& pose_prev,
                                     const BASIC::SE3& pose_curr) {
  if (id == 0) return;

  // between(x_prev, x_curr) = x_prev^{-1} * x_curr
  const gtsam::Pose3 T_prev = toGtsam(pose_prev);
  const gtsam::Pose3 T_curr = toGtsam(pose_curr);
  const gtsam::Pose3 T_rel = T_prev.between(T_curr);

  const auto noise =
      makeNoise(options_.odom_rot_sigma, options_.odom_trans_sigma);
  pending_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id - 1, id, T_rel, noise);
  full_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id - 1, id, T_rel, noise);
  pending_initial_.insert<gtsam::Pose3>(id, T_curr);
  ++pending_factors_;
  if (id + 1 > node_count_) node_count_ = id + 1;
}

void PoseGraphBackend::addLoopFactor(std::size_t id_from, std::size_t id_to,
                                     const BASIC::SE3& T_between,
                                     const double rot_sigma,
                                     const double trans_sigma) {
  if (id_from == id_to) return;

  const double rot = (rot_sigma > 0.0) ? rot_sigma : options_.loop_rot_sigma;
  const double trans =
      (trans_sigma > 0.0) ? trans_sigma : options_.loop_trans_sigma;

  const auto noise = makeNoise(rot, trans);
  pending_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id_from, id_to, toGtsam(T_between), noise);
  full_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id_from, id_to, toGtsam(T_between), noise);
  ++pending_factors_;
  ++pending_loop_factors_;
  ++loop_factor_count_;
}
bool PoseGraphBackend::optimize(std::vector<BASIC::SE3>& poses_out) {
  poses_out.clear();

  // 提交新因子.
  //
  // [BUG FIX] 用 try/catch 包住 update().
  //   GTSAM 的 IndeterminantLinearSystemException (以及 CholeskyFailed 等)
  //   是从 update() / calculateBestEstimate() 里抛出的.
  //   之前只有 calculateBestEstimate() 被包了, 于是 update() 一抛异常
  //   就直接 terminate 掉整个节点 (日志里那句
  //   "terminate called after throwing an instance of
  //    gtsam::IndeterminantLinearSystemException" 就是这么来的).
  //
  //   fastlio_sam 也没有这层保护 (它一旦失败就是整个进程死掉),
  //   但对 super_lio 来说前端已经跑起来了, 因一个优化失败而整体退出
  //   代价太大, 所以这里兜住: 优化失败就返回 false, 让调用方跳过本次
  //   (LoopClosureManager 会放弃本批回环约束, 但图和关键帧继续用).
  if (pending_factors_ > 0) {
    try {
      isam_->update(pending_graph_, pending_initial_);

      // [对齐 fastlio_sam] 迭代次数:
      //   fastlio_sam 的 saveKeyFramesAndFactor() 是这样的:
      //       isam->update(gtSAMgraph, initialEstimate);
      //       isam->update();                       // 总是多来一次
      //       if (aLoopIsClosed) { isam->update() x5 }   // 有回环再补 5 次
      //
      //   这里完全照搬:
      //       - 无参数 update() 总是做 (它触发重线性化 + 重新求解);
      //       - 有回环因子时再多做 5 次, 让回环约束充分收敛.
      isam_->update();

      if (pending_loop_factors_ > 0) {
        for (int i = 0; i < 5; ++i) isam_->update();
      }
    } catch (const std::exception& e) {
      // 优化失败.
      //
      // 【重要】此时这些因子其实已经被推进到 ISAM2 的 nonlinearFactors_ 里了
      // (GTSAM 的 update() 是先加因子, 后做消元求解).
      // 所以我们【不能】把 pending_graph_ 重新塞回去让调用方重试,
      // 否则同一对节点之间会有两份相同的 BetweenFactor, 约束权重翻倍.
      //
      // 处理: 清空待提交队列 (因子已经在图里了), 标记本次失败, 返回 false.
      // ISAM2 内部状态可能不完整, 但由于因子结构没变, 后续 update()
      // 仍会继续在现有图上正常工作.
      pending_graph_.resize(0);
      pending_initial_.clear();
      pending_factors_ = 0;
      pending_loop_factors_ = 0;
      last_residual_ = std::numeric_limits<double>::infinity();
      LOG(WARNING) << YELLOW << " ---> [PoseGraph]: ISAM2 update failed: "
                   << e.what() << RESET;
      return false;
    }
    pending_graph_.resize(0);
    pending_initial_.clear();
    pending_factors_ = 0;
    pending_loop_factors_ = 0;
  }

  if (node_count_ == 0) return false;

  gtsam::Values estimate;
  try {
    estimate = isam_->calculateBestEstimate();
  } catch (const std::exception&) {
    return false;
  }
  if (estimate.empty()) return false;

  poses_out.reserve(node_count_);
  for (std::size_t i = 0; i < node_count_; ++i) {
    if (!estimate.exists(i)) {
      poses_out.clear();
      return false;
    }
    poses_out.push_back(fromGtsam(estimate.at<gtsam::Pose3>(i)));
  }
  estimates_ = poses_out;

  // 记录残差用于诊断.
  try {
    last_residual_ = full_graph_.error(estimate);
  } catch (const std::exception&) {
    last_residual_ = std::numeric_limits<double>::infinity();
  }

  return true;
}

bool PoseGraphBackend::getEstimate(std::size_t id, BASIC::SE3& out) const {
  if (id >= estimates_.size()) return false;
  out = estimates_[id];
  return true;
}

void PoseGraphBackend::reset() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam_.reset(new gtsam::ISAM2(params));
  pending_graph_.resize(0);
  full_graph_.resize(0);
  pending_initial_.clear();
  pending_factors_ = 0;
  pending_loop_factors_ = 0;
  last_residual_ = std::numeric_limits<double>::infinity();
  node_count_ = 0;
  loop_factor_count_ = 0;
  estimates_.clear();
}

} // namespace LI2Sup
