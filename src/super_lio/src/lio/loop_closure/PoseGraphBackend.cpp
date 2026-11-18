#include "lio/loop_closure/PoseGraphBackend.h"

#include <algorithm>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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
  const Eigen::Matrix3d R = pose.R_.cast<double>();
  return gtsam::Pose3(gtsam::Rot3(R),
                      gtsam::Point3(static_cast<double>(pose.t_.x()),
                                    static_cast<double>(pose.t_.y()),
                                    static_cast<double>(pose.t_.z())));
}

BASIC::SE3 PoseGraphBackend::fromGtsam(const gtsam::Pose3& pose) {
  BASIC::SE3 out;
  out.R_ = pose.rotation().matrix().cast<BASIC::scalar>();
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
  if (pending_factors_ > 0) {
    isam_->update(pending_graph_, pending_initial_);
    pending_graph_.resize(0);
    pending_initial_.clear();
    pending_factors_ = 0;

    // 回环约束需要更多迭代才能充分收敛 (与 fastlio_sam 一致).
    // 单纯的里程计因子增量很小, 一次 update 就够了.
    const int extra_iterations = (pending_loop_factors_ > 0) ? 3 : 0;
    for (int i = 0; i < extra_iterations; ++i) isam_->update();
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
