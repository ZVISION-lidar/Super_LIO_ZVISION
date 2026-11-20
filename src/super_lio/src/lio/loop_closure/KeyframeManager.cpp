#include "lio/loop_closure/KeyframeManager.h"

#include <algorithm>
#include <cmath>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace LI2Sup {
namespace {
/// ICP submap 的降采样体素大小 (与 fastlio_sam 的 downSizeFilterICP 一致).
constexpr double kIcpVoxelLeaf = 0.4;
} // namespace

KeyframeManager::KeyframeManager() {
  configure(Options());
}

KeyframeManager::KeyframeManager(const Options& options) {
  configure(options);
}

void KeyframeManager::configure(const Options& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
  scan_context_.reset(new ScanContextManagerType());
  scan_context_->configure(options_.scan_context);
}

bool KeyframeManager::shouldCreateKeyframe(const BASIC::SE3& curr_pose,
                                           const bool use_graph_pose) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (poses_.empty()) return true;

  // 比较基准:
  //   use_graph_pose = true  -> 用图内估计值 (开启回写, ESKF 已被钉在图上)
  //   use_graph_pose = false -> 用原始位姿   (关闭回写, 前端保持纯里程计)
  const BASIC::SE3& base =
      (use_graph_pose && !poses_.empty()) ? poses_.back() : odom_poses_.back();

  const BASIC::SE3 delta = base.inverse() * curr_pose;
  const double dist = delta.t().norm();
  const double angle = delta.so3().log_vee().norm();

  // 位移和角度必须【同时】超过阈值才建关键帧 (与 fastlio_sam 一致).
  if (dist < options_.keyframe_dist && angle < options_.keyframe_angle) {
    return false;
  }
  return true;
}

std::size_t KeyframeManager::addKeyframe(
    const BASIC::SE3& pose,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_.push_back(pose);
  odom_poses_.push_back(pose);
  // 必须深拷贝: 调用方的点云每帧都会复用.
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy(new pcl::PointCloud<pcl::PointXYZI>());
  if (cloud) {
    copy->reserve(cloud->size());
    copy->points = cloud->points;
    copy->width = copy->points.size();
    copy->height = 1;
    copy->is_dense = cloud->is_dense;
  }
  clouds_.push_back(copy);

  // Scan Context 描述子索引与关键帧索引一一对应.
  scan_context_->addScan(*copy);

  return poses_.size() - 1;
}

void KeyframeManager::updatePoses(const std::vector<BASIC::SE3>& poses) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t n = std::min(poses.size(), poses_.size());
  for (std::size_t i = 0; i < n; ++i) poses_[i] = poses[i];
}

std::vector<BASIC::SE3> KeyframeManager::getPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_;
}

std::vector<BASIC::SE3> KeyframeManager::getOdomPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_poses_;
}

bool KeyframeManager::snapshotClouds(std::size_t index, int search_num,
                                      CloudSnapshot& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  out.clouds.clear();
  out.poses.clear();
  out.center_index = index;

  if (poses_.empty() || index >= poses_.size()) return false;

  out.center_pose = poses_[index];

  const int cloud_size = static_cast<int>(poses_.size());
  for (int i = -search_num; i <= search_num; ++i) {
    const int key_near = static_cast<int>(index) + i;
    if (key_near < 0 || key_near >= cloud_size) continue;
    if (key_near >= static_cast<int>(clouds_.size())) continue;

    const auto& src = clouds_[key_near];
    if (!src || src->empty()) continue;

    pcl::PointCloud<pcl::PointXYZI> copy;
    copy.points = src->points;
    copy.width = copy.points.size();
    copy.height = 1;
    copy.is_dense = src->is_dense;

    out.clouds.push_back(std::move(copy));
    out.poses.push_back(poses_[key_near]);
  }
  return !out.clouds.empty();
}

bool KeyframeManager::getWorldCloud(
    std::size_t index, int search_num,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
    const bool downsample) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out) out.reset(new pcl::PointCloud<pcl::PointXYZI>());
  out->clear();

  if (poses_.empty() || index >= poses_.size()) return false;

  const int cloud_size = static_cast<int>(poses_.size());
  for (int i = -search_num; i <= search_num; ++i) {
    const int key_near = static_cast<int>(index) + i;
    if (key_near < 0 || key_near >= cloud_size) continue;
    if (key_near >= static_cast<int>(clouds_.size())) continue;

    const auto& src = clouds_[key_near];
    if (!src || src->empty()) continue;

    const BASIC::SE3& T_w_b = poses_[key_near];
    const Eigen::Matrix4f T = T_w_b.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZI> transformed;
    pcl::transformPointCloud(*src, transformed, T);

    *out += transformed;
  }

  if (out->empty()) return false;

  if (downsample) {
    // 降采样, 与 fastlio_sam 的 downSizeFilterICP 一致.
    // 注意 PCL 1.10 的 API:
    //   setInputCloud(shared_ptr)  -- 传指针
    //   filter(reference)          -- 传引用
    pcl::PointCloud<pcl::PointXYZI> filtered;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(kIcpVoxelLeaf, kIcpVoxelLeaf, kIcpVoxelLeaf);
    voxel_filter.setInputCloud(out);
    voxel_filter.filter(filtered);
    out.reset(new pcl::PointCloud<pcl::PointXYZI>());
    *out = filtered;
  }

  return !out->empty();
}

bool KeyframeManager::getAllWorldCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out) out.reset(new pcl::PointCloud<pcl::PointXYZI>());
  out->clear();

  const std::size_t n = std::min(poses_.size(), clouds_.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto& src = clouds_[i];
    if (!src || src->empty()) continue;

    const Eigen::Matrix4f T = poses_[i].matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI> transformed;
    pcl::transformPointCloud(*src, transformed, T);
    *out += transformed;
  }
  return !out->empty();
}

bool KeyframeManager::getAllWorldCloudOdom(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& out, double max_range) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out) out.reset(new pcl::PointCloud<pcl::PointXYZI>());
  out->clear();

  const std::size_t n = std::min(odom_poses_.size(), clouds_.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto& src = clouds_[i];
    if (!src || src->empty()) continue;

    const Eigen::Matrix4f T = odom_poses_[i].matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI> transformed;
    pcl::transformPointCloud(*src, transformed, T);
    *out += transformed;
  }

  if (max_range > 0.0) {
    // 裁掉离原点太远的点 (原始轨迹漂移可能很远, 便于观察).
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(out->size());
    for (const auto& pt : out->points) {
      if (pt.getVector3fMap().norm() <= max_range) filtered.push_back(pt);
    }
    out->swap(filtered);
  }
  return !out->empty();
}

bool KeyframeManager::getOdomWorldCloud(
    std::size_t index, pcl::PointCloud<pcl::PointXYZI>::Ptr& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out) out.reset(new pcl::PointCloud<pcl::PointXYZI>());
  out->clear();

  if (index >= clouds_.size() || index >= odom_poses_.size()) return false;
  const auto& src = clouds_[index];
  if (!src || src->empty()) return false;

  const Eigen::Matrix4f T = odom_poses_[index].matrix().cast<float>();
  pcl::transformPointCloud(*src, *out, T);
  return !out->empty();
}

ScanContextCandidate KeyframeManager::detectLoopCandidate() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return scan_context_->detectLatest();
}

std::size_t KeyframeManager::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_.size();
}

} // namespace LI2Sup
