#include "lio/loop_closure/LoopDetector.h"

#include <algorithm>
#include <cmath>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include "basic/logs.h"

namespace LI2Sup {
namespace {
/// ICP submap 的降采样体素大小 (与 fastlio_sam 的 downSizeFilterICP 一致).
constexpr double kIcpVoxelLeaf = 0.4;
} // namespace

bool LoopDetector::detect(const KeyframeManager& kfm, LoopConstraint& out) {
  out = LoopConstraint();

  // ---- 1. Scan Context 候选检索 ----
  const ScanContextCandidate candidate = kfm.detectLoopCandidate();
  if (!candidate.valid) return false;

  const std::size_t from_index = candidate.query_index;
  const std::size_t to_index = candidate.match_index;
  if (from_index == to_index) return false;

  // ---- 2. 取当前关键帧点云 (世界系, 单帧) ----
  KeyframeManager::CloudSnapshot cur_snapshot;
  if (!kfm.snapshotClouds(from_index, 0, cur_snapshot)) return false;
  if (cur_snapshot.clouds.empty() || cur_snapshot.poses.empty()) return false;

  // ---- 3. 取历史 submap (世界系, 前后若干帧) ----
  KeyframeManager::CloudSnapshot hist_snapshot;
  if (!kfm.snapshotClouds(to_index, options_.history_search_num, hist_snapshot))
    return false;
  if (hist_snapshot.clouds.empty() || hist_snapshot.poses.empty()) return false;

  // 当前关键帧的世界系位姿 (ICP 与约束导出都要用, 必须取同一份快照).
  const BASIC::SE3 pose_from = cur_snapshot.center_pose;
  // 历史关键帧的世界系位姿 (submap 中心那一帧).
  const BASIC::SE3 pose_to = hist_snapshot.center_pose;

  // ---- 4. 变换到世界系 ----
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr history_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (std::size_t i = 0; i < cur_snapshot.clouds.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZI> transformed;
    pcl::transformPointCloud(cur_snapshot.clouds[i], transformed,
                              cur_snapshot.poses[i].matrix().cast<float>());
    *cur_cloud += transformed;
  }
  for (std::size_t i = 0; i < hist_snapshot.clouds.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZI> transformed;
    pcl::transformPointCloud(hist_snapshot.clouds[i], transformed,
                              hist_snapshot.poses[i].matrix().cast<float>());
    *history_cloud += transformed;
  }

  if (cur_cloud->empty() || history_cloud->empty()) return false;

  // 降采样 (与 fastlio_sam 的 downSizeFilterICP 一致).
  // 注意 PCL 1.10 的 API:
  //   setInputCloud(shared_ptr)  -- 传指针
  //   filter(reference)          -- 传引用
  {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(kIcpVoxelLeaf, kIcpVoxelLeaf, kIcpVoxelLeaf);

    pcl::PointCloud<pcl::PointXYZI> cur_filtered;
    voxel_filter.setInputCloud(cur_cloud);
    voxel_filter.filter(cur_filtered);
    cur_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    *cur_cloud = cur_filtered;

    pcl::PointCloud<pcl::PointXYZI> hist_filtered;
    voxel_filter.setInputCloud(history_cloud);
    voxel_filter.filter(hist_filtered);
    history_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    *history_cloud = hist_filtered;
  }

  if (cur_cloud->size() < 100 || history_cloud->size() < 100) return false;

  // ---- 5. ICP 几何验证 ----
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(
      static_cast<double>(options_.icp_max_correspondence_distance));
  icp.setMaximumIterations(options_.icp_max_iterations);
  icp.setTransformationEpsilon(options_.icp_transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(options_.icp_euclidean_fitness_epsilon);
  icp.setRANSACIterations(0);

  icp.setInputSource(cur_cloud);
  icp.setInputTarget(history_cloud);

  pcl::PointCloud<pcl::PointXYZI> aligned;
  icp.align(aligned);

  if (!icp.hasConverged()) return false;
  const double fitness = icp.getFitnessScore();
  if (fitness > options_.icp_fitness_score_threshold) return false;

  // ---- 6. 导出相对位姿约束 ----
  // ICP 的 source/target 都在世界系:
  //   source = T_from * p_body,  target ≈ T_to * p_body
  // PCL 的 getFinalTransformation() 给出 source -> target 的刚体变换 X,
  // 因此 X * T_from ≈ T_to, 即"修正后的当前关键帧位姿" = X * T_from.
  //
  // 回环约束的语义: 修正后的当前关键帧 与 历史关键帧 应该在同一处,
  // 所以测量值 z = between(x_from, x_to) = x_from^{-1} * x_to
  //   其中 x_from 用修正后的位姿 (X * T_from), x_to 用历史关键帧位姿.
  // (与 fastlio_sam 的 poseFrom.between(poseTo) 完全一致)
  const Eigen::Matrix4f T_icp = icp.getFinalTransformation();
  const BASIC::SE3 T_corr(BASIC::M3(T_icp.block<3, 3>(0, 0)),
                          BASIC::V3(T_icp.block<3, 1>(0, 3)));

  const BASIC::SE3 pose_from_corrected = T_corr * pose_from;
  const BASIC::SE3 T_from_to = pose_from_corrected.inverse() * pose_to;

  out.valid = true;
  out.from_index = from_index;
  out.to_index = to_index;
  out.T_from_to = T_from_to;
  out.fitness_score = fitness;
  out.yaw_difference_rad = candidate.yaw_difference_rad;

  accepted_loops_.emplace_back(from_index, to_index);

  return true;
}

} // namespace LI2Sup
