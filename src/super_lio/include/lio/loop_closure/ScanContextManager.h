#pragma once

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace LI2Sup {

/// Scan Context: G. Kim and A. Kim, IROS 2018.
/// 描述子构建 + 候选检索. 几何验证 (ICP) 由后端负责.
///
/// Ported from fastlio_sam_sc (src/FAST_LIO_SAM/include/scancontext/ScanContextManager.h).
/// 差异: 点云类型由 pcl::PointXYZINormal 改为 pcl::PointXYZI (super_lio 的 PointType).
///       描述子只用 x/y/z, 因此无信息损失.
struct ScanContextCandidate {
  bool valid = false;
  std::size_t query_index = 0;
  std::size_t match_index = 0;
  double distance = std::numeric_limits<double>::infinity();
  double yaw_difference_rad = 0.0;
};

class ScanContextManager {
public:
  struct Config {
    int num_rings = 20;
    int num_sectors = 60;
    double max_radius = 80.0;
    double lidar_height = 2.0;
    int exclude_recent = 30;
    int num_candidates = 10;
    double search_ratio = 0.1;
    double distance_threshold = 0.15;
  };

  ScanContextManager();

  void configure(const Config& config);
  std::size_t addScan(const pcl::PointCloud<pcl::PointXYZI>& scan);
  ScanContextCandidate detectLatest();
  std::size_t size() const { return descriptors_.size(); }

  void reset() {
    descriptors_.clear();
    ring_keys_.clear();
    last_query_index_ = std::numeric_limits<std::size_t>::max();
  }

private:
  Eigen::MatrixXd makeDescriptor(const pcl::PointCloud<pcl::PointXYZI>& scan) const;
  Eigen::VectorXd makeRingKey(const Eigen::MatrixXd& descriptor) const;
  Eigen::VectorXd makeSectorKey(const Eigen::MatrixXd& descriptor) const;
  int findInitialYawShift(const Eigen::VectorXd& query_key,
                          const Eigen::VectorXd& candidate_key) const;
  double descriptorDistance(const Eigen::MatrixXd& query,
                            const Eigen::MatrixXd& candidate,
                            int shift) const;
  std::pair<double, int> alignedDistance(const Eigen::MatrixXd& query,
                                         const Eigen::MatrixXd& candidate) const;

  Config config_;
  std::vector<Eigen::MatrixXd> descriptors_;
  std::vector<Eigen::VectorXd> ring_keys_;
  std::size_t last_query_index_;
};

} // namespace LI2Sup
