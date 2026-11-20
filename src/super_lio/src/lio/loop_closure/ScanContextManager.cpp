#include "lio/loop_closure/ScanContextManager.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace LI2Sup {
namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

ScanContextManager::ScanContextManager()
    : last_query_index_(std::numeric_limits<std::size_t>::max()) {}

void ScanContextManager::configure(const Config& config) {
  if (config.num_rings <= 0 || config.num_sectors <= 0 ||
      config.max_radius <= 0.0 || config.exclude_recent < 1 ||
      config.num_candidates < 1 || config.search_ratio < 0.0 ||
      config.search_ratio > 1.0 || config.distance_threshold <= 0.0) {
    throw std::invalid_argument("invalid Scan Context configuration");
  }

  config_ = config;
  descriptors_.clear();
  ring_keys_.clear();
  last_query_index_ = std::numeric_limits<std::size_t>::max();
}

std::size_t ScanContextManager::addScan(
    const pcl::PointCloud<pcl::PointXYZI>& scan) {
  Eigen::MatrixXd descriptor = makeDescriptor(scan);
  descriptors_.push_back(descriptor);
  ring_keys_.push_back(makeRingKey(descriptor));
  return descriptors_.size() - 1;
}

Eigen::MatrixXd ScanContextManager::makeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>& scan) const {
  Eigen::MatrixXd descriptor = Eigen::MatrixXd::Constant(
      config_.num_rings, config_.num_sectors,
      -std::numeric_limits<double>::infinity());

  for (const pcl::PointXYZI& point : scan.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      continue;

    const double radius = std::hypot(point.x, point.y);
    if (radius > config_.max_radius) continue;

    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) angle += 2.0 * kPi;

    const int ring = std::max(0, std::min(
        config_.num_rings - 1,
        static_cast<int>(std::ceil(
            radius / config_.max_radius * config_.num_rings)) - 1));
    const int sector = std::max(0, std::min(
        config_.num_sectors - 1,
        static_cast<int>(std::ceil(
            angle / (2.0 * kPi) * config_.num_sectors)) - 1));

    // 高度取该 (ring, sector) 内的最大值, 并加上雷达安装高度补偿.
    const double height = static_cast<double>(point.z) + config_.lidar_height;
    descriptor(ring, sector) = std::max(descriptor(ring, sector), height);
  }

  // 未被点填充的格子置 0 (而非 -inf), 使后续余弦相似度计算有定义.
  for (Eigen::Index row = 0; row < descriptor.rows(); ++row) {
    for (Eigen::Index col = 0; col < descriptor.cols(); ++col) {
      if (!std::isfinite(descriptor(row, col))) descriptor(row, col) = 0.0;
    }
  }
  return descriptor;
}

Eigen::VectorXd ScanContextManager::makeRingKey(
    const Eigen::MatrixXd& descriptor) const {
  return descriptor.rowwise().mean();
}

Eigen::VectorXd ScanContextManager::makeSectorKey(
    const Eigen::MatrixXd& descriptor) const {
  return descriptor.colwise().mean().transpose();
}

int ScanContextManager::findInitialYawShift(
    const Eigen::VectorXd& query_key,
    const Eigen::VectorXd& candidate_key) const {
  int best_shift = 0;
  double best_norm = std::numeric_limits<double>::infinity();
  const int sectors = config_.num_sectors;
  for (int shift = 0; shift < sectors; ++shift) {
    double squared_norm = 0.0;
    for (int col = 0; col < sectors; ++col) {
      const int candidate_col = (col - shift + sectors) % sectors;
      const double difference = query_key(col) - candidate_key(candidate_col);
      squared_norm += difference * difference;
    }
    if (squared_norm < best_norm) {
      best_norm = squared_norm;
      best_shift = shift;
    }
  }
  return best_shift;
}

double ScanContextManager::descriptorDistance(
    const Eigen::MatrixXd& query,
    const Eigen::MatrixXd& candidate,
    const int shift) const {
  double similarity_sum = 0.0;
  int effective_sectors = 0;
  const int sectors = config_.num_sectors;
  for (int col = 0; col < sectors; ++col) {
    const int candidate_col = (col - shift + sectors) % sectors;
    const Eigen::VectorXd query_sector = query.col(col);
    const Eigen::VectorXd candidate_sector = candidate.col(candidate_col);
    const double query_norm = query_sector.norm();
    const double candidate_norm = candidate_sector.norm();
    // 全零的列没有信息, 跳过 (不参与平均).
    if (query_norm <= 1e-12 || candidate_norm <= 1e-12) continue;

    similarity_sum += query_sector.dot(candidate_sector) /
                      (query_norm * candidate_norm);
    ++effective_sectors;
  }

  if (effective_sectors == 0) return 1.0;
  return 1.0 - similarity_sum / effective_sectors;
}

std::pair<double, int> ScanContextManager::alignedDistance(
    const Eigen::MatrixXd& query,
    const Eigen::MatrixXd& candidate) const {
  const int initial_shift = findInitialYawShift(
      makeSectorKey(query), makeSectorKey(candidate));
  const int search_radius = static_cast<int>(std::round(
      0.5 * config_.search_ratio * config_.num_sectors));

  double best_distance = std::numeric_limits<double>::infinity();
  int best_shift = initial_shift;
  for (int offset = -search_radius; offset <= search_radius; ++offset) {
    const int shift = (initial_shift + offset + config_.num_sectors) %
                      config_.num_sectors;
    const double distance = descriptorDistance(query, candidate, shift);
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }
  return std::make_pair(best_distance, best_shift);
}

ScanContextCandidate ScanContextManager::detectLatest() {
  ScanContextCandidate result;
  if (descriptors_.empty()) return result;

  // 只查最新加入的描述子. 同一关键帧不重复查询.
  const std::size_t query_index = descriptors_.size() - 1;
  if (last_query_index_ == query_index) return result;
  last_query_index_ = query_index;
  result.query_index = query_index;

  // 排除最近 N 个关键帧, 避免近距误检.
  if (descriptors_.size() <= static_cast<std::size_t>(config_.exclude_recent))
    return result;

  const std::size_t candidate_end =
      descriptors_.size() - static_cast<std::size_t>(config_.exclude_recent);
  std::vector<std::pair<double, std::size_t>> ring_key_distances;
  ring_key_distances.reserve(candidate_end);
  for (std::size_t index = 0; index < candidate_end; ++index) {
    const double distance =
        (ring_keys_[query_index] - ring_keys_[index]).squaredNorm();
    ring_key_distances.emplace_back(distance, index);
  }

  const std::size_t candidate_count = std::min(
      static_cast<std::size_t>(config_.num_candidates),
      ring_key_distances.size());
  std::partial_sort(ring_key_distances.begin(),
                    ring_key_distances.begin() + candidate_count,
                    ring_key_distances.end());

  int best_shift = 0;
  for (std::size_t rank = 0; rank < candidate_count; ++rank) {
    const std::size_t candidate_index = ring_key_distances[rank].second;
    const std::pair<double, int> aligned = alignedDistance(
        descriptors_[query_index], descriptors_[candidate_index]);
    if (aligned.first < result.distance) {
      result.distance = aligned.first;
      result.match_index = candidate_index;
      best_shift = aligned.second;
    }
  }

  result.yaw_difference_rad =
      best_shift * (2.0 * kPi / static_cast<double>(config_.num_sectors));
  result.valid = result.distance < config_.distance_threshold;
  return result;
}

} // namespace LI2Sup
