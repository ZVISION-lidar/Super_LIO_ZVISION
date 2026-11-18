#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>

#include "basic/Manifold.h"
#include "lio/loop_closure/ScanContextManager.h"

namespace LI2Sup {

using ScanContextManagerType = ScanContextManager;

/// 关键帧管理.
///
/// 职责:
///   1. 关键帧判定 (位移 + 角度阈值)
///   2. 保存关键帧位姿与点云 (imu 系, 与 kf_->GetSE3() 同系)
///   3. 维护 Scan Context 描述子 (索引与关键帧一一对应)
///   4. 提供"按关键帧索引取世界系点云"的接口, 供 ICP 与地图重拼接使用
///
/// ================================================================
/// 两套位姿 (非常重要)
/// ================================================================
///   poses_      : 位姿图 (ISAM2) 的节点估计值 X_k, 会被回环优化不断修改.
///                 用于: ICP 几何验证、地图重拼接、可视化.
///
///   odom_poses_ : 关键帧建立瞬间的【原始 ESKF 位姿】, 建立后永不修改.
///                 用于: 关键帧判定、构造里程计因子.
///
/// 为什么关键帧判定要用哪一套?
///   判定依据必须是"相对最新关键帧的纯相对运动".
///
///   - 开启回写 (writeback_enable = true):
///       ESKF 位姿被钉在图上 (T = X_ref * Δ), 所以必须拿【图内估计值】
///       poses_.back() 作比较, 才能得到纯相对运动.
///       若拿原始位姿比较, 回环修正后的位移会被误判成"走了一大段",
///       导致疯狂建关键帧.
///
///   - 关闭回写 (writeback_enable = false):
///       ESKF 保持原始里程计, 图与前端是两条独立的链.
///       此时必须拿【原始位姿】odom_poses_.back() 作比较,
///       且里程计因子也用原始位姿来构造, 保证图内链条是纯里程计链,
///       回环造成的形变可以自由分配到整条轨迹上.
///
/// 线程安全: 所有公开接口内部加锁 (与外部锁无关, 不会死锁).
class KeyframeManager {
public:
  using Ptr = std::shared_ptr<KeyframeManager>;

  struct Options {
    /// 关键帧位移阈值 (m)
    double keyframe_dist = 1.0;
    /// 关键帧角度阈值 (rad)
    double keyframe_angle = 0.2;
    /// Scan Context 参数
    ScanContextManager::Config scan_context;
  };

  /// 一次"点云快照": 拷贝出来的原始点云 + 对应位姿.
  /// 用于把耗时的变换/降采样/ICP 放到锁外执行, 避免阻塞前端.
  struct CloudSnapshot {
    /// 中心关键帧索引 (发起这次查询的关键帧)
    std::size_t center_index = 0;
    /// 中心关键帧的世界系位姿
    BASIC::SE3 center_pose;
    /// 每个点云都在 imu 系 (未变换), 按 [center-search_num, center+search_num] 排列
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
    /// 与 clouds 一一对应的世界系位姿
    std::vector<BASIC::SE3> poses;
  };

  /// 注意: 默认参数不能写成 `= Options()`.
  /// GCC 会报 "default member initializer required before the end of its
  /// enclosing class" (嵌套类的 NSDMI 在类体内还没初始化完就被使用了).
  /// 所以默认构造在 .cpp 里用 KeyframeManager::Options() 实现.
  KeyframeManager();
  explicit KeyframeManager(const Options& options);
  ~KeyframeManager() = default;

  void configure(const Options& options);
  const Options& options() const { return options_; }

  /// 判断当前位姿是否应该成为新关键帧.
  /// @param curr_pose        当前 ESKF 位姿
  /// @param use_graph_pose   true:  与【图内估计值】比较 (开启回写时用)
  ///                         false: 与【原始位姿】比较   (关闭回写时用)
  bool shouldCreateKeyframe(const BASIC::SE3& curr_pose,
                            bool use_graph_pose = true) const;

  /// 保存关键帧. pose 与 cloud 必须是 imu 系.
  /// 返回该关键帧的索引.
  std::size_t addKeyframe(const BASIC::SE3& pose,
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  /// 批量更新关键帧位姿 (位姿图优化后回写). 只影响 poses_.
  void updatePoses(const std::vector<BASIC::SE3>& poses);

  /// 取关键帧位姿 (位姿图当前估计值).
  std::vector<BASIC::SE3> getPoses() const;

  /// 取关键帧的原始位姿 (建立后固定不变).
  std::vector<BASIC::SE3> getOdomPoses() const;

  /// 取指定关键帧索引处的世界系点云 (用位姿图估计值变换到世界系).
  /// @param index       中心关键帧索引
  /// @param search_num  前后各取多少个关键帧拼成 submap (0 = 只取自身)
  /// @param out         输出 (世界系)
  /// @param downsample  是否按 ICP 体素尺寸降采样
  bool getWorldCloud(std::size_t index, int search_num,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                     bool downsample = true) const;

  /// 取所有关键帧拼成的世界系点云 (用位姿图估计值, 不降采样).
  bool getAllWorldCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& out) const;

  /// 拷贝出某个关键帧 (及邻域) 的原始点云与位姿.
  /// 锁内只做内存拷贝, 后续的变换/降采样/ICP 在锁外做.
  bool snapshotClouds(std::size_t index, int search_num,
                      CloudSnapshot& out) const;

  /// Scan Context 候选检索 (线程安全).
  ScanContextCandidate detectLoopCandidate() const;

  std::size_t size() const;

private:
  mutable std::mutex mutex_;
  Options options_;

  /// 位姿图 (ISAM2) 的节点估计值.
  std::vector<BASIC::SE3> poses_;
  /// 关键帧建立瞬间的原始 ESKF 位姿 (永不修改).
  std::vector<BASIC::SE3> odom_poses_;
  /// 关键帧点云 (imu 系), 与 poses_ 一一对应.
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds_;
  /// Scan Context 描述子, 与 poses_ 一一对应.
  std::unique_ptr<ScanContextManagerType> scan_context_;
};

} // namespace LI2Sup
