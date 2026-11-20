

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "ros/ROSWrapper.h"
#include "lio/loop_closure/LoopClosureManager.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO(){
    // [LOOP-CLOSURE] 停止回环线程.
    if(loop_closure_) loop_closure_->stop();
  };

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void ProcessCaceMap();

  // ============================================================
  // [LOOP-CLOSURE] 回环检测 + 位姿图优化
  // ============================================================
  /// 记录关键帧 (主线程).
  void updateKeyframe();
  /// 把位姿图优化结果回写到 ESKF.
  void applyCorrection(const BASIC::SE3& T_corr);
  /// 消费回环约束 -> 位姿图优化 -> 回写 ESKF (主线程).
  void applyLoopWriteback();
  /// 回环可视化发布.
  void publishLoopVisualization();
  /// [LOOP-CLOSURE] 用关键帧重拼接保存地图 (与优化后位姿对齐).
  bool saveMapByKeyframe(const std::string& file_name);

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  alignas(64) bool effect_mask_[50000] = {false};
  alignas(64) bool effect_knn_mask_[50000] = {false};
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

  // ============================================================
  // [LOOP-CLOSURE] 回环检测 + 位姿图优化
  // ============================================================
  LoopClosureManager::Ptr loop_closure_;
  /// 回环可视化限频计数器
  int loop_vis_counter_ = 0;
  /// 优化后全局地图的发布间隔计数器
  int loop_map_counter_ = 0;
  /// 上一次发布 ICP 调试点云的序号 (只在【新回环】时才发)
  std::size_t last_icp_viz_seq_ = 0;

  Timer time_record_;
};

} // namespace END.

#endif


