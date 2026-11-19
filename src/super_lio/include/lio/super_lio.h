

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <memory>
#include <mutex>

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
  virtual ~SuperLIO();;

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
  /// 主线程: 关键帧判定 + 记录关键帧.
  void updateKeyframe();
  /// 主线程: 把位姿图优化结果回写到 ESKF.
  void applyLoopWriteback();
  /// 主线程: 把一个刚体修正量施加到 ESKF 位姿/速度.
  void applyCorrection(const BASIC::SE3& T_corr);
  /// 发布回环边 + 优化后的轨迹 (可视化).
  void publishLoopVisualization();
  /// 用关键帧重拼接生成对齐后的地图.
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

  /// [LOOP-CLOSURE] 回环管理器
  LoopClosureManager::Ptr loop_closure_;
  /// [LOOP-CLOSURE] 可视化发布节流计数
  int loop_vis_counter_ = 0;
  /// [LOOP-CLOSURE] 优化后地图发布节流计数 (拼接开销大, 低频发)
  int loop_map_counter_ = 0;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  std::vector<bool> effect_mask_;
  std::vector<bool> effect_knn_mask_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

  Timer time_record_;
};

} // namespace END.

#endif

