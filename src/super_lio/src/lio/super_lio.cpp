
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>


using namespace BASIC;

namespace LI2Sup{

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);

  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }

  // ============================================================
  // [LOOP-CLOSURE] 回环检测 + 位姿图优化
  //   总开关关闭时, 行为与原来完全一致 (不启动线程, 不记录关键帧).
  // ============================================================
  if(g_loop_closure_enable){
    loop_closure_.reset(new LoopClosureManager());
    loop_closure_->configure();
    loop_closure_->start();
    LOG(INFO) << GREEN << " ---> [SuperLIO]: loop closure enabled." << RESET;
  }else{
    LOG(INFO) << GREEN << " ---> [SuperLIO]: loop closure disabled." << RESET;
  }

  points_world_v3_.reserve(21000);
  effect_mask_.resize(20000, false);
  effect_knn_mask_.resize(20000, false);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  // V3 ref_gravity(0, 0, - g_gravity_norm);
  V3 ref_gravity= g_kf_align_gravity ? gravity : V3(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "Undistort");
    time_record_.Evaluate([this]() { DownSample(); }, "DownSample");
    time_record_.Evaluate([this]() { Observe(); }, "Observe");
    time_record_.Evaluate([this]() { UpdateMap(); }, "UpdateMap");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    UpdateMap();
  }
  // [LOOP-CLOSURE] 先消费回环约束并优化, 再记录新关键帧.
  //   顺序很重要: 先让位姿图吸收回环修正, 新关键帧的里程计因子
  //   才能建立在"已修正"的图上.
  applyLoopWriteback();
  updateKeyframe();
  Output();
  caceData();
  publishLoopVisualization();
}


// ============================================================
// [LOOP-CLOSURE] 关键帧记录
//   关键帧点云保存在 IMU 系 (与 kf_->GetSE3() 同系), 位姿是 T_world_body.
//   这样 ICP 与地图重拼接都不需要 lidar<->body 外参转换.
// ============================================================
void SuperLIO::updateKeyframe(){
  if(!loop_closure_) return;
  if(!ds_undistort_ || ds_undistort_->empty()) return;

  // ds_undistort_ 就是 imu 系点云, 直接用.
  BASIC::SE3 corr;
  if(loop_closure_->maybeAddKeyframe(kf_->GetSE3(), ds_undistort_, corr)){
    // 新关键帧建立后, 位姿图可能对当前节点产生了偏移, 需要回写.
    applyCorrection(corr);
  }
}


// ============================================================
// [LOOP-CLOSURE] 把位姿图优化结果回写到 ESKF
//   注意: 必须用 CorrectPose(), 不能用 SetX().
//   SetX() 会改写 last_imu_time_, 破坏 IMU 递推的时间基准.
//
//   施加方式: T_new = T_corr * T_cur  (T_corr 是世界系下的刚体修正)
// ============================================================
void SuperLIO::applyCorrection(const BASIC::SE3& T_corr){
  if(!loop_closure_ || !g_loop_writeback_enable) return;

  const BASIC::SE3 cur_pose = kf_->GetSE3();
  const BASIC::SE3 new_pose = T_corr * cur_pose;

  kf_->CorrectPose(BASIC::SO3(new_pose.R_), new_pose.t_, BASIC::SO3(cur_pose.R_));

  LOG(INFO) << GREEN << " ---> [SuperLIO]: loop writeback applied, "
            << "dp = [" << (new_pose.t_ - cur_pose.t_).x() << ", "
            << (new_pose.t_ - cur_pose.t_).y() << ", "
            << (new_pose.t_ - cur_pose.t_).z() << "]" << RESET;
}


// ============================================================
// [LOOP-CLOSURE] 消费回环约束 -> 位姿图优化 -> 回写 ESKF
//   必须在主线程做 (与 EKF 更新串行), 保证帧一致性.
//
//   [BUG FIX] 这里【不能】用 g_loop_writeback_enable 提前返回.
//     之前写成:
//         if(!loop_closure_ || !g_loop_writeback_enable) return;
//     结果 writeback_enable=false 时 processLoops() 一次都没被调用,
//     addLoopFactor() 一次都没执行 —— 回环约束全部烂在队列里,
//     位姿图里从来就没有过回环因子.
//     (fastlio_sam 没有这个开关: 回环因子必然入图, 回写也必然执行.)
//
//     现在: 位姿图优化【始终】进行 (回环约束必然入图),
//           只有"把结果写回 ESKF"这一步由 writeback_enable 控制.
//     这样 writeback_enable=false 就真正变成了"只优化不回写"的对照实验.
// ============================================================
void SuperLIO::applyLoopWriteback(){
  if(!loop_closure_) return;

  BASIC::SE3 T_corr;
  if(!loop_closure_->processLoops(T_corr)) return;

  // 回写是独立开关: 位姿图照常优化, 只是不一定改 ESKF.
  applyCorrection(T_corr);
}


// ============================================================
// [LOOP-CLOSURE] 回环可视化
//   发布回环边 (LINE_LIST) 和优化后的关键帧轨迹.
// ============================================================
void SuperLIO::publishLoopVisualization(){
  if(!loop_closure_) return;

  // 限频, 避免刷屏.
  if(++loop_vis_counter_ < 10) return;
  loop_vis_counter_ = 0;

  const std::vector<BASIC::SE3> poses = loop_closure_->optimizedPoses();
  const std::vector<LoopClosureManager::LoopEdge> edges =
      loop_closure_->loopEdges();

  std::vector<LoopEdgeInfo> ros_edges;
  ros_edges.reserve(edges.size());
  for(const auto& edge : edges){
    ros_edges.push_back({edge.from_index, edge.to_index, edge.fitness_score});
  }
  data_wrapper_->pub_loop_closure(poses, ros_edges);

  // ---- [可视化] "优化前"的轨迹 (原始 ESKF 位姿) ----
  //   与上面的"优化后"轨迹叠加, 一眼看出回环把漂移拉回了多少.
  const std::vector<BASIC::SE3> odom_poses =
      loop_closure_->keyframeManager()->getOdomPoses();
  if(!odom_poses.empty()){
    data_wrapper_->pub_loop_path_before(odom_poses);
  }

  // ---- [可视化] ICP 匹配的点云 ----
  //   只在真的有新回环匹配时才发 (lastIcpVisualization 是最近一次的).
  const LoopDetector::IcpVisualization icp_viz =
      loop_closure_->lastIcpVisualization();
  if(icp_viz.valid){
    data_wrapper_->pub_loop_icp_clouds(icp_viz.history_cloud,
                                       icp_viz.corrected_cloud,
                                       icp_viz.raw_cloud);
  }

  // ---- [可视化] 优化后的全局地图 (关键帧重拼接) ----
  //   开销大 (要变换所有关键帧点云), 所以低频发.
  //   对应 fastlio_sam 的 map_global_optimized.
  if(++loop_map_counter_ < 50) return;
  loop_map_counter_ = 0;
  BASIC::CloudPtr opt_map(new PointCloudType());
  if(loop_closure_->keyframeManager()->getAllWorldCloud(opt_map) &&
     !opt_map->empty()){
    // 降采样一下再发, 否则 RViz 会卡.
    if(g_if_filter){
      PointCloudType filtered;
      pcl::VoxelGrid<PointType> voxel_filter;
      voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
      voxel_filter.setInputCloud(opt_map);
      voxel_filter.filter(filtered);
      opt_map.reset(new PointCloudType());
      *opt_map = filtered;
    }
    data_wrapper_->pub_loop_map_optimized(opt_map);
  }
}


// ============================================================
// [LOOP-CLOSURE] 关键帧重拼接保存地图
//   用 ISAM2 优化后的关键帧位姿把关键帧点云 (imu 系) 变换到世界系再拼接.
//   这样保存出的地图与优化后的位姿完全一致 (不需要重建在线地图).
// ============================================================
bool SuperLIO::saveMapByKeyframe(const std::string& file_name){
  if(!loop_closure_) return false;

  BASIC::CloudPtr map_cloud(new PointCloudType());
  if(!loop_closure_->keyframeManager()->getAllWorldCloud(map_cloud)) return false;

  // 降采样
  // 注意 PCL 1.10 的 API:
  //   setInputCloud(shared_ptr)  -- 传指针
  //   filter(reference)          -- 传引用 (输出可以是栈对象)
  if(g_if_filter){
    PointCloudType filtered;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_filter.setInputCloud(map_cloud);
    voxel_filter.filter(filtered);
    map_cloud.reset(new PointCloudType());
    *map_cloud = filtered;
  }

  if(map_cloud->empty()) return false;

  map_cloud->width = map_cloud->size();
  map_cloud->height = 1;
  map_cloud->is_dense = false;

  pcl::io::savePCDFileBinary(file_name, *map_cloud);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: keyframe map saved to: " << file_name
            << ", size: " << map_cloud->size() << RESET;
  return true;
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;

  // ============================================================
  // [LOOP-CLOSURE] 开启回环时, 用关键帧重拼接保存地图.
  //   这样保存的地图与 ISAM2 优化后的位姿完全一致,
  //   而在线地图 (OctVoxMap) 保持原样, 不做重建 (与 fastlio_sam 一致).
  // ============================================================
  if(loop_closure_ && g_loop_keyframe_map_enable){
    const std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Saving map by keyframe (aligned with "
              << "pose graph optimization) ..." << RESET;
    if(saveMapByKeyframe(map_name)) return;
    LOG(WARNING) << YELLOW << " ---> Keyframe map saving failed, "
                 << "fallback to the online map ..." << RESET;
  }

  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;
      if (query_time > propagate_states_.back().time) {
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = TLI_R * raw + TLI_t;
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = propagate_states_.begin();
      for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double s = (query_time - match_iter->time) / dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 t_ei(p_h + v_h * dt + 0.5 * acc_t * dt * dt - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  effect_mask_.resize(ptsize, false);
  effect_knn_mask_.resize(ptsize, false);
  abcd_vec_.resize(ptsize);
  effect_knn_idxs_.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
      
            local_acc.HTVH += J * 1000 * J.transpose();
            local_acc.HTVr -= J * 1000 * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  // [ALIGN-FastLIVO2] 前向递推重锚定.
  // EKF 后验 (v_/R_/p_) 已确定, 通知 ESKF 把前向状态重置到后验,
  // 并在下一条 IMU 到来时用 ~5ms 的细粒度步长逐条重积分.
  // (对应 FastLIVO2 的 state_update_flg = true)
  kf_->NotifyForwardAnchor(measures_.lidar.end_time);

  frame_num_++;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state);  

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

SuperLIO::~SuperLIO(){
  // [LOOP-CLOSURE] 停止回环线程.
  if(loop_closure_) loop_closure_->stop();
}

} // namespace END.