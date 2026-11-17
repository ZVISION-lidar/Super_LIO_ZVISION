#ifndef ESKF_HPP_
#define ESKF_HPP_

#include <deque>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "params.h"


namespace LI2Sup{

class ESKF {
public:
  using Ptr   = std::shared_ptr<ESKF>;
                                                  //         0 3 6 9 12 15
  using STATE = Eigen::Matrix<BASIC::scalar, 18, 1>;     //Flatten: R p v bg ba g. 
  using STATE_DOF = Eigen::Matrix<BASIC::scalar, 17, 1>; //Flatten: R p v bg ba g_2.
  using NOISE = Eigen::Matrix<BASIC::scalar, 12, 12>;
  using COV   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_X   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_W   = Eigen::Matrix<BASIC::scalar, 18, 12>;


  struct Options {
    Options(){}
    int num_iterations_ = 3;
    double quit_eps_ = 1e-6;

    double gyro_var_ = 1e-5;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-6;
    double bias_acce_var_ = 1e-4;
  };


  struct KFState{
    bool need_converge = true;
    BASIC::SE3  pose;
  };


  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  ESKF(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8))
      : options_(options) {
    BuildNoise(options);
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
  }

  void SetInitialConditions(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const float imu_scale = 1.0,
                            const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8));

  bool Predict(const IMUData& imu);

  using ObsFunc = std::function<void(const KFState& kf_state, BASIC::M6& HT_Vinv_H, BASIC::V6& HT_Vinv_r)>;
  bool UpdateObserve(ObsFunc obs);

  double GetTime() const { return current_time_; }

  SysState GetSysState() const { return SysState(current_time_, R_, p_, v_, bg_, ba_); }

  NavState GetNavState() const { return NavState(current_time_, R_, p_, v_); }

  DynamicState GetDynamicState() const { return DynamicState(current_time_, R_.R_, p_, v_, body_omega_, global_acc_); }

  KFState GetKFState() const { return KFState{need_converge_, GetSE3()}; }

  Pose_t   GetPoseT() const { return Pose_t(current_time_, R_, p_); }

  COV GetCov() const { return P_; }

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R_, p_); }

  void SetObsTime(const double obs_time) { current_obs_time_ = obs_time; }
  void SetLastObsTime(const double obs_time) { last_obs_time_ = obs_time; }

  void SetX(const SysState& x);

  void SetCov(const COV& cov){ P_ = cov; }

  BASIC::V3 GetGravity() const { return g_; }

  bool init_ = false;
  bool Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot);

  /// 前向 (纯 IMU) 递推的锚点通知.
  /// 在 EKF 更新完成 (后验速度 v_ 已确定) 后调用:
  ///   - 把前向状态重置到当前后验 (fw_R_/fw_p_/fw_v_)
  ///   - 丢弃缓冲中早于锚点时刻的 IMU
  ///   - 置位 fw_state_update_flg_, 由下一次 ForwardPredict() 触发"逐条重积分"
  /// 这样锚点之后的速度是用 ~5ms 的细粒度步长从锚点重放出来的,
  /// 而不是用单条 acc 外推一大段 dt (那样每个 LIO 帧都会向速度注入一个尖峰).
  void NotifyForwardAnchor(double anchor_time);

  /// 前向递推 (纯 IMU 推算) 的单步推进.
  /// 与 Predict(imu, state_imu, state_robot) 等价, 但改为:
  ///   1) 维护独立的递推 IMU 缓冲, 锚点后逐条重积分;
  ///   2) 姿态采用半隐式 (先用新姿态转加速度), 与 FastLIVO2 的 prop_imu_once 一致.
  bool ForwardPredict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot);

private:
  void BuildNoise(const Options& options);
  void Update();

  /// 前向递推的单步积分 (半隐式, 与 prop_imu_once 同构).
  /// dt  : 本步的时间长度
  /// acc : 本步使用的加速度 (body 系, 已缩放/去偏置前)
  /// gyr : 本步使用的角速度 (body 系, 已去偏置前)
  void ForwardPropagateOnce(double dt, const BASIC::V3& acc, const BASIC::V3& gyr);
  
  bool  need_converge_  = true;
  float imu_scale_ = 1.0;
  IMUData last_imu_;
  double current_time_ = 0.0;
  double last_imu_time_ = -1.0;
  double last_obs_time_ = 0.0;
  double current_obs_time_ = 0.0;

  // nominal state
  BASIC::SO3 R_;
  BASIC::V3 p_ = BASIC::V3::Zero();
  BASIC::V3 v_ = BASIC::V3::Zero();
  BASIC::V3 bg_ = BASIC::V3::Zero();
  BASIC::V3 ba_ = BASIC::V3::Zero();
  BASIC::V3 g_{0, 0, - (BASIC::scalar)g_gravity_norm};
  BASIC::V3 global_acc_  = BASIC::V3::Zero();
  BASIC::V3 body_omega_  = BASIC::V3::Zero();

  STATE dx_ = STATE::Zero();

  COV P_ = COV::Identity();

  NOISE Q_ = NOISE::Zero();

  Options options_;

  //  前向递推状态 (对应 FastLIVO2 的 imu_propagate).
  BASIC::SO3 fw_R_;
  BASIC::V3 fw_p_ = BASIC::V3::Zero();
  BASIC::V3 fw_v_ = BASIC::V3::Zero();

  // ============================================================
  //  前向递推专用状态
  //   对应 FastLIVO2 的 prop_imu_buffer / state_update_flg / latest_ekf_state.
  //   注意: 不能复用 ROSWrapper 的 imu_buffer_, 那个 buffer 会被
  //   sync_measure() 抽干并交给 EKF 去畸变, 前向递推需要保留一份副本.
  // ============================================================
  /// 递推专用 IMU 缓冲 (锚点后从这里逐条重放).
  std::deque<IMUData> fw_imu_buffer_;
  /// 锚点时刻 (EKF 后验对应的时刻, 即 current_obs_time_).
  double fw_anchor_time_ = -1.0;
  /// 锚点后是否需要"逐条重积分".
  bool fw_state_update_flg_ = false;
  /// 上一次积分到的时刻 (绝对时间).
  double fw_last_integr_time_ = -1.0;
  /// 上一次积分使用的 IMU (用于中点平均).
  IMUData fw_last_imu_;
  /// 递推是否已初始化 (第一帧锚点之后才允许积分).
  bool fw_inited_ = false;
};


}


#endif