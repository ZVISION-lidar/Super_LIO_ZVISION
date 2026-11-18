#include "lio/ESKF.h"

using namespace BASIC;

namespace LI2Sup{


/// [1] SEfB: P195：（7.37a） P203: (7.76a) (7.77a)  P220: Table 7-2 
/// [2] VSLAM14: P71: (4.26)  P73: (4.32)
/// [3] Proportional Derivative (PD) Control on the Euclidean Group.  A_matrix
/// [1] = [2] <===> [3]
static inline M3d RightJacobianSO3(const V3& ang_vel, const scalar& dt){
  V3d ang_vel_d = ang_vel.template cast<double>();
  scalar ang_vel_norm = ang_vel_d.norm();
  if (ang_vel_norm < 1e-8){   /// noise? too small
    return M3d::Identity();
  }else{
    V3d r_axis = ang_vel_d / ang_vel_norm;
    double r_ang = ang_vel_norm * dt;
    M3d K;
    K << 0.0, -r_axis[2], r_axis[1], r_axis[2], 0.0, -r_axis[0], -r_axis[1], r_axis[0], 0.0;
    double a = (1 - std::cos(r_ang)) / r_ang;
    double b = 1 - std::sin(r_ang) / r_ang;
    return M3d::Identity() - a * K + b * K * K;   // J_r(phi) = J_l(-phi)
  }
}


/// left jacobian
M3d A_matrix(const V3 & v){
  M3d res;
  double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < 1e-6){
		res = M3d::Identity();
	}
	else{
    M3d K;
    K << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
		res = M3d::Identity() + (1 - std::cos(norm)) / squaredNorm * K 
        + (1 - std::sin(norm) / norm) / squaredNorm * K * K;
	}
  return res;
}


void ESKF::SetInitialConditions(Options options, const V3& init_bg, 
                                const V3& init_ba, const float imu_scale,
                                const V3& gravity) 
{
  BuildNoise(options);
  options_ = options;
  bg_ = init_bg;
  ba_ = init_ba;
  g_ = gravity;
  imu_scale_ = imu_scale;

  P_ = 1e-4 * M18::Identity();
  P_.template block<3, 3>(0, 0) = 0.1 * M_PI / 180.0 * M3::Identity();   // r

}


void ESKF::SetX(const SysState& x) {
  last_imu_time_ = x.timestamp;      // TODO: The timestamp update is not strictly consistent.
  current_time_ = last_imu_time_;
  R_ = x.R;
  p_ = x.p;
  v_ = x.v;
  bg_ = x.bg;
  ba_ = x.ba;
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
}


// ============================================================
// [LOOP-CLOSURE] 位姿图优化结果回写
//   只改位姿/速度, 不动 IMU 时间基准.
//   见头文件注释: 为什么不能用 SetX().
//
//   坐标约定:
//     R_ : body(imu) -> world 的旋转
//     p_ : world 系位置
//     v_ : world 系速度 (ForwardPropagateOnce 里 global_acc 累加的是 world 系)
// ============================================================
void ESKF::CorrectPose(const SO3& R_new, const V3& p_new, const SO3& R_old)
{
  // 修正量是 world 系的左乘修正:  T_new = T_corr * T_old
  //   => R_corr = R_new * R_old^{-1}
  // world 系的速度也要跟着这个 world 系旋转一起转:
  //   v_new = R_corr * v_old
  // (等价地, body 系的速度保持不变)
  const M3 R_corr = R_new.R() * R_old.R().transpose();
  const V3 v_new = R_corr * v_;

  R_ = R_new;
  p_ = p_new;
  v_ = v_new;

  // 前向递推状态同步 (不重置 fw_last_integr_time_, 保持时间连续).
  fw_R_ = R_new;
  fw_p_ = p_new;
  fw_v_ = v_new;

  // 协方差不变: 优化只修正均值, 不改变不确定度.
}


void ESKF::BuildNoise(const Options& options) {
  double et = options.gyro_var_;
  double ev = options.acce_var_;
  double eg = options.bias_gyro_var_;
  double ea = options.bias_acce_var_;

  double et2 = et;  // * et;
  double ev2 = ev;  // * ev;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // set Q
  Q_.diagonal() << et2, et2, et2,    // ng: r
                   ev2, ev2, ev2,    // na: v  -> p
                   eg2, eg2, eg2,    // nbg: bg -> w -> r
                   ea2, ea2, ea2;    // nba: ba -> a -> v -> p
}

void ESKF::Update() {
  R_ = R_ * SO3::Exp(dx_.template block<3, 1>(0, 0));
  p_ += dx_.template block<3, 1>(3, 0);
  v_ += dx_.template block<3, 1>(6, 0);

  bg_ += dx_.template block<3, 1>(9, 0);
  ba_ += dx_.template block<3, 1>(12, 0);

  g_ += dx_.template block<3, 1>(15, 0);
  g_ = g_gravity_norm * (g_.normalized());

  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
  // [ALIGN-FastLIVO2] 前向递推的重锚定不在这里做, 而是由
  // NotifyForwardAnchor() 统一处理 (见该函数注释).
  // 原因: Update() 会被 UpdateObserve() 的迭代循环多次调用,
  // 而"丢弃缓冲 + 逐条重积分"只需要做一次, 且必须在 buffer 完整时做.
}


// ============================================================
// [ALIGN-FastLIVO2] 前向递推的单步积分
//   与 FastLIVO2 的 prop_imu_once() 完全同构:
//     1) 半隐式: 先用角速度更新姿态, 再用【新姿态】把 body 加速度转到世界系;
//     2) 加速度 = 原始值 * imu_scale_ - ba_;
//     3) 世界系比力 = R * acc + g_  (g_ 是世界系重力)
//     4) 位置/速度用梯形积分.
//   差别只在加速度取值: FastLIVO2 用单样本, 这里用中点平均
//   (中点平均对白噪声的抑噪效果更好, 不改变整体行为).
// ============================================================
void ESKF::ForwardPropagateOnce(double dt, const V3& acc_raw, const V3& gyr_raw)
{
  // 角速度去偏置
  V3 gyr = gyr_raw - bg_;

  // 半隐式: 先更新姿态
  SO3 new_R = fw_R_ * SO3::Exp(gyr, dt);
  fw_R_ = new_R;

  // 加速度缩放 + 去偏置
  V3 acc = imu_scale_ * acc_raw - ba_;

  // 世界系比力 (用新姿态)
  V3 global_acc = fw_R_.R() * acc + g_;

  // 梯形积分
  fw_p_ = fw_p_ + fw_v_ * dt + 0.5 * global_acc * dt * dt;
  fw_v_ = fw_v_ + global_acc * dt;
}


// ============================================================
// [ALIGN-FastLIVO2] 前向递推的重锚定
//   在 EKF 更新完成 (后验 v_ 已确定) 后由 Observe() 调用.
//   对应 FastLIVO2 的 state_update_flg 分支:
//     - imu_propagate = latest_ekf_state   (前向状态重置到后验)
//     - 丢弃早于锚点的 IMU
//     - 剩下的 IMU 逐条重积分 (每条一个 ~5ms 的 dt)
//   这里只做"重置 + 丢弃 + 置位", 真正的逐条重积分推迟到下一次
//   ForwardPredict() 里做 (因为那时才知道缓冲里最新到哪一条).
// ============================================================
void ESKF::NotifyForwardAnchor(double anchor_time)
{
  // 前向状态重置到当前 EKF 后验 (fw_R_/fw_p_/fw_v_ 已在 Update() 中同步)
  fw_anchor_time_ = anchor_time;

  // 丢弃早于锚点时刻的 IMU (锚点之前的状态已由 EKF 后验给出, 不需要再递推)
  while (!fw_imu_buffer_.empty() && fw_imu_buffer_.front().secs < anchor_time) {
    fw_imu_buffer_.pop_front();
  }

  // 需要"逐条重积分"
  fw_state_update_flg_ = true;

  // 积分时间基重置到锚点, 下一次重积分从锚点开始.
  fw_last_integr_time_ = anchor_time;
  fw_inited_ = true;
}


// ============================================================
// [ALIGN-FastLIVO2] 前向递推 (纯 IMU 推算) 的主入口
//   与 Predict(imu, state_imu, state_robot) 等价, 但行为对齐 FastLIVO2:
//     - 维护独立的递推 IMU 缓冲 (fw_imu_buffer_)
//     - 锚点后逐条重积分, 步长恒为 ~5ms (而不是一次大 dt)
// ============================================================
bool ESKF::ForwardPredict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot)
{
  // 入缓冲 (递推专用副本, 与 EKF 去畸变用的 imu_buffer_ 无关).
  // 注意: 放在 init_ 判断之前, 否则 init 之前的 IMU 会缺失,
  //       导致第一次锚点时缓冲不完整.
  fw_imu_buffer_.push_back(imu);

  // 缓冲不能无限增长: 锚点会丢弃旧数据, 这里再兜一个上限.
  while (fw_imu_buffer_.size() > 2000) {
    fw_imu_buffer_.pop_front();
  }

  if (!init_) {
    return false;
  }

  // ---- 锚点后: 逐条重积分 ----
  if (fw_state_update_flg_) {
    fw_state_update_flg_ = false;

    // 重置到 EKF 后验 (fw_R_/fw_p_/fw_v_ 已由 Update() 同步)
    fw_last_integr_time_ = fw_anchor_time_;

    // 逐条重放缓冲里的 IMU, 每条一个自身 dt.
    // 加速度取值:
    //   - 第 1 条 (区间 [锚点, t0]): 锚点前一条已被丢弃, 只能用单样本 acc;
    //   - 之后: 用相邻两条的中点平均 (对白噪声降噪, 比单样本更平滑).
    //   FastLIVO2 全部用单样本; 这里在保持"逐条细粒度步长"这个关键行为
    //   一致的前提下, 对可配对的区间额外做了中点平均.
    bool has_prev = false;
    IMUData prev_imu;
    V3 last_gyr_used = V3::Zero();
    V3 last_acc_used = V3::Zero();
    for (const auto& imu_item : fw_imu_buffer_) {
      double dt = imu_item.secs - fw_last_integr_time_;
      if (dt <= 0.0) {
        // 该样本落在锚点时刻或之前, 不积分, 但要作为中点配对的"前一条"保留.
        prev_imu = imu_item;
        has_prev = true;
        continue;
      }
      if (dt > 0.2) break;   // 异常间隔, 丢弃后面的

      V3 acc_use = imu_item.acc;
      V3 gyr_use = imu_item.gyr;
      if (has_prev) {
        acc_use = 0.5 * (imu_item.acc + prev_imu.acc);
        gyr_use = 0.5 * (imu_item.gyr + prev_imu.gyr);
      }

      ForwardPropagateOnce(dt, acc_use, gyr_use);
      fw_last_integr_time_ = imu_item.secs;
      fw_last_imu_ = imu_item;
      prev_imu = imu_item;
      has_prev = true;
      last_gyr_used = gyr_use - bg_;
      last_acc_used = imu_scale_ * acc_use - ba_;
    }

    state_imu.time = imu.secs;
    state_imu.R = fw_R_.R_;
    state_imu.p = fw_p_;
    state_imu.v = fw_v_;
    state_imu.w = last_gyr_used;
    state_imu.a = last_acc_used;

    SO3 robo_R;
    robo_R.R_ = fw_R_.R_ * g_odom_robo.R_;
    V3 robo_p = fw_R_.R() * (-g_odom_robo.R_ * g_odom_robo.t_) + fw_p_;
    state_robot.time = imu.secs;
    state_robot.R = robo_R.R_;
    state_robot.p = robo_p;

    return true;
  }

  // ---- 锚点之后的连续推进 ----
  if (!fw_inited_) {
    // 还没锚定过 (等待第一个 EKF 后验)
    fw_last_integr_time_ = imu.secs;
    fw_last_imu_ = imu;
    fw_inited_ = true;
    return false;
  }

  double dt = imu.secs - fw_last_integr_time_;
  if (dt <= 0.0) {
    // 时间倒退或重复样本: 只更新历史, 不积分
    fw_last_imu_ = imu;
    return false;
  }
  if (dt > 0.2) {
    // 异常大间隔: 重新锚定到当前时刻, 避免用一条 acc 外推一大段
    fw_last_integr_time_ = imu.secs;
    fw_last_imu_ = imu;
    return false;
  }

  // 中点平均 (相邻两条 IMU 间隔正常时, 这是有效的降噪手段)
  V3 acc_mid = 0.5 * (imu.acc + fw_last_imu_.acc);
  V3 gyr_mid = 0.5 * (imu.gyr + fw_last_imu_.gyr);

  ForwardPropagateOnce(dt, acc_mid, gyr_mid);

  fw_last_integr_time_ = imu.secs;
  fw_last_imu_ = imu;

  state_imu.time = imu.secs;
  state_imu.R = fw_R_.R_;
  state_imu.p = fw_p_;
  state_imu.v = fw_v_;
  state_imu.w = gyr_mid - bg_;
  state_imu.a = imu_scale_ * acc_mid - ba_;

  SO3 robo_R;
  robo_R.R_ = fw_R_.R_ * g_odom_robo.R_;
  V3 robo_p = fw_R_.R() * (-g_odom_robo.R_ * g_odom_robo.t_) + fw_p_;
  state_robot.time = imu.secs;
  state_robot.R = robo_R.R_;
  state_robot.p = robo_p;

  return true;
}


bool ESKF::Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot){
  // [ALIGN-FastLIVO2] 已统一走 ForwardPredict().
  // 旧实现在 EKF 锚点后不做"逐条重积分", 会用单条 acc 外推一大段 dt,
  // 导致每个 LIO 帧向速度注入一个尖峰.
  return ForwardPredict(imu, state_imu, state_robot);
}


bool ESKF::Predict(const IMUData& imu) {

  if(last_imu_time_ < 0){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  if(imu.secs <= last_obs_time_){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }
  
  current_time_ = imu.secs;

  double dt;
  if(last_imu_time_ < last_obs_time_){
    dt = imu.secs - last_obs_time_;
  }else if (imu.secs > current_obs_time_){
    dt = current_obs_time_ - last_imu_time_;
    current_time_ = current_obs_time_;
  }else{
    dt = imu.secs - last_imu_time_;
  }

  V3 acc = 0.5 * (imu.acc + last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  body_omega_ = 0.5 * (imu.gyr + last_imu_.gyr) - bg_;
  M3 Jr_dt = (dt * RightJacobianSO3(body_omega_, dt)).cast<scalar>();   // J_l(-phi) = J_r(phi)

  M3 R_m3 = R_.R_;
  M3 R_dt = R_m3 * dt;

  F_X f_x = F_X::Identity();
  f_x.template block<3, 3>(0, 0) = SO3::Exp(-body_omega_, dt).R_;
  f_x.template block<3, 3>(0, 9) = - Jr_dt;
  f_x.template block<3, 3>(3, 6) = M3::Identity() * dt;
  f_x.template block<3, 3>(6, 0) = - R_m3 * SO3::hat(acc) * dt;
  f_x.template block<3, 3>(6, 12) = - R_dt;
  f_x.template block<3, 3>(6, 15) = M3::Identity() * dt;
  

  F_W f_w = F_W::Zero();
  f_w.template block<3, 3>(0, 0) = - Jr_dt;
  f_w.template block<3, 3>(6, 3) = - R_dt;                 // v -> na
  f_w.template block<3, 3>(9, 6) = M3::Identity() * dt;    // ba
  f_w.template block<3, 3>(12, 9) = M3::Identity() * dt;   // bg

  P_ = f_x * P_ * f_x.transpose() + f_w * Q_ * f_w.transpose();

  global_acc_ = R_.R() * acc + g_;
  p_ = p_ + v_ * dt + 0.5 * global_acc_ * dt * dt;
  v_ = v_ + global_acc_ * dt;
  R_ = R_ * SO3::Exp(body_omega_, dt);

  last_imu_time_ = imu.secs;
  last_imu_ = imu;
  return true;
}


const int STATE_DIM = 18;
bool ESKF::UpdateObserve(ESKF::ObsFunc obs) {
  SO3 R_0 = R_;

  M6 HTVH;     // H^T * V^(-1) * H
  V6 HTVr;     // H^T * V^(-1) * residuals
  M18 Pk, Qk;

  need_converge_ = false;
  for (int iter = 0; iter < options_.num_iterations_; ++iter) {
    if(iter > 2){
      need_converge_ = true;
    }

    obs(GetKFState(), HTVH, HTVr);
    /**
     * J = diag(I_3, I_3, J_theta, I_3, I_3, I_3)
     * J_theta = I - 1/2 * δθ_k ∧
     * δθ_k = Log(R_k^T * R_0)  ?  --> Log(R_0^T * R_k) ?
     */
    Pk = P_;
    M3 J_theta = M3::Identity() - 0.5 * SO3::hat((R_0.inverse() * R_).log_vee());
    for(int j = 0; j < STATE_DIM; j+=3){
      Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
    }
    for(int j = 0; j < STATE_DIM; j+=3){
      Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
    }

    /**
     * The normal Kalman filter formula: 
     * K_k = P_k * H_k^T * (H_k * P_k * H_k^T + V)^(-1)
     * δx_k = K_k * (z - h(x_k))
     * 
     * Using SMW identity transformation：
     * (A * B * (D + C * A * B)^(-1)) = ((A^(-1) + B * D^(-1) * C)^(-1) * B * D^(-1))
     * K_k = (P_k^(-1) + H_k^T * V^(-1) * H_k)^(-1) * H_k^T * V^(-1)
     * Q_k = (P_k^(-1) + H_k^T * V^(-1) * H_k)^(-1)
     * HTVr = H_k^T * V^(-1) * residuals
     * residuals = (z - h(x_k))
     */

    M18 Pk_inv = Pk.inverse();
    Pk_inv.block<6,6>(0,0) += HTVH;

    Qk = Pk_inv.inverse();

    V18 error_dx = V18::Zero();
    error_dx.head(6) = HTVr;

    dx_ = Qk * error_dx;

    // update norm state
    Update();

    if (dx_.lpNorm<Eigen::Infinity>() < options_.quit_eps_ && iter > 0) {
      break;
    }
  }

  // update P
  M18 temp_cov = M18::Identity();
  temp_cov.block<6,6>(0,0) = HTVH;
  P_ = (M18::Identity() - Qk * temp_cov) * Pk;

  Pk = P_;

  // project P
  M3 J_theta = M3::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(0, 0));
  for(int j = 0; j < STATE_DIM; j+=3){
    Pk.block<3,3>(0, j).noalias() = J_theta * P_.block<3,3>(0, j);
  }
  for(int j = 0; j < STATE_DIM; j+=3){
    Pk.block<3,3>(j, 0) = Pk.block<3,3>(j, 0) * J_theta.transpose();
  }

  P_ = Pk;

  dx_.setZero();
  
  last_obs_time_ = current_obs_time_;
  return true;
}

}
