#include "lio/loop_closure/PoseGraphBackend.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "basic/logs.h"

namespace LI2Sup {

namespace {
/// 构造对角噪声模型 (Rot3 三个在前, Point3 三个在后).
gtsam::noiseModel::Diagonal::shared_ptr makeNoise(double rot_sigma,
                                                  double trans_sigma) {
  gtsam::Vector6 v;
  v << rot_sigma * rot_sigma, rot_sigma * rot_sigma, rot_sigma * rot_sigma,
       trans_sigma * trans_sigma, trans_sigma * trans_sigma,
       trans_sigma * trans_sigma;
  return gtsam::noiseModel::Diagonal::Variances(v);
}

/// [DIAG] 单个因子的残差诊断信息.
struct FactorDiag {
  std::string type;         ///< "Prior" / "Odom" / "Loop"
  std::size_t key_a = 0;    ///< 第一个节点
  std::size_t key_b = 0;    ///< 第二个节点
  bool is_prior = false;    ///< true = 先验因子
  bool is_loop = false;     ///< true = 回环因子
  bool is_new = false;      ///< true = 本次优化新增的因子
  double chi2 = 0.0;        ///< 该因子贡献的 chi2 (已按噪声模型白化)
  double trans_err = 0.0;   ///< 平移偏差 (m), 原始值
  double rot_err = 0.0;     ///< 旋转偏差 (rad), 原始值
  double trans_sigma = 0.0; ///< 平移 sigma
  double rot_sigma = 0.0;   ///< 旋转 sigma
  double baseline = 0.0;    ///< 回环因子: 两节点在图内的距离 (m)
};
} // namespace

PoseGraphBackend::PoseGraphBackend() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam_.reset(new gtsam::ISAM2(params));
}

PoseGraphBackend::~PoseGraphBackend() = default;

void PoseGraphBackend::configure(const Options& options) {
  options_ = options;
}

gtsam::Pose3 PoseGraphBackend::toGtsam(const BASIC::SE3& pose) {
  // BASIC::M3 (float) -> double 旋转矩阵 -> gtsam::Rot3
  //
  // [BUG FIX] 必须先做正交化再交给 GTSAM.
  //
  //   原因: GTSAM 4.2 编译时若定义了 GTSAM_USE_QUATERNIONS,
  //         Rot3(Matrix3) 走 Eigen 的 "矩阵 -> 四元数" 构造.
  //         Eigen 的这个构造【假设输入矩阵正交, 不做归一化】.
  //         如果传进去的 R 有微小正交性误差 (float 只有 ~7 位有效数字),
  //         得到的四元数就不是单位的.
  //         随后 GTSAM 的 Logmap/Expmap 会产生 NaN,
  //         线性系统求解失败 -> IndeterminantLinearSystemException.
  //
  //   做法: 用四元数中转, 强制归一化后再转回 double 矩阵.
  //         这样无论输入的 float 矩阵正交性如何, GTSAM 拿到的永远是
  //         严格正交的旋转.
  //
  //   fastlio_sam 没有这个问题: 它用 Rot3::RzRyRx(roll, pitch, yaw)
  //   从欧拉角构造, 天生正交.
  Eigen::Quaterniond q(pose.R_.cast<double>());
  q.normalize();
  const Eigen::Matrix3d R = q.toRotationMatrix();

  return gtsam::Pose3(gtsam::Rot3(R),
                      gtsam::Point3(static_cast<double>(pose.t_.x()),
                                    static_cast<double>(pose.t_.y()),
                                    static_cast<double>(pose.t_.z())));
}

BASIC::SE3 PoseGraphBackend::fromGtsam(const gtsam::Pose3& pose) {
  BASIC::SE3 out;
  // GTSAM 的结果本来是正交的, 但转成 float 后会有 ~1e-7 的误差.
  // 这里做一次正交化, 保证回写给 ESKF 的姿态不会引入正交性漂移.
  // (ESKF 侧也做了同样的处理, 这里是第二道防线.)
  Eigen::Quaterniond q(pose.rotation().matrix());
  q.normalize();
  out.R_ = q.toRotationMatrix().cast<BASIC::scalar>();
  out.t_ = pose.translation().cast<BASIC::scalar>();
  out.T_ = Eigen::Matrix4f::Identity();
  out.T_.block<3, 3>(0, 0) = out.R_;
  out.T_.block<3, 1>(0, 3) = out.t_;
  return out;
}

void PoseGraphBackend::addPriorFactor(std::size_t id,
                                      const BASIC::SE3& pose) {
  const auto noise =
      makeNoise(options_.prior_rot_sigma, options_.prior_trans_sigma);
  pending_graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      id, toGtsam(pose), noise);
  full_graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      id, toGtsam(pose), noise);
  pending_initial_.insert<gtsam::Pose3>(id, toGtsam(pose));
  ++pending_factors_;
  if (id + 1 > node_count_) node_count_ = id + 1;

  // [DIAG] 记录因子元信息 (与 full_graph_ 顺序一致).
  FactorMeta meta;
  meta.key_a = id;
  meta.key_b = id;
  meta.is_prior = true;
  meta.rot_sigma = options_.prior_rot_sigma;
  meta.trans_sigma = options_.prior_trans_sigma;
  meta.z = toGtsam(pose);
  meta.is_new = true;
  factor_metas_.push_back(meta);
}

void PoseGraphBackend::addOdomFactor(std::size_t id,
                                     const BASIC::SE3& pose_prev,
                                     const BASIC::SE3& pose_curr) {
  if (id == 0) return;

  // between(x_prev, x_curr) = x_prev^{-1} * x_curr
  const gtsam::Pose3 T_prev = toGtsam(pose_prev);
  const gtsam::Pose3 T_curr = toGtsam(pose_curr);
  const gtsam::Pose3 T_rel = T_prev.between(T_curr);

  const auto noise =
      makeNoise(options_.odom_rot_sigma, options_.odom_trans_sigma);
  pending_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id - 1, id, T_rel, noise);
  full_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id - 1, id, T_rel, noise);
  pending_initial_.insert<gtsam::Pose3>(id, T_curr);
  ++pending_factors_;
  if (id + 1 > node_count_) node_count_ = id + 1;

  // [DIAG] 记录因子元信息.
  FactorMeta meta;
  meta.key_a = id - 1;
  meta.key_b = id;
  meta.is_prior = false;
  meta.rot_sigma = options_.odom_rot_sigma;
  meta.trans_sigma = options_.odom_trans_sigma;
  meta.z = T_rel;
  meta.is_new = true;
  factor_metas_.push_back(meta);
}

void PoseGraphBackend::addLoopFactor(std::size_t id_from, std::size_t id_to,
                                     const BASIC::SE3& T_between,
                                     const double rot_sigma,
                                     const double trans_sigma) {
  if (id_from == id_to) return;

  const double rot = (rot_sigma > 0.0) ? rot_sigma : options_.loop_rot_sigma;
  const double trans =
      (trans_sigma > 0.0) ? trans_sigma : options_.loop_trans_sigma;

  const auto noise = makeNoise(rot, trans);
  pending_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id_from, id_to, toGtsam(T_between), noise);
  full_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      id_from, id_to, toGtsam(T_between), noise);
  ++pending_factors_;
  ++pending_loop_factors_;
  ++loop_factor_count_;

  // [DIAG] 记录因子元信息.
  FactorMeta meta;
  meta.key_a = id_from;
  meta.key_b = id_to;
  meta.is_prior = false;
  meta.rot_sigma = rot;
  meta.trans_sigma = trans;
  meta.z = toGtsam(T_between);
  meta.is_loop = true;
  meta.is_new = true;
  factor_metas_.push_back(meta);
}
bool PoseGraphBackend::optimize(std::vector<BASIC::SE3>& poses_out,
                                bool diag_enable) {
  poses_out.clear();

  // 提交新因子.
  //
  // [BUG FIX] 用 try/catch 包住 update().
  //   GTSAM 的 IndeterminantLinearSystemException (以及 CholeskyFailed 等)
  //   是从 update() / calculateBestEstimate() 里抛出的.
  //   之前只有 calculateBestEstimate() 被包了, 于是 update() 一抛异常
  //   就直接 terminate 掉整个节点 (日志里那句
  //   "terminate called after throwing an instance of
  //    gtsam::IndeterminantLinearSystemException" 就是这么来的).
  //
  //   fastlio_sam 也没有这层保护 (它一旦失败就是整个进程死掉),
  //   但对 super_lio 来说前端已经跑起来了, 因一个优化失败而整体退出
  //   代价太大, 所以这里兜住: 优化失败就返回 false, 让调用方跳过本次
  //   (LoopClosureManager 会放弃本批回环约束, 但图和关键帧继续用).
  if (pending_factors_ > 0) {
    try {
      isam_->update(pending_graph_, pending_initial_);

      // [对齐 fastlio_sam] 迭代次数:
      //   fastlio_sam 的 saveKeyFramesAndFactor() 是这样的:
      //       isam->update(gtSAMgraph, initialEstimate);
      //       isam->update();                       // 总是多来一次
      //       if (aLoopIsClosed) { isam->update() x5 }   // 有回环再补 5 次
      //
      //   这里完全照搬:
      //       - 无参数 update() 总是做 (它触发重线性化 + 重新求解);
      //       - 有回环因子时再多做 5 次, 让回环约束充分收敛.
      isam_->update();

      if (pending_loop_factors_ > 0) {
        for (int i = 0; i < 5; ++i) isam_->update();
      }
    } catch (const std::exception& e) {
      // 优化失败.
      //
      // 【重要】此时这些因子其实已经被推进到 ISAM2 的 nonlinearFactors_ 里了
      // (GTSAM 的 update() 是先加因子, 后做消元求解).
      // 所以我们【不能】把 pending_graph_ 重新塞回去让调用方重试,
      // 否则同一对节点之间会有两份相同的 BetweenFactor, 约束权重翻倍.
      //
      // 处理: 清空待提交队列 (因子已经在图里了), 标记本次失败, 返回 false.
      // ISAM2 内部状态可能不完整, 但由于因子结构没变, 后续 update()
      // 仍会继续在现有图上正常工作.
      pending_graph_.resize(0);
      pending_initial_.clear();
      pending_factors_ = 0;
      pending_loop_factors_ = 0;
      last_residual_ = std::numeric_limits<double>::infinity();
      LOG(WARNING) << YELLOW << " ---> [PoseGraph]: ISAM2 update failed: "
                   << e.what() << RESET;
      return false;
    }
    pending_graph_.resize(0);
    pending_initial_.clear();
    pending_factors_ = 0;
    pending_loop_factors_ = 0;
  }

  if (node_count_ == 0) return false;

  gtsam::Values estimate;
  try {
    estimate = isam_->calculateBestEstimate();
  } catch (const std::exception&) {
    return false;
  }
  if (estimate.empty()) return false;

  poses_out.reserve(node_count_);
  for (std::size_t i = 0; i < node_count_; ++i) {
    if (!estimate.exists(i)) {
      poses_out.clear();
      return false;
    }
    poses_out.push_back(fromGtsam(estimate.at<gtsam::Pose3>(i)));
  }
  estimates_ = poses_out;

  // 记录残差用于诊断.
  try {
    last_residual_ = full_graph_.error(estimate);
  } catch (const std::exception&) {
    last_residual_ = std::numeric_limits<double>::infinity();
  }

  // [DIAG] 逐因子残差诊断 (只在显式请求时执行, 默认关闭).
  if (diag_enable) {
    diagnoseFactorResiduals(true, estimate);
  }

  // [DIAG] 清掉"本次新增"标记.
  //
  //   放在诊断之后: 诊断要看到刚加进来的因子.
  //   下一个关键帧又会加新的里程计因子, 它们会被重新标记 is_new,
  //   但老的回环因子不会再被当成"新增"重复打印.
  for (FactorMeta& m : factor_metas_) {
    m.is_new = false;
  }

  return true;
}

// ============================================================
// [DIAG] 逐因子残差诊断
// ============================================================
//
// 目的: 定位残差 (chi2) 到底由哪些因子贡献.
//
//   残差大本身不一定是坏事 —— 它可能只是噪声模型取得过紧,
//   把正常的累积漂移放大了几个数量级. 需要区分:
//     - 误差集中在少数长基线回环因子上 -> 那些回环可能接错了地方;
//     - 误差均匀分布在里程计因子上   -> 是 sigma 过紧 + 累积漂移, 属正常.
//
// 开启时打印:
//   1) 一行 chi2 汇总 (总数 / 里程计均值 / 回环均值);
//   2) 本次【新增】回环因子的明细 (不限条数).
//
// 只打日志, 不改动任何优化逻辑.
std::size_t PoseGraphBackend::diagnoseFactorResiduals(
    bool enable, const gtsam::Values& estimate) const {
  if (!enable) return 0;
  if (full_graph_.empty() || estimate.empty()) return 0;

  std::vector<FactorDiag> diags;
  diags.reserve(full_graph_.size());

  double total_chi2 = 0.0;
  double odom_chi2 = 0.0;
  double loop_chi2 = 0.0;
  double prior_chi2 = 0.0;
  std::size_t odom_count = 0;
  std::size_t loop_count = 0;

  std::size_t factor_index = 0;
  for (const gtsam::NonlinearFactor::shared_ptr& factor : full_graph_) {
    if (!factor) {
      ++factor_index;
      continue;
    }
    // factor 本身【不使用】:
    //   GTSAM 各版本的 NonlinearFactor::error(values) 返回类型不一致
    //   (有的版本返回 double, 有的返回 Vector), 直接调用会编译失败.
    //   下面的 chi2 全部由我们自己记录的测量值 + 图内估计值算出.

    // 取该因子的元信息 (sigma / 节点 / 测量值).
    // full_graph_ 与 factor_metas_ 是同步 push 的, 顺序一一对应.
    const FactorMeta* meta = nullptr;
    if (factor_index < factor_metas_.size()) {
      meta = &factor_metas_[factor_index];
    }
    ++factor_index;
    if (!meta) continue;

    FactorDiag diag;
    diag.is_prior = meta->is_prior;
    diag.is_loop = meta->is_loop;
    diag.is_new = meta->is_new;
    diag.key_a = meta->key_a;
    diag.key_b = meta->key_b;
    diag.rot_sigma = meta->rot_sigma;
    diag.trans_sigma = meta->trans_sigma;

    // ---- 白化后的 chi2 贡献 (严格对齐 GTSAM 的定义) ----
    //
    // 【不调用 factor->error()】
    //   不同 GTSAM 版本的 NonlinearFactor::error(values) 返回类型不一致
    //   (本机编译报错: 它返回的是 double, 不是 Vector).
    //   改为完全自算 —— 我们自己记了因子的测量值 z 和 sigma,
    //   直接用图内估计值去对比, 结果更可控, 也不受版本影响.
    //
    // GTSAM 对 Pose3 因子的定义 (见 BetweenFactor::evaluateError /
    // PriorFactor::evaluateError):
    //   BetweenFactor<Pose3>(k1,k2,z):  误差 = Local(z, between(x_k1, x_k2))
    //   PriorFactor<Pose3>(k,z):        误差 = -Local(x_k, z)
    //
    //   Local 返回【6 维切空间误差向量】[w(3), u(3)]:
    //     w = 旋转向量 (Rot3::Logmap)
    //     u = 平移分量 —— 注意它【不是】delta 的原始平移,
    //         而是切空间坐标 (GTSAM_POSE3_EXPMAP=ON 时由 Logmap 给出,
    //         OFF 时是 first-order 的原始平移).
    //
    //   白化 = 逐分量除以对应的 sigma
    //          (前 3 个用 rot_sigma, 后 3 个用 trans_sigma),
    //   chi2 = ||白化后误差||^2.
    //
    // 【为什么用 traits<Pose3>::Local, 而不是自己写 Logmap】
    //   GTSAM 的 Local 是否等于 Logmap 取决于编译选项 GTSAM_POSE3_EXPMAP
    //   (默认 ON). 用 traits 里的公开接口, 才能保证和 GTSAM 自己的
    //   因子误差【完全一致】, 不受编译选项影响.
    gtsam::Pose3 x_a;
    gtsam::Pose3 x_b;
    try {
      x_a = estimate.at<gtsam::Pose3>(diag.key_a);
      x_b = diag.is_prior ? x_a : estimate.at<gtsam::Pose3>(diag.key_b);
    } catch (const std::exception&) {
      continue;
    }

    gtsam::Vector6 e;
    if (diag.is_prior) {
      // PriorFactor: 误差 = -Local(x_k, z)
      e = -gtsam::traits<gtsam::Pose3>::Local(x_a, meta->z);
    } else {
      // BetweenFactor: 误差 = Local(z, between(x_k1, x_k2))
      e = gtsam::traits<gtsam::Pose3>::Local(meta->z, x_a.between(x_b));
    }

    const gtsam::Vector3 w = e.head<3>();
    const gtsam::Vector3 u = e.tail<3>();

    // 平移偏差 (m): 切空间平移分量的模长.
    diag.trans_err = u.norm();

    // 旋转偏差 (rad): 旋转向量的模长.
    diag.rot_err = w.norm();

    // 白化后的 chi2: 逐分量白化后平方求和 (与 GTSAM 的
    // squaredMahalanobisDistance 完全一致).
    //
    // 【踩过的坑】早期版本写的是 `3.0 * t * t`, 其中 t = |v| / sigma.
    //   那是错的 —— |v| 已经是 3 维向量的模长, 它的平方【本身就是】
    //   3 个分量平方和, 再乘 3 等于把 chi2 放大了 3 倍.
    //   (GTSAM 的 error() 另有 0.5 的 loss 系数, 所以 DIAG 的 total
    //    恰好是 GTSAM residual 的 2 倍 —— 这个关系现在是有意保持的.)
    double chi2 = 0.0;
    if (diag.trans_sigma > 1e-12) {
      chi2 += (u / diag.trans_sigma).squaredNorm();
    }
    if (diag.rot_sigma > 1e-12) {
      chi2 += (w / diag.rot_sigma).squaredNorm();
    }
    diag.chi2 = chi2;
    total_chi2 += chi2;

    // 分类.
    //
    // 【必须用 meta->is_loop, 不能用 key_b == key_a + 1 反推】
    //   相邻两帧之间也可能存在回环约束 (比如刚走过的路又走回来),
    //   用下标相邻性反推会把它误分类成里程计因子.
    if (diag.is_prior) {
      prior_chi2 += diag.chi2;
      diag.type = "Prior";
    } else if (diag.is_loop) {
      loop_chi2 += diag.chi2;
      ++loop_count;
      diag.type = "Loop";
    } else {
      odom_chi2 += diag.chi2;
      ++odom_count;
      diag.type = "Odom";
    }

    // 回环因子: 顺便量一下两节点在图内的距离 (基线长度).
    if (diag.type == "Loop") {
      try {
        const gtsam::Pose3 pa = estimate.at<gtsam::Pose3>(diag.key_a);
        const gtsam::Pose3 pb = estimate.at<gtsam::Pose3>(diag.key_b);
        diag.baseline = (pb.translation() - pa.translation()).norm();
      } catch (const std::exception&) {
        // 忽略.
      }
    }

    diags.push_back(std::move(diag));
  }

  // 按 chi2 从大到小排.
  std::sort(diags.begin(), diags.end(),
            [](const FactorDiag& a, const FactorDiag& b) {
              return a.chi2 > b.chi2;
            });

  LOG(INFO) << GREEN << " ---> [PoseGraph][DIAG] chi2 breakdown: total = "
            << total_chi2 << " | odom = " << odom_chi2 << " ("
            << odom_count << " factors, avg "
            << (odom_count ? odom_chi2 / static_cast<double>(odom_count) : 0.0)
            << ")" << " | loop = " << loop_chi2 << " ("
            << loop_count << " factors, avg "
            << (loop_count ? loop_chi2 / static_cast<double>(loop_count) : 0.0)
            << ")" << " | prior = " << prior_chi2 << RESET;

  // 打印本次【新增】回环因子的明细.
  //
  // 【为什么只打新增的】
  //   全图 chi2 最大的那几个因子, 每次优化基本都是同样那几条早期回环,
  //   连续打 50 次都是重复内容, 没有新信息.
  //   而"本次新增的回环表现如何"每次都不同, 且正是需要盯的东西:
  //   dev 远大于 sigma => 这次回环接错了地方, 图会试图硬拉, 轨迹会变形.
  //
  // 不限条数: 正常情况下一次优化只新增 1 个回环 (检测线程 2Hz 每次只出
  // 一个候选); 但如果主线程被长时间阻塞, 检测线程会攒下一批候选一次性
  // 灌进来 —— 那正是最该逐条看清的情况, 所以这里不做条数截断.
  std::vector<const FactorDiag*> sel;
  for (const FactorDiag& d : diags) {
    if (d.is_new && d.is_loop) sel.push_back(&d);
  }

  for (std::size_t i = 0; i < sel.size(); ++i) {
    const FactorDiag& d = *sel[i];
    std::ostringstream oss;
    oss << " ---> [PoseGraph][DIAG] #" << (i + 1)
        << " " << d.type
        << " [" << d.key_a << "->" << d.key_b << "]"
        << " chi2 = " << d.chi2
        << " | dev = " << d.trans_err << " m (sigma " << d.trans_sigma
        << ", " << (d.trans_sigma > 1e-12 ? d.trans_err / d.trans_sigma : 0.0)
        << "x)"
        << " | rot = " << d.rot_err << " rad (sigma " << d.rot_sigma
        << ", " << (d.rot_sigma > 1e-12 ? d.rot_err / d.rot_sigma : 0.0)
        << "x)";
    if (d.is_loop) oss << " | baseline = " << d.baseline << " m";
    LOG(INFO) << YELLOW << oss.str() << RESET;
  }

  return diags.size();
}

bool PoseGraphBackend::getEstimate(std::size_t id, BASIC::SE3& out) const {
  if (id >= estimates_.size()) return false;
  out = estimates_[id];
  return true;
}

void PoseGraphBackend::reset() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam_.reset(new gtsam::ISAM2(params));
  pending_graph_.resize(0);
  full_graph_.resize(0);
  pending_initial_.clear();
  pending_factors_ = 0;
  pending_loop_factors_ = 0;
  last_residual_ = std::numeric_limits<double>::infinity();
  node_count_ = 0;
  loop_factor_count_ = 0;
  factor_metas_.clear();
  estimates_.clear();
}

} // namespace LI2Sup
