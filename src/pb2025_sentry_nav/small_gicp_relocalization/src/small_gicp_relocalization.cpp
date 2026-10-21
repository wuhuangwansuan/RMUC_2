// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/icp.h"
// ===== [RELOC-FIX] P2-3: RANSAC 外点剔除所需头文件 =====
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  // ===== [RELOC-FIX] P2-2/P2-3: 粗配准参数 =====
  this->declare_parameter("fpfh_radius", 0.25);              // FPFH 搜索半径 [m]
  this->declare_parameter("ransac_inlier_threshold", 0.5);   // RANSAC 内点阈值 [m]
  this->declare_parameter("ransac_max_iterations", 1000);    // RANSAC 最大迭代次数
  this->declare_parameter("ransac_min_inliers", 10);         // RANSAC 最少内点数
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
//粗配准检测参数
  this->declare_parameter("enable_reloc_check", true);
  this->declare_parameter("nis_alpha", 0.05);
  this->declare_parameter("nis_prior_trans_std", 0.5);
  this->declare_parameter("nis_prior_rot_std", 0.1);
  // ===== [RELOC-FIX] 新增粗配准先验不确定度参数 =====
  this->declare_parameter("coarse_prior_trans_std", 1.5);     // 粗配准平移标准差 [m]
  this->declare_parameter("coarse_prior_rot_std", 0.2618);    // 粗配准旋转标准差 [rad] ≈ 15°
  this->declare_parameter("min_inlier_ratio", 0.3);
  this->declare_parameter("max_hessian_cond", 1.0e6);

  // ===== ##2026-10-20## [RELOC-FIX] P2-2: 粗/细两级配准参数 =====
  // coarse_leaf_size：粗配准所用点云的体素边长。点数随 leaf³ 下降，
  //   本场地先验地图 0.1 m→1.36e5 点，1.5 m→约 6.7e2 点（约 200 倍差距）。
  this->declare_parameter("coarse_leaf_size", 1.5);
  // fine_leaf_size：细配准（GICP）源点云体素边长；<=0 时沿用 registered_leaf_size。
  this->declare_parameter("fine_leaf_size", 0.0);
  // GICP 迭代上限：粗配准模式初值差，给更多迭代；位姿可信后无需太多。
  this->declare_parameter("coarse_max_iterations", 50);
  this->declare_parameter("fine_max_iterations", 30);
  // 粗配准结果相对上一可信位姿允许的最大修正量，超出即判为跑飞并回退。
  this->declare_parameter("coarse_max_translation_delta", 8.0);
  this->declare_parameter("coarse_max_rotation_delta_deg", 35.0);
  // map->odom 时间戳前推量，供下游按略晚于扫描时刻查询变换。
  this->declare_parameter("tf_future_offset_sec", 0.1);


  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  // ===== [RELOC-FIX] P2-2/P2-3: 读取粗配准参数 =====
  this->get_parameter("fpfh_radius", fpfh_radius_);
  this->get_parameter("ransac_inlier_threshold", ransac_inlier_threshold_);
  this->get_parameter("ransac_max_iterations", ransac_max_iterations_);
  this->get_parameter("ransac_min_inliers", ransac_min_inliers_);
  this->get_parameter("map_frame", map_frame_);//地图
  this->get_parameter("odom_frame", odom_frame_);//odom
  this->get_parameter("base_frame", base_frame_);//base_footprint
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);//front_mid360
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  //粗配准检测参数
  this->get_parameter("enable_reloc_check", enable_reloc_check_);
  this->get_parameter("nis_alpha", nis_alpha_);
  this->get_parameter("nis_prior_trans_std", nis_prior_trans_std_);
  this->get_parameter("nis_prior_rot_std", nis_prior_rot_std_);
  // ===== [RELOC-FIX] 读取粗配准先验不确定度参数 =====
  this->get_parameter("coarse_prior_trans_std", coarse_prior_trans_std_);
  this->get_parameter("coarse_prior_rot_std", coarse_prior_rot_std_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_hessian_cond", max_hessian_cond_);

  // ===== ##2026-10-20## [RELOC-FIX] P2-2: 读取粗/细两级配准参数 =====
  this->get_parameter("coarse_leaf_size", coarse_leaf_size_);
  this->get_parameter("fine_leaf_size", fine_leaf_size_);
  this->get_parameter("coarse_max_iterations", coarse_max_iterations_);
  this->get_parameter("fine_max_iterations", fine_max_iterations_);
  this->get_parameter("coarse_max_translation_delta", coarse_max_translation_delta_);
  this->get_parameter("coarse_max_rotation_delta_deg", coarse_max_rotation_delta_deg_);
  this->get_parameter("tf_future_offset_sec", tf_future_offset_sec_);
  // fine_leaf_size 未配置时保持旧行为（与 GICP 源点云原来的分辨率一致）  ##2026-10-20##
  if (fine_leaf_size_ <= 0.0f) {
    fine_leaf_size_ = registered_leaf_size_;
  }
  // 粗配准体素必须比细配准更粗，否则粗配准没有任何提速意义  ##2026-10-20##
  if (coarse_leaf_size_ < fine_leaf_size_) {
    RCLCPP_WARN(
      this->get_logger(),
      "coarse_leaf_size (%.3f) < fine_leaf_size (%.3f), clamping to fine_leaf_size",
      coarse_leaf_size_, fine_leaf_size_);
    coarse_leaf_size_ = fine_leaf_size_;
  }


  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() = Eigen::Vector3d(init_pose_[0], init_pose_[1], init_pose_[2]);
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  //若无初始位姿，则使用单位矩阵进行初始化
  previous_result_t_ = result_t_;
  rough_result_t_ = result_t_;

  //粗配准先验点云和目标点云
  rough_source = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  rough_target = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  //粗配准法线
  r_normal_source = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  r_normal_target = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

  //粗配准FPFH特征对象
  fpfh_source = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh_target = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();//累积的帧点云
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);//提取先验点云

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);//对提取的先验点云降采样得到target_
  
  if(!fine_trust_)
  {
    // ===== ##2026-10-20## [RELOC-FIX] P2-2: 粗配准目标点云按 coarse_leaf_size 真正降采样 =====
    // 这里不是简单复制 target_（那样分辨率仍是 global_leaf_size=0.1 m，约 1.36e5 点），
    // 而是把 target_ 再体素化到 coarse_leaf_size（1.5 m 时约 6.7e2 点）。
    // 之后 FPFH 特征与暴力对应搜索的规模下降约两个数量级，粗配准耗时从分钟级降到亚秒级。
    // 从 target_ 而不是 global_map_ 出发，是因为 target_ 已完成 0.1 m 体素化，点数少 8 倍，
    // 且每个 0.1 m 体素等权，得到的粗体素质心比原始扫描点密度更均匀。
    rough_target = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointCovariance>, pcl::PointCloud<pcl::PointXYZ>>(
      *target_, coarse_leaf_size_, num_threads_);
    RCLCPP_INFO(
      this->get_logger(),
      "Coarse registration target: %zu -> %zu points (leaf %.2f m)",
      target_->size(), rough_target->size(), coarse_leaf_size_);
  }

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));//接收实时点云，累积得到帧点云

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));//获得初始位置

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));//细配准，计算纠正的坐标转换

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));//发布map系下的odom坐标
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);//将先验点云从lidar系变换到odom系，并暂存在global_map_
}

//接收实时点云，累积得到帧点云 accumulated_cloud_
void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

Eigen::Matrix<double, 6, 1>
SmallGicpRelocalizationNode::se3_log(const Eigen::Isometry3d & T)
{
  Eigen::Matrix<double, 6, 1> xi;
  const Eigen::AngleAxisd aa(T.linear());
  const Eigen::Vector3d omega = aa.angle() * aa.axis();  // 旋转向量
  xi.head<3>() = omega;                                   // 旋转在前

  const double theta = omega.norm();
  if (theta < 1e-10) {
    xi.tail<3>() = T.translation();                       // 小角度近似
  } else {
    const Eigen::Matrix3d Omega = small_gicp::skew(omega);
    const Eigen::Matrix3d V =
      Eigen::Matrix3d::Identity()
      + (1.0 - std::cos(theta)) / (theta * theta) * Omega
      + (theta - std::sin(theta)) / (theta * theta * theta) * Omega * Omega;
    xi.tail<3>() = V.inverse() * T.translation();         // V⁻¹ t
  }
  return xi;
}

double SmallGicpRelocalizationNode::chi2_quantile(int dof, double alpha)
{
  // 标准正态单边分位数查表（线性插值）
  static const double z_tab[][2] = {
    {0.10, 1.2816}, {0.05, 1.6449}, {0.02, 2.0537},
    {0.01, 2.3263}, {0.005, 2.5758}, {0.001, 3.0902}};
  double z = z_tab[2][1];
  if (alpha >= z_tab[0][0]) {
    z = z_tab[0][1];
  } else {
    for (size_t i = 0; i + 1 < 6; ++i) {
      if (alpha <= z_tab[i][0] && alpha >= z_tab[i + 1][0]) {
        const double t = (z_tab[i][0] - alpha) / (z_tab[i][0] - z_tab[i + 1][0]);
        z = z_tab[i][1] + t * (z_tab[i + 1][1] - z_tab[i][1]);
        break;
      }
    }
  }
  const double k = static_cast<double>(dof);
  const double s = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
  return k * s * s * s;   // Wilson–Hilferty
}

//细配准置信度检测
// ===== [RELOC-FIX] 签名扩展：传入先验位姿与粗/细配准模式；移除内部对 previous_result_t_ 的修改 =====
bool SmallGicpRelocalizationNode::reloc_checker(
  const small_gicp::RegistrationResult & result,
  const Eigen::Isometry3d & prior,
  bool coarse_mode)
{
  if (!enable_reloc_check_) {
    return true;
  }

  const size_t N = result.num_inliers;
  const size_t M = source_ ? source_->size() : 0;
  if (N < 10 || M == 0) {
    RCLCPP_WARN(this->get_logger(), "[NIS] too few inliers: N=%zu, M=%zu", N, M);
    return false;
  }

  // ---- 守卫 1：内点率 ----
  const double inlier_ratio = static_cast<double>(N) / static_cast<double>(M);
  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_WARN(this->get_logger(), "[NIS] inlier ratio too low: %.3f < %.3f",
                inlier_ratio, min_inlier_ratio_);
    return false;
  }

  // ---- NIS-1：量测空间（残差型）----
  const int dof_res = static_cast<int>(3 * N);
  const double eps_res = 2.0 * result.error;
  const double thr_res = chi2_quantile(dof_res, nis_alpha_);
  RCLCPP_INFO(this->get_logger(), "[NIS-1] eps_res=%.1f  thr=%.1f  (dof=%d, avg/pt=%.2f)",
              eps_res, thr_res, dof_res, eps_res / static_cast<double>(dof_res));
  if (eps_res > thr_res) {
    RCLCPP_WARN(this->get_logger(), "[NIS-1] residual NIS rejected");
    return false;
  }
  // ---- NIS-2：位姿空间（创新型）----
  // ===== [RELOC-FIX] 按模式选择先验不确定度；delta 以传入的 prior 为基准（不再依赖成员副作用）=====
  const double prior_rot_std = coarse_mode ? coarse_prior_rot_std_ : nis_prior_rot_std_;
  const double prior_trans_std = coarse_mode ? coarse_prior_trans_std_ : nis_prior_trans_std_;

  const Eigen::Matrix<double, 6, 1> delta =
    se3_log(prior.inverse() * result.T_target_source);

  Eigen::Matrix<double, 6, 6> Sigma_prior = Eigen::Matrix<double, 6, 6>::Zero();
  Sigma_prior.diagonal().head<3>().setConstant(prior_rot_std * prior_rot_std);
  Sigma_prior.diagonal().tail<3>().setConstant(prior_trans_std * prior_trans_std);

  // H 可能奇异，加正则并用特征分解做截断伪逆
  const Eigen::Matrix<double, 6, 6> H_reg =
    result.H + 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H_reg);
  Eigen::Matrix<double, 6, 1> ev = es.eigenvalues();
  // ===== [RELOC-FIX] 伪逆阈值改为相对阈值，避免依赖 H 的绝对量纲 =====
  const double ev_floor = 1e-9 * std::max(es.eigenvalues().maxCoeff(), 1.0);
  for (int i = 0; i < 6; ++i) {
    ev(i) = (ev(i) > ev_floor) ? 1.0 / ev(i) : 0.0;   // 截断伪逆
  }
  const Eigen::Matrix<double, 6, 6> H_inv =
    es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();

  const Eigen::Matrix<double, 6, 6> S = Sigma_prior + H_inv;
  // ===== [RELOC-FIX] LLT 分解失败时显式拒绝，避免 NaN 继续传播 =====
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(S);
  if (llt.info() != Eigen::Success) {
    RCLCPP_WARN(this->get_logger(), "[NIS-2] innovation covariance is not positive definite");
    return false;
  }
  const Eigen::Matrix<double, 6, 1> w = llt.matrixL().solve(delta);  // 白化
  const double eps_pose = w.squaredNorm();

  const double thr_pose = chi2_quantile(6, nis_alpha_);
  RCLCPP_INFO(this->get_logger(), "[NIS-2] eps_pose=%.3f  thr=%.3f  (dof=6)", eps_pose, thr_pose);
  if (!std::isfinite(eps_pose) || eps_pose > thr_pose) {
    RCLCPP_WARN(this->get_logger(), "[NIS-2] pose NIS rejected");
    return false;
  }

  // ---- 守卫 4：H 条件数（几何退化）----
  const double lmax = es.eigenvalues().maxCoeff();
  const double lmin = std::max(es.eigenvalues().minCoeff(), 1e-12);
  const double cond = lmax / lmin;
  RCLCPP_INFO(this->get_logger(), "[NIS] cond(H)=%.2e", cond);
  if (cond > max_hessian_cond_) {
    RCLCPP_WARN(this->get_logger(), "[NIS] degenerate geometry: cond=%.2e", cond);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[NIS] relocalization ACCEPTED");
  return true;
}

bool SmallGicpRelocalizationNode::rough_match()
{
  if (!rough_source || !rough_target || rough_source->empty() || rough_target->empty()) {
    RCLCPP_WARN(this->get_logger(), "[ROUGH] empty source/target cloud");
    return false;
  }

  // ===== ##2026-10-20## [RELOC-FIX] P2-2: 法线 / FPFH 搜索半径必须随粗配准体素一起放大 =====
  // 若沿用 0.1 m 的法线半径，在 1.5 m 体素的粗配准点云上每个点都找不到邻居，
  // 法线退化为 NaN，FPFH 全为零，粗配准必然失败。约定：
  //   法线半径 = 2 × coarse_leaf_size
  //   FPFH 半径 = max(fpfh_radius, 3 × coarse_leaf_size)
  const double coarse_normal_radius = 2.0 * static_cast<double>(coarse_leaf_size_);
  const double coarse_fpfh_radius =
    std::max(fpfh_radius_, 3.0 * static_cast<double>(coarse_leaf_size_));

  //计算法线
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_;
  normal_.setRadiusSearch(coarse_normal_radius);   // 与粗配准体素匹配，见上
  normal_.setInputCloud(rough_source);
  normal_.compute(*r_normal_source);   
  normal_.setInputCloud(rough_target);
  normal_.compute(*r_normal_target);

  //计算特征值
  pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_;
  fpfh_.setRadiusSearch(coarse_fpfh_radius);   // ##2026-10-20## [RELOC-FIX] P2-2: 随 coarse_leaf_size 自动放大
  fpfh_.setInputCloud(rough_source);
  fpfh_.setInputNormals(r_normal_source);
  fpfh_.compute(*fpfh_source);
  fpfh_.setInputCloud(rough_target);
  fpfh_.setInputNormals(r_normal_target);
  fpfh_.compute(*fpfh_target);

  //建立对应关系
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,pcl::FPFHSignature33> corr_est_;
  corr_est_.setInputSource(fpfh_source);
  corr_est_.setInputTarget(fpfh_target);
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  corr_est_.determineCorrespondences(*correspondences);

  if (correspondences->size() < 3) {
    RCLCPP_WARN(this->get_logger(), "[ROUGH] too few FPFH correspondences: %zu", correspondences->size());
    return false;
  }

  // ===== [RELOC-FIX] P2-3: 用 RANSAC 剔除 FPFH 错配，避免直接 SVD 被离群点拉偏 =====
  // ##2026-10-20## 内点阈值改为随粗配准体素放大：粗体素内的质心偏差可达约半个体素对角线，
  // 沿用 0.5 m 会把墙面上本来正确的对应点判成外点。
  const double coarse_inlier_threshold =
    std::max(ransac_inlier_threshold_, 0.6 * static_cast<double>(coarse_leaf_size_));
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac;
  ransac.setInputSource(rough_source);
  ransac.setInputTarget(rough_target);
  ransac.setInlierThreshold(coarse_inlier_threshold);
  ransac.setMaximumIterations(ransac_max_iterations_);
  ransac.setRefineModel(false);

  pcl::CorrespondencesPtr inliers(new pcl::Correspondences);
  ransac.getRemainingCorrespondences(*correspondences, *inliers);

  if (static_cast<int>(inliers->size()) < ransac_min_inliers_) {
    RCLCPP_WARN(
      this->get_logger(), "[ROUGH] RANSAC inliers too few: %zu < %d (raw: %zu)",
      inliers->size(), ransac_min_inliers_, correspondences->size());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "[ROUGH] RANSAC inliers: %zu / %zu",
    inliers->size(), correspondences->size());

  //SVD估计刚性变换（使用 RANSAC 内点）
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> rough_trans_;
  Eigen::Matrix4f transformation;
  rough_trans_.estimateRigidTransformation(*rough_source, *rough_target, *inliers, transformation);
  rough_result_t_.matrix() = transformation.cast<double>();
  return true;
}

//细配准，计算纠正的坐标转换 得到source_
void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }else {
    RCLCPP_INFO(this->get_logger(), "////Performing registration with %zu accumulated points ////", accumulated_cloud_->size());
  }

  // ===== ##2026-10-20## [RELOC-FIX] P2-2: 细配准源点云使用 fine_leaf_size =====
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, fine_leaf_size_);//downsample the accumulated_cloud to 'fine_leaf_size_' and put it into 'source_'

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);//get covariance with 'num_threads'

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> (
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));//source_tree_先验点云地图中的kdtree
  if (!source_ || !source_tree_) {
    return;
  }

  // ===== [RELOC-FIX] 粗配准结果作为细配准初值，保证粗/细配准真正衔接 =====
  // 细配准初值：粗配准模式下用粗配准结果，否则用上一帧可信位姿
  const bool coarse_mode = !fine_trust_;
  Eigen::Isometry3d prior = previous_result_t_;

  if (coarse_mode) {
    // ===== ##2026-10-20## [RELOC-FIX] P2-2: 粗配准源点云按 coarse_leaf_size 真正降采样 =====
    // 必须与 rough_target 使用同一体素尺寸，否则两侧 FPFH 描述子的尺度不一致，
    // 匹配率极低。这里从 source_ 出发（已是 fine_leaf_size，点数远少于原始累积帧）。
    rough_source = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointCovariance>, pcl::PointCloud<pcl::PointXYZ>>(
      *source_, coarse_leaf_size_, num_threads_);

    // ===== [RELOC-FIX] P2-2/P2-3: 粗配准失败时回退到上一可信位姿 =====
    if (rough_match()) {
      // ===== ##2026-10-20## [RELOC-FIX] P2-2: 粗配准结果合理性守卫 =====
      // FPFH + RANSAC 在对称 / 空旷场地偶尔给出完全错误的解，若直接作为 GICP 初值，
      // 后续 NIS 还可能因为"自洽的错误位姿"而误接受。这里用相对上一可信位姿的
      // 平移/旋转修正量做一道硬门限（对应参数 coarse_max_translation_delta /
      // coarse_max_rotation_delta_deg）。
      static constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
      const Eigen::Isometry3d delta = rough_result_t_ * previous_result_t_.inverse();
      const double d_trans = delta.translation().norm();
      const double d_rot_deg = Eigen::AngleAxisd(delta.rotation()).angle() * kRadToDeg;
      if (
        d_trans > coarse_max_translation_delta_ || d_rot_deg > coarse_max_rotation_delta_deg_) {
        RCLCPP_WARN(
          this->get_logger(),
          "[ROUGH] coarse result rejected: delta t=%.2f m (limit %.2f), r=%.1f deg (limit %.1f)",
          d_trans, coarse_max_translation_delta_, d_rot_deg, coarse_max_rotation_delta_deg_);
        prior = previous_result_t_;
      } else {
        prior = rough_result_t_;   // 关键：用粗配准结果作为细配准初值
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[ROUGH] coarse registration failed, fallback to last trusted pose");
      prior = previous_result_t_;
    }
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;//最大欧氏距离，加速每次迭代，防止个别极值拉偏迭代的位姿
  // ===== ##2026-10-20## [RELOC-FIX] P2-2: 迭代上限按模式取值 =====
  // 粗配准模式的初值误差大，需要更多迭代才能收敛（原先硬编码 20 常出现 converged=false）；
  // 位姿可信后每帧只做微调，fine_max_iterations 足够。
  register_->optimizer.max_iterations = coarse_mode ? coarse_max_iterations_ : fine_max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, prior);//计算并迭代'target'和'source'的刚体变换矩阵

  // ===== [RELOC-FIX] 用局部 accepted 表示检查结果；只有"收敛且通过"才切换状态 =====
  const bool accepted = reloc_checker(result, prior, coarse_mode);//细配准置信度检测

  if (result.converged && accepted) {
    result_t_ = previous_result_t_ = result.T_target_source;//将估计位置替换为当前位置,即最终刚体变换
    fine_trust_ = true;   // [RELOC-FIX] 进入"一直细配准"模式
  } else {
    fine_trust_ = false;  // [RELOC-FIX] 失去信任，下一轮重新做粗配准
    RCLCPP_WARN(
      this->get_logger(),
      "Relocalization rejected (converged=%d, accepted=%d), fallback to coarse registration.",
      result.converged, accepted);
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // ##2026-10-20## 时间戳前推量改为参数 tf_future_offset_sec（原为硬编码 0.1 s），
  // 使下游在略晚于扫描时刻查询时仍能命中该变换
  // （见 https://robotics.stackexchange.com/a/96615）
  transform_stamped.header.stamp =
    last_scan_time_ + rclcpp::Duration::from_seconds(tf_future_offset_sec_);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  RCLCPP_INFO(this->get_logger(),"||x:%f",translation.x());
  RCLCPP_INFO(this->get_logger(),"||y:%f",translation.y());
  RCLCPP_INFO(this->get_logger(),"||z:%f",translation.z());

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);//由接受的实时点云获得当前预估的位置坐标
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
