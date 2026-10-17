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

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
//细配准置信度检测
#include <Eigen/Dense>
#include <small_gicp/registration/registration_result.hpp>
#include <small_gicp/util/lie.hpp>   // skew(), se3_exp()

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ===== [RELOC-FIX] P2-2/P2-3: 粗配准改为返回成功与否，便于失败时回退 =====
  bool rough_match();
  // ===== [RELOC-FIX] 检查函数改为纯检查：先验与模式由参数传入，不再修改成员状态 =====
  /// @param prior       NIS-2 使用的先验位姿（同时也是细配准初值）
  /// @param coarse_mode true=粗配准后首次检查，false=一直细配准
  bool reloc_checker(
    const small_gicp::RegistrationResult & result,
    const Eigen::Isometry3d & prior,
    bool coarse_mode);
  /// SE3 对数映射，与 small_gicp::se3_exp 约定一致（旋转在前）
  static Eigen::Matrix<double, 6, 1> se3_log(const Eigen::Isometry3d & T);
  /// 卡方分位数（Wilson–Hilferty 近似，无外部依赖）
  static double chi2_quantile(int dof, double alpha);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  // ===== [RELOC-FIX] P2-2/P2-3: 粗配准参数（FPFH 搜索半径 / RANSAC 外点剔除）=====
  double fpfh_radius_;              // FPFH 特征搜索半径 [m]
  double ransac_inlier_threshold_;  // RANSAC 内点阈值 [m]
  int    ransac_max_iterations_;    // RANSAC 最大迭代次数
  int    ransac_min_inliers_;       // RANSAC 最少内点数
  std::vector<double> init_pose_;
//细配准置信度检测
  bool   enable_reloc_check_;
  double nis_alpha_;             // 显著性水平 α
  double nis_prior_trans_std_;   // 细配准先验平移标准差 [m]
  double nis_prior_rot_std_;     // 细配准先验旋转标准差 [rad]
  // ===== [RELOC-FIX] 新增粗配准先验不确定度（原为硬编码，且旋转/平移疑似写反）=====
  double coarse_prior_trans_std_;   // 粗配准先验平移标准差 [m]
  double coarse_prior_rot_std_;     // 粗配准先验旋转标准差 [rad]
  double min_inlier_ratio_;      // 最小内点率
  double max_hessian_cond_;      // H 条件数上限

  bool fine_trust_{false};

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  Eigen::Isometry3d rough_result_t_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr rough_target;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rough_source;
  pcl::PointCloud<pcl::Normal>::Ptr r_normal_target;
  pcl::PointCloud<pcl::Normal>::Ptr r_normal_source;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source;

  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
