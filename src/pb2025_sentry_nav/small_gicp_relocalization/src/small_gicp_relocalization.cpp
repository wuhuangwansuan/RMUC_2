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
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);//地图
  this->get_parameter("odom_frame", odom_frame_);//odom
  this->get_parameter("base_frame", base_frame_);//base_footprint
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);//front_mid360
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() = Eigen::Vector3d(init_pose_[0], init_pose_[1], init_pose_[2]);
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

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
  
  //抛开协方差置信度表示，仅将降采样后的点云复制给rough_target
  rough_target->resize(target_->size());
  for (size_t i = 0; i < target_->size(); ++i) {
      rough_target->points[i].x = target_->points[i].x;
      rough_target->points[i].y = target_->points[i].y;
      rough_target->points[i].z = target_->points[i].z;
  }
  rough_target->width = target_->width;
  rough_target->height = target_->height;
  

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

void SmallGicpRelocalizationNode::rough_match()
{
  //计算法线
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_;
  normal_.setRadiusSearch(0.1);                                                              //法线估计半径 记得改为全局变量
  normal_.setInputCloud(rough_source);
  normal_.compute(*r_normal_source);   
  normal_.setInputCloud(rough_target);
  normal_.compute(*r_normal_target);

  //计算特征值
  pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_;
  fpfh_.setRadiusSearch(0.001);                                                                        //设定特征值估计半径 记得改为全局变量
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

  //SVD估计刚性变换
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> rough_trans_;
  Eigen::Matrix4f transformation;
  rough_trans_.estimateRigidTransformation(*rough_source, *rough_target, *correspondences, transformation);
  previous_result_t_.matrix() = transformation.cast<double>();
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

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);//downsample the accumulated_cloud to 'registered_leaf_size_' and put it into 'source_'

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);//get covariance with 'num_threads'

  //抛开协方差置信度表示，仅将降采样后的点云复制给rough_source
  rough_source->resize(source_->size());
  for (size_t i = 0; i < source_->size(); ++i) {
      rough_source->points[i].x = source_->points[i].x;
      rough_source->points[i].y = source_->points[i].y;
      rough_source->points[i].z = source_->points[i].z;
  }
  rough_source->width = source_->width;
  rough_source->height = source_->height;

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> (
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));//source_tree_先验点云地图中的kdtree
  if (!source_ || !source_tree_) {
    return;
  }

  rough_match();//------------------------------粗配准和细配准现在是共用同一个估计位姿变量名，不知道是否会有影响。

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;//最大欧氏距离，加速每次迭代，防止个别极值拉偏迭代的位姿
  register_->optimizer.max_iterations = 20;//最大迭代循环次数（20为常用值）

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);//计算并迭代'target'和'source'的刚体变换矩阵

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;//将估计位置替换为当前位置,即最终刚体变换
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
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
