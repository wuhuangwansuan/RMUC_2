#include<iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
 
using namespace std;
 
int main(int argc, char** argv)
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
 
    // 加载点云文件
    pcl::io::loadPLYFile("C:\\Users\\Administrator\\Desktop\\src.ply", *cloud_source);   
    pcl::io::loadPLYFile("C:\\Users\\Administrator\\Desktop\\tar.ply", *cloud_target); 
 
    // 创建法线对象
    pcl::PointCloud<pcl::Normal>::Ptr normals_source(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_target(new pcl::PointCloud<pcl::Normal>);
 
    // 估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nor;
    nor.setInputCloud(cloud_source);
    nor.setRadiusSearch(0.1);
    nor.compute(*normals_source);
    nor.setInputCloud(cloud_target);
    nor.compute(*normals_target);
 
    // 创建FPFH特征对象
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_source(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_target(new pcl::PointCloud<pcl::FPFHSignature33>);
 
    // 估计FPFH特征
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud_source);
    fpfh.setInputNormals(normals_source);
    fpfh.setRadiusSearch(0.001); // 设定特征估计的半径
    fpfh.compute(*fpfhs_source);
    fpfh.setInputCloud(cloud_target);
    fpfh.setInputNormals(normals_target);
    fpfh.compute(*fpfhs_target);
 
    // 建立对应关系
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_est;
    corr_est.setInputSource(fpfhs_source);
    corr_est.setInputTarget(fpfhs_target);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    corr_est.determineCorrespondences(*correspondences);
 
    // 使用SVD估计刚性变换
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    Eigen::Matrix4f transformation;
    trans_est.estimateRigidTransformation(*cloud_source, *cloud_target, *correspondences, transformation);
 
    // 创建ICP对象进行精细配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.align(*cloud_source, transformation); // 使用粗配准得到的变换矩阵进行初始对齐
 
    // 可视化
    pcl::visualization::PCLVisualizer viewer("FPFH Visualization");
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // 设置背景颜色为黑色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 255, 0, 0); // 红色表示源点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0); // 绿色表示目标点云
    viewer.addPointCloud(cloud_source, source_color, "source_cloud");
    viewer.addPointCloud(cloud_target, target_color, "target_cloud");
 
    // 主循环，直到用户关闭窗口
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
 
    return 0;
}