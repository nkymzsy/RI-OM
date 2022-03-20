#ifndef COMMON_H
#define COMMON_H


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h> 

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>




void vector_to_pcl( std::vector<Eigen::Vector3d> &points_vector,pcl::PointCloud<pcl::PointXYZ> &points_pcl);
void pcl_to_vector( std::vector<Eigen::Vector3d> &points_vector,pcl::PointCloud<pcl::PointXYZ> &points_pcl);
void undistort(std::vector<Eigen::Vector3d> &pointcloud,Eigen::Matrix4d detal_T);
void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres);


#endif      