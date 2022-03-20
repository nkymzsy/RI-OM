#ifndef _ROS_MSG_HELP_
#define _ROS_MSG_HELP_

#include <Eigen/Core>
#include <vector>
#include <mutex>
#include <queue>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

namespace zsy_ndt_lib
{
    void to_ros_msg(ri_om::key_info &msg, Eigen::Matrix4d &T, std::vector<Eigen::Vector3d> &pointcloud, uint32_t stamp);
    void from_ros_msg(ri_om::key_info::ConstPtr msg, Eigen::Matrix4d &T, std::vector<Eigen::Vector3d> &pointcloud,uint32_t &stamp);

}

#endif