#ifndef COMMON_HPP
#define COMMON_HPP

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <fstream>

#include "ri_om/key_info.h"


#include "../lib/ndt_map.h"
#include "../lib/common.h"
#include "../lib/scan_to_map.h"

#include "../lib/ros_msg_help.h"
#include "../lib/closeloop_detection.h"
#include "../lib/kalman.hpp"



void trans_to_map(std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d T)
{
    Eigen::Matrix3d R=T.block(0,0,3,3);
    Eigen::Vector3d t=T.block(0,3,3,1);
    for(uint i=0;i<points.size();i++)
    {
        points[i]=R*points[i]+t;
    }
}

#endif