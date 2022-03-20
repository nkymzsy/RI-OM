#ifndef   SCAN_TO_MAP_HPP
#define SCAN_TO_MAP_HPP

#include "common.h"
#include "ndt_map.h"

namespace zsy_ndt_lib
{
    class scan_to_map
    {
    private:

        double para_angle[3];
        double para_t[3];

        double para_angle_old[3];
        double para_t_old[3];

        //处理时需要记录的有用数据
        //scan now
        std::vector<Eigen::Vector3d> mean_scan;
        // std::vector<Eigen::Matrix3d> cov_scan;  //无用
        //map
        std::vector<Eigen::Vector3d> mean_old;
        std::vector<Eigen::Matrix3d> cov_old;
        std::vector<float> certainty_old;

        pcl::PointCloud<pcl::PointXYZ> mean_sub_pcl;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        float max_distence=1;

        void maketree(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,std::vector<Eigen::Vector3d> &scan);
    public:
        scan_to_map(ndt_map *all_map, std::vector<Eigen::Vector3d> &scan,Eigen::Matrix4d &T);
        void optimize_once();
        void optimize(uint times=100);
        void  result(Eigen::Matrix4d &T);
    };

}

#endif