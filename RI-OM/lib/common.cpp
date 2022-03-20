#include "common.h"


void pcl_to_vector( std::vector<Eigen::Vector3d> &points_vector,pcl::PointCloud<pcl::PointXYZ> &points_pcl)
{
    points_vector.reserve(points_pcl.size());
    for(auto iter:points_pcl )
    {
        points_vector.emplace_back(iter.x,iter.y,iter.z);
    }
}

void vector_to_pcl( std::vector<Eigen::Vector3d> &points_vector,pcl::PointCloud<pcl::PointXYZ> &points_pcl)
{
    points_pcl.reserve(points_vector.size());
    pcl::PointXYZ p;
    for(auto iter:points_vector )
    {
        p.x=iter.x();
        p.y=iter.y();
        p.z=iter.z();
        points_pcl.push_back(p);
    }
}


void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        if(cloud_in.points[i].z<-3)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}