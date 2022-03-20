#include "ri_om/key_info.h"
#include "ros_msg_help.h"
namespace zsy_ndt_lib
{
    void to_ros_msg(ri_om::key_info &msg,Eigen::Matrix4d &T,std::vector<Eigen::Vector3d> &pointcloud,uint32_t stamp )
    {
        msg.cloud.reserve(pointcloud.size()*3);

        for(auto iter:pointcloud)
        {
            msg.cloud.push_back(iter.x());
            msg.cloud.push_back(iter.y());
            msg.cloud.push_back(iter.z());
        }
        for(uint i=0;i<16;i++)
        {
            msg.T.push_back(T(i));
        }
        msg.stamp=stamp;
    }


    void from_ros_msg(ri_om::key_info::ConstPtr msg,Eigen::Matrix4d &T,std::vector<Eigen::Vector3d> &pointcloud, uint32_t &stamp)
    {

        for(uint i=0;i<16;i++)
        {
            T(i)=msg->T[i];
        }
        pointcloud.reserve(msg->cloud.size()/3);
        for(uint i=0;i<(msg->cloud.size());i=i+3)
        {
            pointcloud.emplace_back(msg->cloud[i],msg->cloud[i+1],msg->cloud[i+2]);
        }
        stamp=msg->stamp;
    }

}