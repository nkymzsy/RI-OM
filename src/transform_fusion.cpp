#include "utility.hpp"

Eigen::Matrix4d detal_T=Eigen::Matrix4d::Identity(); 
nav_msgs::Odometry laserOdometry;
ros::Publisher inter_odom_pub;

bool init=false;
kalman_filter_3D fliter;



void msg_callback_bef(nav_msgs::Odometry::ConstPtr msg)
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Matrix4d T_raw=Eigen::Matrix4d::Identity();
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    t.x() = msg->pose.pose.position.x;
    t.y() = msg->pose.pose.position.y;
    t.z() = msg->pose.pose.position.z;
    T_raw.block(0,0,3,3)=q.matrix();
    T_raw.block(0,3,3,1)=t;

    Eigen::Matrix4d T;
    T = detal_T *   T_raw;

    Eigen::Matrix3d R_pub=T.block(0,0,3,3);
    Eigen::Quaterniond q_pub(R_pub);
    Eigen::Vector3d t_pub(T.block(0,3,3,1));
    
    t_pub=fliter.input(t_pub);
    laserOdometry.pose.pose.orientation.x = q_pub.x();
    laserOdometry.pose.pose.orientation.y = q_pub.y();
    laserOdometry.pose.pose.orientation.z = q_pub.z();
    laserOdometry.pose.pose.orientation.w = q_pub.w();
    laserOdometry.pose.pose.position.x = t_pub.x();
    laserOdometry.pose.pose.position.y = t_pub.y();
    laserOdometry.pose.pose.position.z = t_pub.z();
    inter_odom_pub.publish(laserOdometry);
}

void msg_callback_aft(nav_msgs::Odometry::ConstPtr msg)
{
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    t.x() = msg->pose.pose.position.x;
    t.y() = msg->pose.pose.position.y;
    t.z() = msg->pose.pose.position.z;
    detal_T.block(0,0,3,3)=q.matrix();
    detal_T.block(0,3,3,1)=t;
}


int main(int argc, char **argv)
{
    ROS_INFO("transform fusion starts");
    
    ros::init(argc, argv, "transform_fusion");
    ros::NodeHandle n;
    ros::Subscriber fast_sub = n.subscribe("fast_odom", 10, msg_callback_bef);
    ros::Subscriber pointcloud_sub = n.subscribe("odom_detal_T", 10, msg_callback_aft);
    inter_odom_pub=  n.advertise<nav_msgs::Odometry>("inter_odom", 10);
    laserOdometry.header.frame_id="map";
    ros::spin();

    return 0;

}
