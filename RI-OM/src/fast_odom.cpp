#include "utility.hpp"

std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_Buf;
ros::Publisher pub_key_info; 
Eigen::Matrix4d T_to_init= Eigen::Matrix4d::Zero(); 

struct ros_msg_struct
{
    ros_msg_struct()
    {
        fast_odom_path.header.frame_id = "map";
        laserOdometry.header.frame_id = "map";
    }
    ros::Publisher pub_key;
    ros::Publisher pub_fast_path;
    ros::Publisher pub_fast_odom;

    nav_msgs::Path fast_odom_path;
    nav_msgs::Odometry laserOdometry;

    void update_path(const Eigen::Matrix4d &T)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = T(0, 3);
        pose_stamped.pose.position.y = T(1, 3);
        pose_stamped.pose.position.z = T(2, 3);
        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        Eigen::Quaterniond q(R);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        fast_odom_path.poses.push_back(pose_stamped);
    }
    void publish_path(const Eigen::Matrix4d &T_to_init)
    {
        update_path(T_to_init);
        pub_fast_path.publish(fast_odom_path);
    }
    void publish_odom(const Eigen::Matrix4d &T,uint32_t stamp)
    {
        laserOdometry.header.stamp = ros::Time().now();
        laserOdometry.header.seq=stamp;
        Eigen::Matrix3d R=T.block(0,0,3,3);
        Eigen::Quaterniond q(R);
        Eigen::Vector3d t(T.block(0,3,3,1));
        laserOdometry.pose.pose.orientation.x = q.x();
        laserOdometry.pose.pose.orientation.y = q.y();
        laserOdometry.pose.pose.orientation.z = q.z();
        laserOdometry.pose.pose.orientation.w = q.w();
        laserOdometry.pose.pose.position.x = t.x();
        laserOdometry.pose.pose.position.y = t.y();
        laserOdometry.pose.pose.position.z = t.z();
        pub_fast_odom.publish(laserOdometry);
    }
}ros_msg;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloud_Buf.push(msg);
}

uint32_t count=0;
Eigen::Matrix4d detal_T_last=Eigen::Matrix4d::Identity();
pcl::PointCloud<pcl::PointXYZ> scan_new_raw;
pcl::PointCloud<pcl::PointXYZ> scan_new_down;
pcl::PointCloud<pcl::PointXYZ> scan_old;

void process()
{

    const sensor_msgs::PointCloud2::ConstPtr msg_in = cloud_Buf.front();
    cloud_Buf.pop();
    pcl::fromROSMsg(*msg_in, scan_new_raw);

    std::vector<int> index; 
    pcl::removeNaNFromPointCloud(scan_new_raw, scan_new_raw, index);
    //removeClosedPointCloud(scan_new_raw,scan_new_raw,5);

    pcl::VoxelGrid<pcl::PointXYZ> down; 
    down.setInputCloud(scan_new_raw.makeShared());
    down.setLeafSize (0.5f, 0.5f, 0.5f);
    down.filter(scan_new_down);


    count++;
    if (T_to_init == Eigen::Matrix4d::Zero())
    {
        T_to_init = Eigen::Matrix4d::Identity();
        ri_om::key_info msg;
        Eigen::Matrix4d detal_T_init =Eigen::Matrix4d::Identity();

        std::vector<Eigen::Vector3d> scan_vector;
        pcl_to_vector(scan_vector, scan_new_raw);
        zsy_ndt_lib::to_ros_msg(msg, detal_T_init, scan_vector,count);

        ros_msg.pub_key.publish(msg);
        scan_old.swap(scan_new_raw);
    }
    else
    {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>  gicp;
        gicp.setMaxCorrespondenceDistance(1);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setMaximumIterations (30);
        gicp.setRANSACIterations(0);
        gicp.setInputSource (scan_new_down.makeShared());
        gicp.setInputTarget (scan_old.makeShared());

        pcl::PointCloud<pcl::PointXYZ> output;
        gicp.align(output);
        Eigen::Matrix4d detal_T= gicp.getFinalTransformation().cast<double>();
        T_to_init=T_to_init*detal_T;
        
        if(count%5==0)
        {
            ri_om::key_info msg;

            std::vector<Eigen::Vector3d> scan_vector;
            pcl_to_vector(scan_vector, scan_new_raw);

            zsy_ndt_lib::to_ros_msg(msg,T_to_init,scan_vector,count);
            msg.header.stamp=msg_in->header.stamp; 
            ros_msg.pub_key.publish(msg);
        }
        ros_msg.publish_path(T_to_init);
        ros_msg.publish_odom(T_to_init,count);
        scan_old.swap(scan_new_down);
    }
}



int main(int argc, char **argv)
{
    ROS_INFO("fast odomtery process starts");
    ros::init(argc, argv, "fast_odomtery");
    ros::NodeHandle n;
    ros::Subscriber pointcloud_sub = n.subscribe("/rslidar_points", 1000, pointcloud_callback);
    
    ros_msg.pub_key = n.advertise<ri_om::key_info>("key_info", 100, false);
    ros_msg.pub_fast_path = n.advertise<nav_msgs::Path>("fast_path", 100);
    ros_msg.pub_fast_odom = n.advertise<nav_msgs::Odometry>("/fast_odom", 100);

    while (ros::ok())
    {
        if (!cloud_Buf.empty())
        {
            process();
        }
        ros::spinOnce();
    }

    return 0;
}



