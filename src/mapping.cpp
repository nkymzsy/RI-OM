#include "utility.hpp"

const float size_voxel = 1;
zsy_ndt_lib::closeloop_detection close_det_opt; 

Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
Eigen::Matrix4d T_last_bef_inv; 
Eigen::Matrix4d inter_detal_T;  
uint32_t stamp;

struct local_map_struct
{
    std::shared_ptr<zsy_ndt_lib::mean_scan> scan2mean; 
    std::shared_ptr<zsy_ndt_lib::ndt_map> local_ndtmap;           
    std::shared_ptr<std::vector<Eigen::Vector3d>> curr_scan; 
    Eigen::Matrix4d curr_scan_T;                           
    Eigen::Vector3d t_last;                                        
} localmap;

struct global_map_struct
{
    std::mutex mtx;
    std::vector<std::vector<Eigen::Vector3d>> pointclouds;
    std::queue<std::shared_ptr<std::vector<Eigen::Vector3d>>> pointclouds_queue;
    std::vector<Eigen::Matrix4d> T;
    std::vector<Eigen::Matrix4d> T_for_show;
    bool close_updata = false; 
    std::vector<std::pair<uint, uint>> constraint_index;

    Eigen::Vector3d t_last; 
    Eigen::Matrix4d T_last;
} global_map;

struct ros_msg_struct
{
    ros_msg_struct()
    {
        odom_aft_mapped.header.frame_id = "map";
        global_path.header.frame_id = "map";
    }

    std::queue<ri_om::key_info::ConstPtr> msg_Buf; //前端发过来的消息的接收队列

    ros::Publisher golbal_map;
    ros::Publisher pub_cur_scan;
    ros::Publisher pub_global_map;
    ros::Publisher pub_odom_aft_mapped;
    ros::Publisher pub_odom_detal_T;
    ros::Publisher pub_loop_constraint;
    ros::Publisher pub_global_path;

    nav_msgs::Odometry odom_aft_mapped;
    nav_msgs::Odometry odom_detal_T;
    nav_msgs::Path global_path;

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

        global_path.poses.push_back(pose_stamped);
    }

    void update_path(const std::vector<Eigen::Matrix4d> &T_lists)
    {
        global_path.poses.clear();
        for (auto iter : T_lists)
        {
            update_path(iter);
        }
    }
    void publish_globle_map()
    {
        std::vector<Eigen::Matrix4d> T_list;
        global_map.mtx.lock();
        T_list = global_map.T_for_show;
        while (!global_map.pointclouds_queue.empty())
        {
            std::vector<Eigen::Vector3d> pointcloud = *(global_map.pointclouds_queue.front());
            global_map.pointclouds.emplace_back(std::move(pointcloud));
            global_map.pointclouds_queue.pop();
        }
        global_map.mtx.unlock();

        if (T_list.size() == 0)
            return;

        pcl::PointCloud<pcl::PointXYZ> global_map_pointcloud; //全局地图
        uint size_all = 0;
        for (uint i = 0; i < global_map.pointclouds.size(); i++)
        {
            size_all += global_map.pointclouds[i].size();
        }
        global_map_pointcloud.reserve(size_all);
        pcl::PointXYZ p;
        for (uint i = 0; i < global_map.pointclouds.size(); i++)
        {
            Eigen::Vector3d p_eigen;
            Eigen::Vector3d t = T_list[i].block(0, 3, 3, 1);
            Eigen::Matrix3d R = T_list[i].block(0, 0, 3, 3);

            for (uint j = 0; j < global_map.pointclouds[i].size(); j++)
            {
                p_eigen = R * global_map.pointclouds[i][j] + t;
                p.x = p_eigen.x();
                p.y = p_eigen.y();
                p.z = p_eigen.z();
                global_map_pointcloud.push_back(p);
            }
        }

        //降采样
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(global_map_pointcloud.makeShared());
        sor.setLeafSize(0.4, 0.4f, 0.4f); //体素大小
        sor.filter(global_map_pointcloud);

        sensor_msgs::PointCloud2 msg_pub;
        pcl::toROSMsg(global_map_pointcloud, msg_pub);
        msg_pub.header.frame_id = "map";
        pub_global_map.publish(msg_pub);
    }

    void publish_visualize_loop()
    {
        if (global_map.constraint_index.empty())
            return;

        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = "map";
        markerNode.header.stamp = ros::Time::now();
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = "map";
        markerEdge.header.stamp = ros::Time::now();
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it : global_map.constraint_index)
        {
            int key_cur = it.first;
            int key_pre = it.second;
            geometry_msgs::Point p;
            p.x = global_map.T[key_cur](0, 3);
            p.y = global_map.T[key_cur](1, 3);
            p.z = global_map.T[key_cur](2, 3);
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = global_map.T[key_pre](0, 3);
            p.y = global_map.T[key_pre](1, 3);
            p.z = global_map.T[key_pre](2, 3);
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pub_loop_constraint.publish(markerArray);
    }

    void publish_T_now()
    {

        Eigen::Matrix3d R = T.block(0, 0, 3, 3);
        Eigen::Vector3d t = T.block(0, 3, 3, 1);
        Eigen::Quaterniond q(R);
        odom_aft_mapped.header.seq = stamp;
        odom_aft_mapped.pose.pose.orientation.x = q.x();
        odom_aft_mapped.pose.pose.orientation.y = q.y();
        odom_aft_mapped.pose.pose.orientation.z = q.z();
        odom_aft_mapped.pose.pose.orientation.w = q.w();
        odom_aft_mapped.pose.pose.position.x = t.x();
        odom_aft_mapped.pose.pose.position.y = t.y();
        odom_aft_mapped.pose.pose.position.z = t.z();

        pub_odom_aft_mapped.publish(odom_aft_mapped);
    }

    void publish_local_map()
    {
        pcl::PointCloud<pcl::PointXYZ> new_scan; 
        new_scan.reserve(localmap.curr_scan->size());
        pcl::PointXYZ p;
        Eigen::Vector3d p_eigen;
        Eigen::Vector3d t = localmap.curr_scan_T.block(0, 3, 3, 1);
        Eigen::Matrix3d R = localmap.curr_scan_T.block(0, 0, 3, 3);
        for (uint j = 0; j < localmap.curr_scan->size(); j++)
        {
            p_eigen = R * (*localmap.curr_scan)[j] + t;
            p.x = p_eigen.x();
            p.y = p_eigen.y();
            p.z = p_eigen.z();
            new_scan.push_back(p);
        }
        sensor_msgs::PointCloud2 msg_pub;
        pcl::toROSMsg(new_scan, msg_pub);
        msg_pub.header.frame_id = "map";
        pub_cur_scan.publish(msg_pub);
    }

    void publish_inter_T_detal()
    {
        Eigen::Matrix3d R = inter_detal_T.block(0, 0, 3, 3);
        Eigen::Vector3d t = inter_detal_T.block(0, 3, 3, 1);
        Eigen::Quaterniond q(R);
        odom_detal_T.pose.pose.orientation.x = q.x();
        odom_detal_T.pose.pose.orientation.y = q.y();
        odom_detal_T.pose.pose.orientation.z = q.z();
        odom_detal_T.pose.pose.orientation.w = q.w();
        odom_detal_T.pose.pose.position.x = t.x();
        odom_detal_T.pose.pose.position.y = t.y();
        odom_detal_T.pose.pose.position.z = t.z();

        pub_odom_detal_T.publish(odom_detal_T);
    }

    void publish_msg()
    {
        publish_inter_T_detal();

        publish_local_map();

        publish_visualize_loop();

        pub_global_path.publish(global_path);
    }

} ros_msg;



void msg_callback(ri_om::key_info::ConstPtr msg)
{
    ros_msg.msg_Buf.push(msg);
}

void process()
{

    const ri_om::key_info::ConstPtr msg = ros_msg.msg_Buf.front();
    ros_msg.msg_Buf.pop();

    Eigen::Matrix4d T_fast;
    std::shared_ptr<std::vector<Eigen::Vector3d>> scan(new std::vector<Eigen::Vector3d>);

    zsy_ndt_lib::from_ros_msg(msg, T_fast, *scan, stamp);

    if (T == Eigen::Matrix4d::Zero())
    {
        T = T_fast;
        T_last_bef_inv = T_fast.inverse();
        //如果是第一帧的话要先建局部地图
        localmap.curr_scan = scan;
        localmap.curr_scan_T = T;

        localmap.scan2mean.reset(new zsy_ndt_lib::mean_scan(size_voxel));
        localmap.local_ndtmap.reset(new zsy_ndt_lib::ndt_map(size_voxel));
        localmap.local_ndtmap->insert_scan(*scan);
        localmap.t_last = T.block(0, 3, 3, 1);

        global_map.mtx.lock();
        global_map.pointclouds_queue.push(scan);
        global_map.mtx.unlock();
        global_map.T.push_back(T);
        global_map.t_last = T.block(0, 3, 3, 1);
        global_map.T_last = T;

        std::vector<Eigen::Vector3d> mean;
        std::vector<Eigen::Matrix3d> cov;
        localmap.local_ndtmap->mean_and_cov(mean, cov);
        close_det_opt.add_a_close_key(mean, T);

        ros_msg.update_path(T);
    }
    else
    {
        std::vector<Eigen::Matrix4d> T_list;
        std::vector<std::pair<uint, uint>> constraint;
        std::shared_ptr<zsy_ndt_lib::ndt_map> new_local_ndtmap;
        if (close_det_opt.find_close(T_list, constraint, new_local_ndtmap)) //loop detection
        {
            //修正子图位姿
            Eigen::Matrix4d detal_T = T_list.back() * (global_map.T[T_list.size() - 1].inverse());
            localmap.curr_scan_T = detal_T * localmap.curr_scan_T;

            //修正当前位置()
            T = detal_T * T;
            //修正全局位姿
            for (uint i = 0; i < T_list.size(); i++)
            {
                global_map.T[i] = T_list[i];
            }
            for (uint i = T_list.size(); i < global_map.T.size(); i++)
            {
                global_map.T[i] = detal_T * global_map.T[i];
            }
            global_map.constraint_index = constraint;

            global_map.T_last = detal_T * global_map.T_last;
            global_map.t_last = global_map.T_last.block(0, 3, 3, 1);
            ros_msg.update_path(global_map.T);

            //修正子图
            localmap.local_ndtmap.reset();
            localmap.local_ndtmap = new_local_ndtmap;

            if ((localmap.curr_scan) != NULL)
            {
                localmap.local_ndtmap->insert_scan(*localmap.curr_scan, localmap.curr_scan_T);
            }
        }
    }
    Eigen::Matrix4d T_detal = T_last_bef_inv * T_fast;
    T = T * T_detal;
    T_last_bef_inv = T_fast.inverse();
    //scan-to-map matching
    std::vector<Eigen::Vector3d> scan_mean;
    localmap.scan2mean->get_mean(*scan,scan_mean);
    zsy_ndt_lib::scan_to_map scan2map(localmap.local_ndtmap.get(), scan_mean, T); 
    scan2map.optimize(20);
    scan2map.result(T);

    inter_detal_T = T * T_last_bef_inv;

    Eigen::Vector3d t_now = T.block(0, 3, 3, 1);
    //for local map
    localmap.curr_scan = scan; 
    localmap.curr_scan_T = T;
    if ((localmap.t_last - t_now).norm() > 0.2)
    {
        localmap.t_last = T.block(0, 3, 3, 1);
        localmap.local_ndtmap->insert_scan(*scan, T);
        localmap.local_ndtmap->cutting(T.block(0, 3, 3, 1));
    }

    //for global  map
    if ((global_map.t_last - t_now).norm() > 0.4)
    {
        Eigen::Matrix4d detal_T = global_map.T_last.inverse() * T;
        close_det_opt.add_a_close_key(*scan, detal_T);

        std::shared_ptr<std::vector<Eigen::Vector3d>> flitered_scan(new std::vector<Eigen::Vector3d>);
        *flitered_scan=*scan;
        localmap.local_ndtmap->wash(*flitered_scan, T);
        ros_msg.update_path(T);
        global_map.T.push_back(T);
        global_map.t_last = T.block(0, 3, 3, 1);
        global_map.T_last = T;

        global_map.mtx.lock();
        global_map.T_for_show = global_map.T;
        global_map.pointclouds_queue.push(flitered_scan);
        global_map.mtx.unlock();
    }
}


void loop_detection()
{
    ros::Rate rate(5);
    while (ros::ok())
    {
        rate.sleep();
        //close_det_opt.do_detection();    //location-based closed-loop detection
        close_det_opt.do_detection_sc(); //using scan-context
    }
}

void visualize_globemap_Thread()
{
    ros::Rate rate(0.2); 
    while (ros::ok())
    {
        rate.sleep();
        ros_msg.publish_globle_map(); 
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Mapping process starts");

    ros::init(argc, argv, "mapping");
    ros::NodeHandle n;
    ros::Subscriber pointcloud_sub = n.subscribe("/key_info", 1000, msg_callback);

    ros_msg.pub_odom_aft_mapped = n.advertise<nav_msgs::Odometry>("odom_aft_mapped", 1);
    ros_msg.pub_odom_detal_T = n.advertise<nav_msgs::Odometry>("odom_detal_T", 1);

    ros_msg.pub_loop_constraint = n.advertise<visualization_msgs::MarkerArray>("loop_constraint", 1);
    ros_msg.pub_global_path = n.advertise<nav_msgs::Path>("global_path", 1);

    ros_msg.pub_cur_scan = n.advertise<sensor_msgs::PointCloud2>("curr_scan", 1);
    ros_msg.pub_global_map = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

    std::thread visualizeMapThread(&visualize_globemap_Thread);
    std::thread loop_thread(&loop_detection);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!ros_msg.msg_Buf.empty())
        {
            process(); 
            ros_msg.publish_msg();
        }
    }
    visualizeMapThread.join();
    loop_thread.join();

    return 0;
}