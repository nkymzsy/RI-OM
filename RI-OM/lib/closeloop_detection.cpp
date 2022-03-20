#include "closeloop_detection.h"

namespace zsy_ndt_lib
{
    closeloop_detection::closeloop_detection()
    {
        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        noise_prior = gtsam::noiseModel::Diagonal::Variances(Vector6);
        noise_odom = gtsam::noiseModel::Diagonal::Variances(Vector6);

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);
    }

    void closeloop_detection::add_a_close_key(std::vector<Eigen::Vector3d> points, Eigen::Matrix4d detal_T)
    {
        mutex_need_to_detection.lock();
        std::shared_ptr<close_key_data> new_close_key(new close_key_data(points, detal_T));  
        new_close_key_queue.push(new_close_key);
        need_to_detection = true;
        mutex_need_to_detection.unlock();
    }


    void closeloop_detection::do_detection_sc()
    {

        mutex_need_to_detection.lock();
        bool need=need_to_detection;
        need_to_detection=false;
        new_close_key_queue_temp.swap(new_close_key_queue);
        mutex_need_to_detection.unlock();
        //need loop detection
        if (need)
        {
            std::vector<std::pair<uint, uint>> constraint_new;  
            while(!new_close_key_queue_temp.empty())
            {
                close_key_data data=*new_close_key_queue_temp.front();
                new_close_key_queue_temp.pop();
                if( !close_key_history.empty())
                {
                    data.T=close_key_history.back().T*data.T;
                }
                close_key_history.emplace_back(std::move(data));



                if (close_key_history.size() == 1)
                {
                    gtsam::Pose3 pose = gtsam::Pose3(close_key_history.front().T);
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(1, pose, noise_prior));
                    isamInitEstimate.insert(1, pose);
                }
                else
                {
                    gtsam::Pose3 pose_from = gtsam::Pose3((close_key_history.end() - 1)->T);
                    gtsam::Pose3 pose_to = gtsam::Pose3((close_key_history.end() - 2)->T);

                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(close_key_history.size(), close_key_history.size() - 1,  pose_from.between(pose_to), noise_odom));
                    isamInitEstimate.insert( close_key_history.size() , pose_to);
                }

                //add SCANCONTEXT
                pcl::PointCloud<pcl::PointXYZ> cloud;
                vector_to_pcl(close_key_history.back().cloud,cloud);
                scManager.makeAndSaveScancontextAndKeys(cloud);
            }
     
            if (close_key_history.size() < 100)
                return;
            int SC_closest_id = -1; 
            float yaw=0; 
            auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
            SC_closest_id=detectResult.first;
            yaw=detectResult.second;

            if(SC_closest_id==-1)
                return;


            pcl::PointCloud<pcl::PointXYZ> cloud_target;
            uint start=(SC_closest_id-20)<0?0:(SC_closest_id-20);
            for (uint i = start; i < start+20; i++)
            {
                    Eigen::Matrix4d T_detal=close_key_history[(uint)SC_closest_id].T.inverse()  *close_key_history[i].T;
                    pcl::PointXYZ point;
                    for(auto iter:close_key_history[i].cloud)
                    {
                        iter=T_detal.block(0,0,3,3)*iter+T_detal.block(0,3,3,1);
                        point.x=iter.x();
                        point.y=iter.y();
                        point.z=iter.z();
                        cloud_target.push_back(point);
                    }
            }
            pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter; 
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.setInputCloud(cloud_target.makeShared());
            downSizeFilter.filter(cloud_target);

            pcl::PointCloud<pcl::PointXYZ> cloud_sources; 

            Eigen::Matrix3d R_init;
            R_init<<cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1;
            Eigen::Matrix3d R_init_inv=R_init.inverse();
            Eigen::Matrix4d T_init=Eigen::Matrix4d::Identity();
            T_init.block(0,0,3,3)=R_init_inv;
            for(auto iter:close_key_history.back().cloud)
            {
                pcl::PointXYZ point;
                iter=R_init_inv*iter;
                point.x=iter.x();
                point.y=iter.y();
                point.z=iter.z();
                cloud_sources.push_back(point);
            }
            downSizeFilter.setInputCloud(cloud_sources.makeShared());
            downSizeFilter.filter(cloud_sources);
            
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
            icp.setInputSource(cloud_sources.makeShared());
            icp.setInputTarget(cloud_target.makeShared());
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);
            pcl::PointCloud<pcl::PointXYZ> unused_result;
            icp.align(unused_result);
            // std::cout<<"icp_error"<<icp.getFitnessScore()<<"  "<<icp.hasConverged()<<std::endl;
            if (icp.hasConverged() == false || icp.getFitnessScore() > max_error_threshold)
                return;
            else
            {
                constraint_new.emplace_back(close_key_history.size()-1,SC_closest_id);
                gtsam::Vector Vector6(6);
                float noiseScore = icp.getFitnessScore() ;
                Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
                constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                Eigen::Matrix4d T= icp.getFinalTransformation().cast<double>()*T_init;
                gtsam::Pose3 pose = gtsam::Pose3(T);   
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(SC_closest_id + 1,close_key_history.size(), pose, constraintNoise));
            }
            
            if (constraint_new.size())
            {
                isam->update(gtSAMgraph,isamInitEstimate);
                isam->update();
                isamInitEstimate.clear();
                gtSAMgraph.resize(0);

                isamCurrentEstimate = isam->calculateEstimate();
                for (uint i =0; i < close_key_history.size(); i++)
                {

                    gtsam::Pose3 oneEstimate;
                    oneEstimate = isamCurrentEstimate.at<gtsam::Pose3>(i+1);

                    close_key_history[i].T = oneEstimate.matrix();
                }


                std::vector<Eigen::Matrix4d> T_new;
                T_new.reserve(close_key_history.size());


                std::shared_ptr<zsy_ndt_lib::ndt_map> new_local_ndtmap(new zsy_ndt_lib::ndt_map(1));
                uint start=(SC_closest_id-25)<0?0:(SC_closest_id-25);
                for (uint i = start; i < start+25; i+=2)
                {
                    new_local_ndtmap->insert_scan(close_key_history[i].cloud, close_key_history[i].T);
                }

                for (auto iter : close_key_history)
                {
                    T_new.push_back(iter.T);
                }

                mutex_loop_closed.lock();
                loop_closed = true;
                T_list_after_optmize=T_new;
                this->new_local_ndtmap=new_local_ndtmap;
                constraint.insert(constraint.end(),constraint_new.begin(),constraint_new.end());
                mutex_loop_closed.unlock();
            }
        }
    }

    void closeloop_detection::do_detection()
    {
            mutex_need_to_detection.lock();
            bool need=need_to_detection;
            need_to_detection=false;
            new_close_key_queue_temp.swap(new_close_key_queue);
            mutex_need_to_detection.unlock();
            if (need)
            {
                while(!new_close_key_queue_temp.empty())
                {
                    close_key_data data=*new_close_key_queue_temp.front();
                    new_close_key_queue_temp.pop();
                    if( !close_key_history.empty())
                   {
                       data.T=close_key_history.back().T*data.T;
                   }
                    close_key_history.emplace_back(std::move(data));



                    if (close_key_history.size() == 1)
                    {
                        gtsam::Pose3 pose = gtsam::Pose3(close_key_history.front().T);
                        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(1, pose, noise_prior));
                        isamInitEstimate.insert(1, pose);
                    }
                    else
                    {
                        gtsam::Pose3 pose_from = gtsam::Pose3((close_key_history.end() - 1)->T);
                        gtsam::Pose3 pose_to = gtsam::Pose3((close_key_history.end() - 2)->T);

                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(close_key_history.size(), close_key_history.size() - 1,  pose_from.between(pose_to), noise_odom));
                        isamInitEstimate.insert( close_key_history.size() , pose_to);
                    }
                }


                if (close_key_history.size() < 100)
                    return;

                pcl::PointCloud<pcl::PointXYZ> position;
                pcl::PointXYZ p;
                for(int i=0;i<((int)close_key_history.size()-95);++i)
                {
                    p.x = close_key_history[i].T(0, 3);
                    p.y = close_key_history[i].T(1, 3);
                    p.z = close_key_history[i].T(2, 3);
                    position.push_back(p);
                }

                pcl::KdTreeFLANN<pcl::PointXYZ> pose_kdtree;
                pose_kdtree.setInputCloud(position.makeShared());
                pcl::PointXYZ position_now;
                position_now.x=close_key_history.back().T(0,3);
                position_now.y=close_key_history.back().T(1,3);
                position_now.z=close_key_history.back().T(2,3);

                std::vector<int> pointSearchIndLoop;
                std::vector<float> pointSearchSqDisLoop;
                pose_kdtree.radiusSearch(position_now, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

                pcl::PointCloud<pcl::PointXYZ> cloud_sources;
                pcl::PointXYZ point;
                for(auto iter:close_key_history.back().cloud)
                {
                    point.x=iter.x();
                    point.y=iter.y();
                    point.z=iter.z();
                    cloud_sources.push_back(point);
                }
                pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
                downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
                downSizeFilter.setInputCloud(cloud_sources.makeShared());
                downSizeFilter.filter(cloud_sources);

                std::vector<std::pair<uint, uint>> constraint_new;  

                pcl::PointCloud<pcl::PointXYZ> cloud_target;
                for (auto iter : pointSearchIndLoop)
                {
                    pcl::PointXYZ point;
                    Eigen::Matrix4d T_detal=close_key_history[pointSearchIndLoop[0]].T.inverse()  *close_key_history[iter].T;

                    for(auto iter2:close_key_history[iter].cloud)
                    {
                        iter2=T_detal.block(0,0,3,3)*iter2+T_detal.block(0,3,3,1);
                        point.x=iter2.x();
                        point.y=iter2.y();
                        point.z=iter2.z();
                        cloud_target.push_back(point);
                    }
                }

                if(cloud_target.size()==0)
                    return;
                downSizeFilter.setInputCloud(cloud_target.makeShared());
                downSizeFilter.filter(cloud_target);

                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
                icp.setInputSource(cloud_sources.makeShared());
                icp.setInputTarget(cloud_target.makeShared());
                icp.setMaxCorrespondenceDistance(100);
                icp.setMaximumIterations(100);
                icp.setTransformationEpsilon(1e-6);
                icp.setEuclideanFitnessEpsilon(1e-6);
                icp.setRANSACIterations(0);
                pcl::PointCloud<pcl::PointXYZ> unused_result;
                icp.align(unused_result);
                // std::cout<<"icp_error"<<icp.getFitnessScore()<<"  "<<icp.hasConverged()<<std::endl;
                if (icp.hasConverged() == false || icp.getFitnessScore() > max_error_threshold)
                    return;
                else
                {
                    constraint_new.emplace_back(close_key_history.size()-1,pointSearchIndLoop[0]);
                    gtsam::Vector Vector6(6);
                    float noiseScore = icp.getFitnessScore() ;
                    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
                    constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                    Eigen::Matrix4d T= icp.getFinalTransformation().cast<double>();
                    gtsam::Pose3 pose = gtsam::Pose3(T);   
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(pointSearchIndLoop[0] + 1,close_key_history.size(), pose, constraintNoise));
                }
                
                if (constraint_new.size())
                {
                    isam->update(gtSAMgraph,isamInitEstimate);
                    isam->update();
                    isamInitEstimate.clear();
                    gtSAMgraph.resize(0);

                    isamCurrentEstimate = isam->calculateEstimate();
                    for (uint i =0; i < close_key_history.size(); i++)
                    {

                        gtsam::Pose3 oneEstimate;
                        oneEstimate = isamCurrentEstimate.at<gtsam::Pose3>(i+1);

                        close_key_history[i].T = oneEstimate.matrix();
                    }

                    std::vector<Eigen::Matrix4d> T_new;
                    T_new.reserve(close_key_history.size());


                    std::shared_ptr<zsy_ndt_lib::ndt_map> new_local_ndtmap(new zsy_ndt_lib::ndt_map(1));
                    //uint start=(pointSearchIndLoop[0]-25)<0?0:pointSearchIndLoop[0]-25;
                    uint start=(close_key_history.size()-25)<0?0:pointSearchIndLoop[0]-25;
                    for (uint i = start; i < close_key_history.size(); i++)
                    {
                        new_local_ndtmap->insert_scan(close_key_history[i].cloud, close_key_history[i].T);
                    }

                    for (auto iter : close_key_history)
                    {
                        T_new.push_back(iter.T);
                    }

                    mutex_loop_closed.lock();
                    loop_closed = true;
                    T_list_after_optmize=T_new;
                    this->new_local_ndtmap=new_local_ndtmap;
                    constraint.insert(constraint.end(),constraint_new.begin(),constraint_new.end());
                    mutex_loop_closed.unlock();
                }
            }
    }

    bool closeloop_detection::find_close(std::vector<Eigen::Matrix4d> &T_list, std::vector<std::pair<uint,uint>> &constraint,std::shared_ptr<zsy_ndt_lib::ndt_map> &new_local_ndtmap)
    {
        mutex_loop_closed.lock();
        bool falg = loop_closed;
        if (falg)
        {
            T_list = T_list_after_optmize;
            constraint = this->constraint;
            loop_closed = false;
            new_local_ndtmap=this->new_local_ndtmap;
        }
        mutex_loop_closed.unlock();
        return falg;
    };

};
