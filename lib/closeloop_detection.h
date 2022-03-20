#ifndef CLOSELOOP_DETECTION
#define CLOSELOOP_DETECTION
#include "common.h"
#include "ndt_map.h"
#include  "scancontext/Scancontext.h"


namespace zsy_ndt_lib
{   
    struct close_key_data
    {
        close_key_data(std::vector<Eigen::Vector3d> &cloud,Eigen::Matrix4d T)
        {
            this->cloud=cloud;
            this->T=T;
        }
        std::vector<Eigen::Vector3d>cloud;
        Eigen::Matrix4d T;
    };


    class closeloop_detection
    {
    private:
        float historyKeyframeSearchRadius=7;
        const double max_error_threshold=0.3;


        std::vector<close_key_data> close_key_history;      //这个变量是do_detection()线程独享的
        std::queue< std::shared_ptr<close_key_data>> new_close_key_queue;
        std::queue< std::shared_ptr<close_key_data>> new_close_key_queue_temp;
        std::vector<Eigen::Matrix4d > T_list_after_optmize;//优化完成后的节点位姿
        std::vector<std::pair<uint,uint>> constraint;               //约束关系 用来可视化的
        std::shared_ptr<zsy_ndt_lib::ndt_map> new_local_ndtmap;      //新的ndt局部子图

        bool loop_closed=false;           //是否进行了闭环优化标志位
        bool need_to_detection=false; //是否需要检测标志位

        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::ISAM2 *isam;
        
        gtsam::noiseModel::Diagonal::shared_ptr noise_odom;
        gtsam::noiseModel::Diagonal::shared_ptr noise_prior;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
        gtsam::Values isamCurrentEstimate;
        gtsam::Values isamInitEstimate;

        SCManager scManager;

    public:
        std::mutex mutex_loop_closed;               //new_close_key和need_to_detection 共用这把线程锁
        std::mutex mutex_need_to_detection;//T_list_after_optmize和need_to_detection共用这个线程锁


        closeloop_detection();
        void do_detection();
        void do_detection_sc();
        void add_a_close_key(std::vector<Eigen::Vector3d>means, Eigen::Matrix4d detal_T);
        bool find_close(std::vector<Eigen::Matrix4d> &T_list,std::vector<std::pair<uint,uint>> &constraint,std::shared_ptr<zsy_ndt_lib::ndt_map> &new_local_ndtmap);

    };
};









#endif