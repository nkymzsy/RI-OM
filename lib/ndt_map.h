#ifndef NDT_MAP
#define NDT_MAP

#include "common.h"
#include "iostream"

namespace zsy_ndt_lib
{
    class my_key : public Eigen::Vector3i
    {
        friend bool operator<(const my_key &key1, const my_key &key2)
        {
            if (key1[0]!= key2[0])
                return key1[0]< key2[0];

            if (key1[1]!= key2[1])
                return key1[1]< key2[1];

            if (key1[2]!= key2[2])
                return key1[2]< key2[2];
            return false;
        }
    };
    typedef my_key key_xyz;
    class ndt_cell
    {
    private:
        const uint count_min = 5;             //方格内计算统计特征的最少样本点数防止退化

        std::vector<Eigen::Vector3d> points; //未初始化成功时记录点
        Eigen::Vector3d mean;                //均值 栅格坐标系下的
        Eigen::Matrix3d cov;                     //协方差
        Eigen::Vector3d center;              //当前方格原点坐标
        uint count;                                         //当前方格点数
        uint miss;                                          //这个栅格被miss的次数

        uint miss_this;                                //这个栅格最新miss的次数
        bool hit_falg;

    public:
        bool flag; //此方格是否初始化成功
        float certainty;         //这个栅格的确定度

        ndt_cell(Eigen::Vector3d &center);
        void pushpoint(const Eigen::Vector3d &point);
        void miss_once(){++miss_this;}
        void distribution(Eigen::Vector3d &mean, Eigen::Matrix3d &cov);
        void distribution(Eigen::Vector3d &mean, Eigen::Matrix3d &cov,float &certain);
    };


    class ndt_map
    {
    private:
        float _length;                      //栅格边长
        std::map<key_xyz, ndt_cell> my_map; //栅格地图

        std::vector<Eigen::Vector3d> mean_list;
        std::vector<Eigen::Matrix3d> cov_list;
        std::vector<float> certainty;

        key_xyz  lidar_origin_cell;                    //插入点云时原点的坐标

        void count_index(const Eigen::Vector3d &point, int *index); //计算一个点所属栅格
        void count_index(const Eigen::Vector3d &point,key_xyz &key); //计算一个点所属栅格
        void updata_mean_and_cov();
        void insert_misses( const Eigen::Vector3i end_cell);

        std::map<key_xyz, int> miss_cell;

    public:
        ndt_map(float length);
        void init(float length, std::map<key_xyz, ndt_cell> &my_map);
        void insert_scan(const std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d T=Eigen::Matrix4d::Identity() );
        void mean_and_cov( std::vector<Eigen::Vector3d> &mean,std::vector<Eigen::Matrix3d> &cov );
        void mean_cov_certainty(std::vector<Eigen::Vector3d> &mean,std::vector<Eigen::Matrix3d> &cov ,std::vector<float> &certainty);
        float length_size(){return _length;}
        void cutting(Eigen::Vector3d t);
        void wash(std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d T);
    };

    class mean_cell
    {
    private:
        const uint count_min = 5;         
        std::vector<Eigen::Vector3d> cloud; 
    public:
        bool mean(Eigen::Vector3d &point);
        void pushpoint(const Eigen::Vector3d &point);
    };

    class mean_scan
    {
        private:
            float _length;                     
            std::map<key_xyz, mean_cell> my_scan; 
            void count_index(const Eigen::Vector3d &point,key_xyz &key); //计算一个点所属栅格
        public:
            mean_scan(float length){    _length = length;   };
            void get_mean(const std::vector<Eigen::Vector3d> &scan_in, std::vector<Eigen::Vector3d> &scan_out);
    };


}

#endif