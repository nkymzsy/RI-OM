#include "scan_to_map.h"
#include <math.h>
#include "ctime"
#include <pcl/io/ply_io.h>


namespace zsy_ndt_lib
{
    struct ndt_factor
    {
        ndt_factor(Eigen::Vector3d point, Eigen::Vector3d point_nearest, Eigen::Matrix3d cell_cov,double s)
            : _point(point), _point_nearest(point_nearest),_s(s)
        {
            //关键一步 计算协方差矩阵的逆
            Eigen::Matrix3d trick=(cell_cov+1e-6*Eigen::Matrix3d::Identity()).inverse();
            _cell_cov_inv=trick/trick.maxCoeff();

         }

        template <typename T>
        bool operator()(const T *angle, const T *t, T *residual) const
        {
            T A = t[1] - T(_point_nearest[1]) +
                  T(_point[0]) * (ceres::cos(angle[0]) * ceres::sin(angle[2]) + ceres::sin(angle[0]) * ceres::sin(angle[1]) * ceres::cos(angle[2])) +
                  T(_point[1]) * (ceres::cos(angle[0]) * ceres::cos(angle[2]) - ceres::sin(angle[0]) * ceres::sin(angle[1]) * ceres::sin(angle[2])) -
                  T(_point[2]) * ceres::sin(angle[0]) * ceres::cos(angle[1]);

            T B = t[2] - T(_point_nearest[2]) +
                  T(_point[0]) * (ceres::sin(angle[0]) * ceres::sin(angle[2]) - ceres::cos(angle[0]) * ceres::sin(angle[1]) * ceres::cos(angle[2])) +
                  T(_point[1]) * (ceres::sin(angle[0]) * ceres::cos(angle[2]) + ceres::cos(angle[0]) * ceres::sin(angle[1]) * ceres::sin(angle[2])) +
                  T(_point[2]) * ceres::cos(angle[0]) * ceres::cos(angle[1]);

            T C = t[0] - T(_point_nearest[0]) +
                  T(_point[0]) * ceres::cos(angle[1]) * ceres::cos(angle[2]) -
                  T(_point[1]) * ceres::cos(angle[1]) * ceres::sin(angle[2]) +
                  T(_point[2]) * ceres::sin(angle[1]);

            T D = ((T(_cell_cov_inv(2, 2)) * B + T(_cell_cov_inv(1, 2)) * A + T(_cell_cov_inv(0, 2)) * C) * B +
                   (T(_cell_cov_inv(1, 2)) * B + T(_cell_cov_inv(1, 1)) * A + T(_cell_cov_inv(0, 1)) * C) * A +
                   (T(_cell_cov_inv(0, 2)) * B + T(_cell_cov_inv(0, 1)) * A + T(_cell_cov_inv(0, 0)) * C) * C);

            residual[0] = T(_s)*D;

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d point, const Eigen::Vector3d point_nearest, const Eigen::Matrix3d cell_cov,double s)
        {
            return (new ceres::AutoDiffCostFunction<ndt_factor, 1, 3, 3>(
                new ndt_factor(point, point_nearest, cell_cov,s)));
        }

        Eigen::Vector3d _point;
        Eigen::Vector3d _point_nearest;
        Eigen::Matrix3d _cell_cov_inv;
        double _cell_cov_determinant_sqrt;
        double _s;
    };

    //参数1 sub_map：当前子图
    //参数2 scan：待匹配激光帧
    //参数3 T:两帧之间的优化初始值
    scan_to_map::scan_to_map(ndt_map *scan_old,  std::vector<Eigen::Vector3d> &scan,Eigen::Matrix4d &T)
    {
        //记录老的一帧分布情况
        scan_old->mean_cov_certainty(mean_old,cov_old,certainty_old);

        //将地图子图数据建立kdtree
        maketree(kdtree, mean_old);

        //记录scan_map的栅格重心位置
        mean_scan=scan;


        //将旋转矩阵转换成欧拉角和平移向量赋给优化变量初值
        Eigen::Vector3d t_init(T.block(0, 3, 3, 1));
        Eigen::Matrix3d R(T.block(0, 0, 3, 3));
        Eigen::Vector3d angle_Eigen(R.eulerAngles(0, 1, 2));
        para_angle[0] = angle_Eigen[0];
        para_angle[1] = angle_Eigen[1];
        para_angle[2] = angle_Eigen[2];
        para_t[0] = t_init[0];
        para_t[1] = t_init[1];
        para_t[2] = t_init[2];

    }

    
    void scan_to_map::maketree(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, std::vector<Eigen::Vector3d> &mean_sub)
    {
        for (auto iter : mean_sub)
        {
            pcl::PointXYZ p(iter.x(),iter.y(),iter.z());
            mean_sub_pcl.push_back(p);
        }
        kdtree.setInputCloud(mean_sub_pcl.makeShared());
    }

    void scan_to_map::optimize_once()
    {
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_angle, 3);
        problem.AddParameterBlock(para_t, 3);

        Eigen::Vector3d point_temp;

        Eigen::Matrix3d R_init;
        R_init = Eigen::AngleAxisd(para_angle[0], Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(para_angle[1], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(para_angle[2], Eigen::Vector3d::UnitZ());

        Eigen::Vector3d t_init(para_t[0], para_t[1], para_t[2]);

        std::vector<int> index(1);
        std::vector<float> pointNKNSquaredDistance(1);

        ceres::LossFunction *loss_function = new ceres::HuberLoss(10);
        //ceres::LossFunction *loss_function = NULL;
        
        for (uint i=0;i<mean_scan.size();i++)
        {
            point_temp = R_init * mean_scan[i] + t_init;

            //在kdtree中找这个点的最近邻点
            pcl::PointXYZ p(point_temp.x() , point_temp.y() , point_temp.z());
            kdtree.nearestKSearch(p, 1, index, pointNKNSquaredDistance);
            
            if(pointNKNSquaredDistance[0]>max_distence )
                continue;

           double s=this->certainty_old[index[0]];

            ceres::CostFunction *cost_function = ndt_factor::Create(mean_scan[i], mean_old[index[0]], cov_old[index[0]],s);
            problem.AddResidualBlock(cost_function, loss_function, para_angle, para_t);

        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 1;
        ceres::Solver::Summary summary;

       
        ceres::Solve(options, &problem, &summary);

    }


    bool judgle(double *a1,double *a2,double *t1,double *t2)
    {
        if(((t1[0]-t2[0])*(t1[0]-t2[0])+ (t1[1]-t2[1])*(t1[1]-t2[1])+ (t1[2]-t2[2])*(t1[2]-t2[2]))>0.001*0.001)
            return false;
        if( fabs(a1[0]-a2[0])>0.5/180*3.14 ||  fabs(a1[1]-a2[1])>0.5/180*3.14 || fabs(a1[2]-a2[2])>0.1/180*3.14)
            return false;
        return true;
    }
    

    void scan_to_map::optimize(uint times)
    {
        uint i;
        para_angle_old[0]=para_angle[0];
        para_angle_old[1]=para_angle[1];
        para_angle_old[2]=para_angle[2];
        para_t_old[0]=para_t[0];
        para_t_old[1]=para_t[1];
        para_t_old[2]=para_t[2];

        for(i=0;i<times;i++)
        {
            optimize_once();
            if(judgle(para_angle_old,para_angle,para_t_old,para_t))
                break;
            para_angle_old[0]=para_angle[0];
            para_angle_old[1]=para_angle[1];
            para_angle_old[2]=para_angle[2];
            para_t_old[0]=para_t[0];
            para_t_old[1]=para_t[1];
            para_t_old[2]=para_t[2];
        }
    }


    void scan_to_map::result(Eigen::Matrix4d &T)
    {
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(para_angle[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(para_angle[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(para_angle[2], Eigen::Vector3d::UnitZ());
        T.block(0, 0, 3, 3) = R;
        T.block(0, 3, 3, 1) << para_t[0], para_t[1], para_t[2];
    }

}
