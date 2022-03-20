#include "ndt_map.h"

namespace zsy_ndt_lib
{
    void ndt_cell::pushpoint(const Eigen::Vector3d &point)
    {
        hit_falg=true;
        Eigen::Vector3d point_to_center = point - center;
        if (flag)
        {
            Eigen::Vector3d all = count * mean + point_to_center;
            Eigen::Vector3d mean_new = all / (count + 1);
            Eigen::Matrix3d cov_new = (count-1)*cov/count+(point_to_center-mean)*((point_to_center-mean).transpose())/(count+1);
            mean = mean_new;
            cov = cov_new;
            ++count;
        }
        else
        {
            points.push_back(point_to_center);
            ++count;
            if (count == count_min)
            {
                Eigen::Vector3d all_mean = Eigen::Vector3d::Zero();
                Eigen::Matrix3d all_cov = Eigen::Matrix3d::Zero();
                for (auto iter : points)
                {
                    all_mean += iter;
                    all_cov += iter * (iter.transpose());
                }
                mean = all_mean / count_min;
                cov = all_cov / count_min - mean * (mean.transpose());
                flag = true;
            }
        }
    }

    ndt_cell::ndt_cell(Eigen::Vector3d &center)
    {
        this->center = center;
        flag = false;
        hit_falg=false;
        count = 0;
        certainty=0;
        miss=0;
        miss_this=0;
    };

    void ndt_cell::distribution(Eigen::Vector3d &mean, Eigen::Matrix3d &cov)
    {
        mean = this->mean + center;
        cov = this->cov;
    }

    void ndt_cell::distribution(Eigen::Vector3d &mean, Eigen::Matrix3d &cov,float &certain)
    {
        mean = this->mean + center;
        cov = this->cov;
        if(hit_falg)
        {
            miss+=miss_this;
            miss_this=0;
        }
        else
            miss_this=0;
        hit_falg=false;
        certain=(float)count/(count+miss);
        certainty=certain;
    }


    ndt_map::ndt_map(float length)
    {
        _length = length;
    }


    void ndt_map::count_index(const Eigen::Vector3d &point, int *index)
    {
        index[0] = floor(point.x() / _length);
        index[1] = floor(point.y() / _length);
        index[2] = floor(point.z() / _length);
    }

    void ndt_map::count_index(const Eigen::Vector3d &point,key_xyz &key)
    {
        key[0] = floor(point.x() / _length);
        key[1] = floor(point.y() / _length);
        key[2] = floor(point.z() / _length);
    }


    void ndt_map::insert_scan(const std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d T) 
    {
        count_index(T.block(0,3,3,1), lidar_origin_cell);

        for (auto iter : points)
        {
            Eigen::Vector3d point = T.block(0, 0, 3, 3) * iter + T.block(0, 3, 3, 1); //将该点从雷达坐标系转到世界坐标系下来
            key_xyz key;
            count_index(point,key);
            insert_misses(key);

            if (my_map.count(key) == 0)
            {
                Eigen::Vector3d center(key[0], key[1], key[2]);
                center=center*_length;
                ndt_cell cell(center);
                my_map.insert(std::pair<key_xyz, ndt_cell>(key, cell));
            }
            auto cell_iter = my_map.find(key);
            cell_iter->second.pushpoint(point);
        }
        updata_mean_and_cov();
    }

    void ndt_map::insert_misses(const Eigen::Vector3i end_cell)
    {

            const Eigen::Vector3i delta = end_cell - lidar_origin_cell;
            const int num_samples = delta.cwiseAbs().maxCoeff();

            for (int position = std::max(0, num_samples - 2); position < num_samples; ++position)
            {
                const Eigen::Vector3i miss_cell = lidar_origin_cell + delta * position / num_samples;
                key_xyz key;
                key<<miss_cell[0], miss_cell[1], miss_cell[2];
                if (my_map.count(key) == 0)
                {
                    Eigen::Vector3d center(miss_cell[0], miss_cell[1], miss_cell[2]);
                    center=center*_length;
                    ndt_cell cell(center);
                    my_map.insert(std::pair<key_xyz, ndt_cell>(key, cell));
                }
                if (my_map.count(key) != 0)
                {
                    auto cell_iter = my_map.find(key);
                    cell_iter->second.miss_once();
                }
            }
    }



    void ndt_map::updata_mean_and_cov()  //计算每一个栅格内的数据 并把均值方差数据读取回来
    {
         Eigen::Vector3d mean;
         Eigen::Matrix3d cov;
         float certain;
         mean_list.clear();
         cov_list.clear();
         certainty.clear();

         for(auto iter=my_map.begin();iter!=my_map.end();iter++)
         {
             if(iter->second.flag)
             {
                 iter->second.distribution(mean,cov,certain);
                 mean_list.push_back(mean);
                 cov_list.push_back(cov);
                 certainty.push_back(certain);
             }
         }
    }

     void ndt_map::mean_and_cov( std::vector<Eigen::Vector3d> &mean_list,std::vector<Eigen::Matrix3d> &cov_list )
     {
        mean_list=this->mean_list;
        cov_list=this->cov_list;
     }

     void ndt_map::mean_cov_certainty(std::vector<Eigen::Vector3d> &mean_list,std::vector<Eigen::Matrix3d> &cov_list ,std::vector<float> &certainty)
     {
        mean_list=this->mean_list;
        cov_list=this->cov_list; 
        certainty=this->certainty;
     }



    void ndt_map::init(float length, std::map<key_xyz, ndt_cell> &my_map)
    {
        this->my_map=my_map;
        this->_length=length;
    }

    void ndt_map::cutting(Eigen::Vector3d t) 
    {
        key_xyz key_center;
        count_index(t, key_center);
        for (auto iter = my_map.begin(); iter != my_map.end(); )
        {
            if ((iter->first-key_center).norm()>100)
                my_map.erase(iter++);
            else
                iter++;
        }
    }

    void nearkey(const key_xyz &key,std::vector<key_xyz> &near_key)
    {

       for(int i=-1;i<2;i++)
            for(int j=-1;j<2;j++)
                for(int k=-1;k<2;k++)
                {
                        key_xyz temp=key;
                        temp[0]+=i;
                        temp[1]+=j;
                        temp[2]+=k;
                        near_key.emplace_back(temp);
                }
    }
//2-step filter
    void ndt_map::wash(std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d T)
    {
        std::map<key_xyz, int> danamic_cell;
        std::vector<Eigen::Vector3d> points_new;
         for (auto iter : points)
        {
            Eigen::Vector3d point = T.block(0, 0, 3, 3) * iter + T.block(0, 3, 3, 1); //将该点从雷达坐标系转到世界坐标系下来
            key_xyz key;
            count_index(point,key);

            if (my_map.count(key) != 0)
            {
                auto cell_iter = my_map.find(key);
                if(cell_iter->second.certainty<0.5)
                {
                    std::vector<key_xyz> near_key;
                    nearkey(key,near_key);
                    for(auto  iter:near_key)
                    {
                        if(danamic_cell.count(iter) == 0)
                            danamic_cell.insert(std::make_pair(iter,0));
                    }
                }
            }
        }
         for (auto iter : points)
        {
            Eigen::Vector3d point = T.block(0, 0, 3, 3) * iter + T.block(0, 3, 3, 1); 
            key_xyz key ;
            count_index(point,key);

            if (danamic_cell.count(key) == 0)
            {
                points_new.push_back(iter);
            }
        }
        points.swap(points_new);
    }

    bool mean_cell::mean(Eigen::Vector3d &point)
    {
        point<<0,0,0;
        if(cloud.size()<count_min)
            return false;

        for(auto iter:cloud)
        {
            point+=iter;
        }
        point=point/cloud.size();
        return true;
    }

    void mean_cell::pushpoint(const Eigen::Vector3d &point)
    {
        cloud.push_back(point);
    }

    void mean_scan::count_index(const Eigen::Vector3d &point, key_xyz &key)
    {
        key[0] = floor(point.x() / _length);
        key[1] = floor(point.y() / _length);
        key[2] = floor(point.z() / _length);
    }

    void mean_scan::get_mean(const std::vector<Eigen::Vector3d> &scan_in, std::vector<Eigen::Vector3d> &scan_out)
    {
        my_scan.clear();
        for (auto iter : scan_in)
        {
            key_xyz key;
            count_index(iter,key);
            if (my_scan.count(key) == 0)
            {
                mean_cell cell;
                my_scan.insert(std::pair<key_xyz, mean_cell>(key, cell));
            }
            auto cell_iter = my_scan.find(key);
            cell_iter->second.pushpoint(iter);
        }
        for(auto iter=my_scan.begin();iter!=my_scan.end();iter++)
         {
             Eigen::Vector3d point;
             if(iter->second.mean(point))
             {
                scan_out.emplace_back(point);
             }
         }
    }
}
