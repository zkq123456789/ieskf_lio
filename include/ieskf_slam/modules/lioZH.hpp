#ifndef LIOZH_HPP
#define LIOZH_HPP 
# include "ieskf_slam/modules/ieskf.hpp"
# include "ieskf_slam/modules/front_end.hpp"
# include "ieskf_slam/modules/geomoetry.hpp"


namespace IESKFLIO
{
    class LIOZH :public IESKF::OBSERVEZHInterface
    { 
        private:
            std::shared_ptr<FrontEnd> front_end_ptr;
            const int NEAR_POINTS_NUM = 5;
            //1) point_imu 2)normal_vector 3)distance      
            using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d,double>;
            KDTreeConstPtr global_map_kdtree_ptr;
            PCLPointCloudPtr current_cloud_ptr;
            PCLPointCloudConstPtr local_map_cptr;
        public:
            using Ptr = std::shared_ptr<LIOZH>;
            void prepare(KDTreeConstPtr kdtree_ptr_, PCLPointCloudPtr current_cloud_ptr_,
                        PCLPointCloudConstPtr local_map_cptr_){
                global_map_kdtree_ptr = kdtree_ptr_;
                current_cloud_ptr = current_cloud_ptr_;
                local_map_cptr = local_map_cptr_;
            }
            bool observe(const IESKF::State18&state,Eigen::MatrixXd & Z,Eigen::MatrixXd & H)override{
                std::vector<loss_type> loss_v;
                loss_v.resize(current_cloud_ptr->size());
                std::vector<bool> is_effect_point(current_cloud_ptr->size(),false);
                std::vector<loss_type> loss_real;
                int  vaild_points_num = 0;
                #ifdef MP_EN
                    omp_set_num_threads(MP_PROC_NUM);
                    #pragma omp parallel for
                #endif
                for(size_t i = 0; i < current_cloud_ptr->size(); i++){
                    //将point从imu坐标系转换到世界系
                    Point point_imu = current_cloud_ptr->points[i];
                    Point point_world;
                    point_world = transformPoint(point_imu,state.rotation,state.pos);
                    //邻近搜索
                    std::vector<int> point_ind;//每个点的索引
                    std::vector<float> point_dis;//每个点的距离
                    global_map_kd_tree.nearestKSearch(point_world, NEAR_POINTS_NUM, point_ind, point_dis);
                    //判断是否有临近点构成平面
                    if(point_dis.size() < NEAR_POINTS_NUM || point_dis[NEAR_POINTS_NUM-1] > 5){
                        continue;
                    }
                    std::vector<Point> planar_points;
                    //找到地图中对应的点
                    for(int ni = 0; ni < NEAR_POINTS_NUM; ni++){
                        plannar_points.push_back(local_map_cptr->at(point_ind[ni]));
                    }
                    Eigen::Vector4d pancd;
                    if(planarCheck(planar_points, pabcd, 1)){
                        double distance = point_world.x*pabcd(0)+point_world.y*pabcd(1)+point_world.z*pabcd(2)+pabcd(3);
                        //计算残差
                        loss_type loss;
                        // imu系下点的坐标，用于求H矩阵
                        loss.first = {point_imu.x, point_imu.y, point_imu.z}
                        //平面法向量
                        loss.second = {pabcd.block<3,1>(0,0)};
                        //残差
                        loss.third = distance;
                        if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
                        double s = 1 - 0.9 * fabs(distance) / sqrt(loss.first.norm());
                        if(s > 0.9 ){
                            vaild_points_num++;
                            loss_v[i] = loss;
                            is_effect_point[i] = true;
                        }

                    }
                }
                //记录有效的点
                for (size_t i = 0; i <current_cloud_ptr->size() ; i++)
                {
                    if(is_effect_point[i])loss_real.push_back(loss_v[i]);
                }
                // 根据有效点的数量分配H Z的大小
                vaild_points_num = loss_real.size();
                H = Eigen::MatrixXd::Zero(vaild_points_num, 18); 
                Z.resize(vaild_points_num,1);
                for (int vi = 0; vi < vaild_points_num; vi++)
                {
                    // H 记录导数
                    Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(loss_real[vi].first);
                    H.block<1,3>(vi,0) = dr.transpose();
                    H.block<1,3>(vi,3) = loss_real[vi].second.transpose();
                    // Z记录距离
                    Z(vi,0) = loss_real[vi].thrid;
                }
                return true;
            }

    };
    
} // namespace IESKFLIO

class LIOZH :public IESKF

#endif
