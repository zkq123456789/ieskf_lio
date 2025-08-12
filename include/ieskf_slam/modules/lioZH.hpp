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
                for(size_t i = 0; i < current_cloud_ptr->size(); i++){
                    //将point从imu坐标系转换到世界系
                    Point point_imu = current_cloud_ptr->points[i];
                    Point point_world;
                    point_world = transformPoint(point_imu,state.rotation,state.pos);
                    //邻近搜索
                    std::vector<int> point_ind;
                    std::vector<float> point_dis;
                    global_map_kd_tree.nearestKSearch(point_world, NEAR_POINTS_NUM, point_ind, point_dis);
                    //判断是否有临近点构成平面
                    if(point_dis.size() < NEAR_POINTS_NUM || point_dis[NEAR_POINTS_NUM-1] > 5){
                        continue;
                    }
                }
            }

    };
    
} // namespace IESKFLIO

class LIOZH :public IESKF

#endif
