#include "ieskf_slam/modules/frontEnd.hpp"
#include <cstddef>
namespace IESKFLIO
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {
        float leaf_size;
        getParam("filter_leaf_size",leaf_size,0.5f);
        voxel_filter.setLeafSize(leaf_size,leaf_size,leaf_size);

        std::vector<double>extrin_v;
        getParam("extrin_r",extrin_v,std::vector<double>());
        extrin_r.setIdentity();
        extrin_t.setZero();
        if(extrin_v.size()==9){
            Eigen::Matrix3d extrin_r33;
            extrin_r33<<extrin_v[0],extrin_v[1],extrin_v[2],extrin_v[3],extrin_v[4],extrin_v[5],extrin_v[6],extrin_v[7],extrin_v[8];
            extrin_r = extrin_r33;
        }else if (extrin_v.size()==3){
            extrin_r.x() = extrin_v[0];
            extrin_r.y() = extrin_v[1];
            extrin_r.z() = extrin_v[2];
            extrin_r.w() = extrin_v[3];
        }
        getParam("extrin_t",extrin_v,std::vector<double>());
        if(extrin_v.size()==3){
            extrin_t<<extrin_v[0],extrin_v[1],extrin_v[2];
        }

        ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
        map_ptr  = std::make_shared<MapManager>(config_file_path,"map");
        fbpropagate_ptr = std::make_shared<frontBackPropagate>();
        
        lio_zh_model_ptr = std::make_shared<LIOZH>();
        ieskf_ptr->ob_zh_ptr =lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(map_ptr->readKDtree(),filter_point_cloud_ptr,map_ptr->getLocalMap());
    }
    void FrontEnd::addImu(const IMU &imu){
        imu_deque_.push_back(imu);
        std::cout<<"receive IMU"<<std::endl;
    }
    void FrontEnd::addPointCloud(const PointCloud &pointcloud){
        pointcloud_deque_.push_back(pointcloud);
        std::cout<<"receive Cloud"<<std::endl;
    }
    
    bool FrontEnd::sync(MeasureGroup &measure_ground_){
        if(imu_deque_.empty()||pointcloud_deque_.empty()){
            // std::cout<<"1111111111 "<<std::endl;
            return false;
        }
        measure_ground_.imu_deque.clear();
        measure_ground_.pointcloud.cloud_ptr->clear();
        
        double imu_begin_time = imu_deque_.front().time_stamp.sec();
        double imu_end_time = imu_deque_.back().time_stamp.sec();
        double cloud_start_time =pointcloud_deque_.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque_.front().cloud_ptr->points.back().offset_time/1e9+cloud_start_time;

        if(imu_begin_time>cloud_end_time){
            pointcloud_deque_.pop_front();
            std::cout<<"imu_begin_time"<<imu_begin_time<<"cloud_end_time"<<imu_end_time<<std::endl;
            std::cout<<"2222222222222222"<<std::endl;
            return false;
        }
        if (imu_end_time<cloud_end_time){
            std::cout<<"33333333333333 "<<std::endl;
            return false;
        }
        measure_ground_.pointcloud = pointcloud_deque_.front();
        pointcloud_deque_.pop_front();
        measure_ground_.lidar_begin_time = cloud_start_time;
        measure_ground_.lidar_end_time = cloud_end_time;
        while(!imu_deque_.empty()){
            if(imu_deque_.front().time_stamp.sec()<cloud_end_time){
                measure_ground_.imu_deque.push_back(imu_deque_.front());
                imu_deque_.pop_front();
            }
            else if(imu_deque_.front().time_stamp.sec()>cloud_start_time){
                imu_deque_.pop_front();
            }else{
                break;
            }
        }
        std::cout<<"sync"<<std::endl;
        return true;

    }
    bool FrontEnd::track(){
        MeasureGroup msg;
        // 寻找同一时刻的点云和位姿 
        if(sync(msg)){
            if(!imu_init){
                map_ptr->reset();
                
                map_ptr->addScan(msg.pointcloud.cloud_ptr,Eigen::Quaterniond::Identity(),Eigen::Vector3d::Zero());
                initState(msg);
                return false;
            }
            std::cout<<msg.imu_deque.size()<<" scale: "<<imu_scale<<std::endl;
            fbpropagate_ptr->propagate(msg,ieskf_ptr);
            voxel_filter.setInputCloud(msg.pointcloud.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);
            ieskf_ptr->update();
            auto x = ieskf_ptr->getState();
            map_ptr->addScan(filter_point_cloud_ptr,x.rotation,x.pos);
            return true;
        }
        return false;
        
        
    }
    void FrontEnd::updatePointCloud(MeasureGroup &measure_ground_){ 
        // 滤波
        VoxelFilter vf;
        vf.setLeafSize(0.5,0.5,0.5);
        vf.setInputCloud(measure_ground_.pointcloud.cloud_ptr);
        vf.filter(*measure_ground_.pointcloud.cloud_ptr);
        Eigen::Matrix4f trans;
        trans.setIdentity();
        auto state = ieskf_ptr->getState();
        trans.block<3,3>(0,0) = state.rotation.toRotationMatrix().cast<float>();
        trans.block<3,1>(0,3) = state.pos.cast<float>();
        pcl::transformPointCloud(*measure_ground_.pointcloud.cloud_ptr,current_pointcloud,trans);
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return *filter_point_cloud_ptr;
    }

    void FrontEnd::initState(MeasureGroup &measure_ground_){
        static int imu_count = 0;
        static Eigen::Vector3d sum_acc{0,0,0};
        static Eigen::Vector3d sum_gyro{0,0,0};
        IESKF &ieskf = *ieskf_ptr;
        auto x = ieskf.getState();
        if(imu_init){
            return;
        }
        for(size_t i=0; i<measure_ground_.imu_deque.size(); i++){
            imu_count++;
            sum_acc += measure_ground_.imu_deque[i].acceleration;
            sum_gyro += measure_ground_.imu_deque[i].gyroscope;
        }
        if(imu_count>5){
            sum_acc = sum_acc / imu_count;
            x.bias_g = sum_gyro / imu_count;
            imu_scale  = GRAVITY / sum_acc.norm();
            x.gravity = - sum_acc / sum_acc.norm() * GRAVITY;
            ieskf.setState(x);
            imu_init = true; 
            fbpropagate_ptr->imu_scale = imu_scale;
            fbpropagate_ptr->last_imu = measure_ground_.imu_deque.back();
        }
        return;
    }

    IESKF::State18 FrontEnd::readState(){
        return ieskf_ptr->getState();
    }
}