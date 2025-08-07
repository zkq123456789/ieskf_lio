#include "ieskf_slam/modules/frontEnd.hpp"
namespace IESKFLIO
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    { 
    }
    void FrontEnd::addImu(const IMU &imu){
        imu_deque_.push_back(imu);
        std::cout<<"receive IMU"<<std::endl;
    }
    void FrontEnd::addPointCloud(const PointCloud &pointcloud){
        pointcloud_deque_.push_back(pointcloud);
        std::cout<<"receive Cloud"<<std::endl;
    }
    void FrontEnd::addPose(const Pose&pose){
        pose_deque_.push_back(pose);
        std::cout<<"receive Pose"<<std::endl;
    }
    bool FrontEnd::sync(MeasureGround &measure_ground_){
        measure_ground_.imu_deque.clear();
        measure_ground_.pointcloud = pointcloud_deque_.clean();
        if(pose_deque_.empty()||pointcloud_deque_.empty()){
            return false;
        }

        double imu_begin_time = imu_deque_.front().time_stamp.sec();
        double imu_end_time = imu_deque_.back().time_stamp.sec();
        double cloud_start_time =pointcloud_deque_.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque_.front().cloud_ptr->points.back().offset_time/1e9+cloud_start_time;

        if(imu_begin_time<cloud_end_time){
            pointcloud_deque_.pop_front();
            return false;
        }
        if (imu_end_time<cloud_end_time){
            return false;
        }
        measure_ground_.pointcloud = pointcloud_deque_.front();
        measure_ground_.lidar_begin_time = cloud_start_time;
        measure_ground_.lidar_end_time = cloud_end_time;
        while(!imu_deque_.empty()){
            if(imu_deque_.front().time_stamp.toSec()<cloud_end_time){
                measure_ground_.imu_deque.push_back(imu_deque_.front());
                imu_deque_.pop_front();
            }
            else if(imu_deque_.front().time_stamp.toSec()>cloud_start_time){
                imu_deque_.pop_front();
            }else{
                break;
            }
        }
        return true;

    }
    bool FrontEnd::track(){
        MeasureGround msg;
        // 寻找同一时刻的点云和位姿
        if(sync(msg)){
            if()
        }
        
        
        if(pointcloud_deque_.empty()){
            return false;
        }
        // 滤波
        VoxelFilter vf;
        vf.setLeafSize(0.5,0.5,0.5);
        vf.setInputCloud(pointcloud_deque_.front().cloud_ptr);
        vf.filter(*pointcloud_deque_.front().cloud_ptr);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = pose_deque_.front().rotation.toRotationMatrix().cast<float>();
        trans.block<3,1>(0,3) = pose_deque_.front().position.cast<float>();
        pcl::transformPointCloud(*pointcloud_deque_.front().cloud_ptr,current_pointcloud_,trans);


        pointcloud_deque_.pop_front();
        pose_deque_.pop_front();
        return true;
    }
        const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return current_pointcloud_;
    }
}