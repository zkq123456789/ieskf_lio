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
    bool FrontEnd::track(){
        if(pose_deque_.empty()||pointcloud_deque_.empty()){
            return false;
        }
        // 寻找同一时刻的点云和位姿
        
        while (!pose_deque_.empty()&&pose_deque_.front().time_stamp.nsec()<pointcloud_deque_.front().time_stamp.nsec())
        {   
            std::cout<<"1"<<std::endl;
            pose_deque_.pop_front();
        }
        if(pose_deque_.empty()){
            return false;
        }
        while (!pointcloud_deque_.empty()&&pointcloud_deque_.front().time_stamp.nsec()<pose_deque_.front().time_stamp.nsec())
        {
            std::cout<<"2"<<std::endl;
            pointcloud_deque_.pop_front();
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