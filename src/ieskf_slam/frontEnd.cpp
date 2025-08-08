#include "ieskf_slam/modules/frontEnd.hpp"
#include <cstddef>
namespace IESKFLIO
{
    FrontEnd::FrontEnd(const std::string &config_file_path,const std::string & prefix ):ModuleBase(config_file_path,prefix,"Front End Module")
    {
        ieskf_ptr = std::make_shared<IESKF>(config_file_path,"ieskf");
        map_ptr  = std::make_shared<MapManager>(config_file_path,"map");
        fbpropagate_ptr = std::make_shared<frontBackPropagate>();
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
    bool FrontEnd::sync(MeasureGroup &measure_ground_){
        measure_ground_.imu_deque.clear();
        measure_ground_.pointcloud.cloud_ptr->clear();
        if(imu_deque_.empty()||pointcloud_deque_.empty()){
            std::cout<<"1111111111 "<<std::endl;
            return false;
        }

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
            return true;
        }
        return false;
        
        
    }
    const PCLPointCloud& FrontEnd::readCurrentPointCloud(){
        return current_pointcloud_;
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
}