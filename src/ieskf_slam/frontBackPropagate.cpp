
#include "ieskf_slam/modules/frontBackPropagate.hpp"


namespace IESKFLIO { 

    frontBackPropagate::frontBackPropagate(){
        
    }
    frontBackPropagate::~frontBackPropagate(){
        
    }
    void frontBackPropagate::propagate(MeasureGroup&mg_,IESKF::Ptr ieskf_ptr_){
        std::sort(mg_.pointcloud.cloud_ptr->points.begin(),
                  mg_.pointcloud.cloud_ptr->points.end(),
                  [](Point x,Point y){return x.offset_time < y.offset_time;});
        mg_.imu_deque.push_front(last_imu);
        IMU in;
        for(auto it_imu = mg_.imu_deque.begin();it_imu < (mg_.imu_deque.end() - 1);it_imu++){
            auto &&begin = *it_imu;
            auto &&end = *(it_imu+1);
            auto angvel_avr = (begin.gyroscope + end.gyroscope) / 2.0;
            auto acc_avr = (begin.acceleration + end.acceleration) * imu_scale / 2.0;
            double dt = end.time_stamp.sec() - begin.time_stamp.sec();
            in.acceleration = acc_avr;
            in.gyroscope = angvel_avr;
            ieskf_ptr_->predict(in,dt);
        }
        double dt = mg_.lidar_end_time - mg_.imu_deque.back().time_stamp.sec();
        last_imu = mg_.imu_deque.back();
        ieskf_ptr_->predict(in,dt);
        last_imu.time_stamp.fromSec(mg_.lidar_end_time);
    }
    

}
