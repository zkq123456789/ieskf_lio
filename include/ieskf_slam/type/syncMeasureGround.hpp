#pragma once
#include "imu.hpp"
#include "pointCloud.hpp"

namespace IESKFLIO {
    struct MeasureGround
    {
        double lidar_begin_time;
        std::deque<IMU> imu_deque;
        PointCloud pointcloud;
        double lidar_end_time;
    };
     
}
