#pragma once
#include "imu.hpp"
#include "pointCloud.hpp"
#include <deque>

namespace IESKFLIO {
    struct MeasureGroup
    {
        double lidar_begin_time;
        std::deque<IMU> imu_deque;
        PointCloud pointcloud;
        double lidar_end_time;
    };
     
}
