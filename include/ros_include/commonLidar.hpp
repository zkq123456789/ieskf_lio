#pragma once
#include "ieskf_slam/type/baseType.hpp"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNOETIC
{
    class CommonLidarProcessInterface
    {
    public:
        // 根据不同的lidar 转换成统一的cloud
        virtual bool process(const sensor_msgs::PointCloud2 &msg,IESKFLIO::PointCloud&cloud) = 0;
    };
} // namespace ROSNOETIC common_lidar_process_interface