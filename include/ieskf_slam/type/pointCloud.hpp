#pragma once
#include "ieskf_slam/type/point.hpp"
#include "ieskf_slam/type/timeStamp.hpp"
namespace IESKFLIO
{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;//pcl::PointCloud<Point>的指针
        PointCloud(){
            cloud_ptr = pcl::make_shared<PCLPointCloud>();
        }
    };
} // namespace IESKFLIO