#pragma once
#include "ieskf_slam/type/pointCloud.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace IESKFLIO
{
    // 体素滤波器
    using VoxelFilter = pcl::VoxelGrid<Point>;
    // KDTree
    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    // 定义重力常量
    const double GRAVITY = 9.81;
} // namespace IESKFLIO