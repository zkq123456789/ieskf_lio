#pragma once
#include <Eigen/Dense>
#include "timeStamp.hpp"
namespace IESKFLIO
{
    struct Pose
    {
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
} // namespace IESKFLIO