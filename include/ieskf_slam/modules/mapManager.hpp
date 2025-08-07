#include <iostream>
#include <Eigen/Core>
#include "pointCloud.hpp"

namespace IESKFLIO
{
    class MapManager{
        private:
        public:
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &curr_q, const Eigen::Vector3d &curr_t);
        PCLPointCloudPtr getMap();
    }
} // namespace IESKFLIO
class MapManager