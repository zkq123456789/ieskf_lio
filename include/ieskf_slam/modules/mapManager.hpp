

#include <iostream>
#include <Eigen/Core>
#include "ieskf_slam/type/pointCloud.hpp"
#include "ieskf_slam/modules/moduleBase.hpp"
#include "pcl/common/transforms.h"
#include "ieskf_slam/type/baseType.hpp"

namespace IESKFLIO
{
    class MapManager: private ModuleBase {
        private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;
        public:
        MapManager(const std::string &config_file, const std::string &prefix);
        ~MapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &curr_q, const Eigen::Vector3d &curr_t);
        PCLPointCloudConstPtr getLocalMap();
        KDTreeConstPtr readKDtree();
    };
} // namespace IESKFLIO
