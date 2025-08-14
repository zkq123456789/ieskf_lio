

#include "ieskf_slam/modules/mapManager.hpp"
#include "pcl/common/transforms.h"
#include "ieskf_slam/modules/math.hpp"
namespace IESKFLIO
{
    MapManager::MapManager(const std::string &config_file_path,const std::string & prefix )
    :ModuleBase(config_file_path,prefix,"MapManager")
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
    }
    
    MapManager::~MapManager()
    {
    }
    void MapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,const Eigen::Vector3d &pos_t){
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan,scan,compositeTransform(att_q,pos_t).cast<float>());
        VoxelFilter filter;
        filter.setLeafSize(0.5,0.5,0.5);

        *local_map_ptr+=scan;
        filter.setInputCloud(local_map_ptr);
        filter.filter(*local_map_ptr);
        kdtree_ptr->setInputCloud(local_map_ptr);
    }
    void MapManager::reset(){
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr MapManager::getLocalMap(){
        return local_map_ptr;
    }
    KDTreeConstPtr MapManager::readKDtree(){
        return kdtree_ptr;
    }
} // namespace IESKFLIO