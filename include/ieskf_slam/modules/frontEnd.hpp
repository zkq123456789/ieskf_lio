#ifndef FRONTEND_H
#define FRONTEND_H
#include "ieskf_slam/type/pointCloud.hpp"
#include "ieskf_slam/modules/moduleBase.hpp"
#include "ieskf_slam/type/imu.hpp"
#include "ieskf_slam/type/baseType.hpp"
#include "ieskf_slam/type/pose.hpp"
#include "ieskf_slam/type/syncMeasureGroup.hpp"
#include "ieskf_slam/modules/ieskf.hpp"
#include "ieskf_slam/modules/mapManager.hpp"
#include "ieskf_slam/modules/frontBackPropagate.hpp"
#include <pcl/common/transforms.h>
namespace IESKFLIO{

    class FrontEnd: private ModuleBase
    {
    public:
        using Ptr = std::shared_ptr<FrontEnd>;
    private:
        std::deque<IMU> imu_deque_;
        std::deque<PointCloud> pointcloud_deque_;
        PCLPointCloud current_pointcloud;
        std::shared_ptr<IESKF> ieskf_ptr;
        std::shared_ptr<MapManager> map_ptr;
        std::shared_ptr<frontBackPropagate> fbpropagate_ptr;
        double imu_scale = 1.0;
        bool imu_init = false;
    public:
        FrontEnd(const std::string &config_file_path,const std::string & prefix );
        ~FrontEnd() = default;
        // 需要向前端传入imu和点云数据
        void addImu(const IMU&imu);
        void addPointCloud(const PointCloud&pointcloud);
        
        //同步
        bool sync(MeasureGroup &measure_ground_);
        // 跟踪
        bool track();
        //点云读取
        const PCLPointCloud &readCurrentPointCloud();
        //初始化
        void initState(MeasureGroup &measure_ground_);
        void updatePointCloud(MeasureGroup &measure_ground_);
        //读取状态
        IESKF::State18 readState();
    };

}//namespace IESKFLIO
#endif