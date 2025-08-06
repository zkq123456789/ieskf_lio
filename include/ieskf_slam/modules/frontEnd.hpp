#ifndef FRONTEND_H
#define FRONTEND_H
#include "ieskf_slam/type/pointCloud.hpp"
#include "ieskf_slam/modules/moduleBase.hpp"
#include "ieskf_slam/type/imu.hpp"
#include "ieskf_slam/type/baseType.hpp"
#include "ieskf_slam/type/pose.hpp"
#include <pcl/common/transforms.h>
namespace IESKFLIO{

    class FrontEnd: private ModuleBase
    {
    public:
        using Ptr = std::shared_ptr<FrontEnd>;
    private:
        std::deque<IMU> imu_deque_;
        std::deque<PointCloud> pointcloud_deque_;
        std::deque<Pose> pose_deque_; 
        PCLPointCloud current_pointcloud_;
    public:
        FrontEnd(const std::string &config_file_path,const std::string & prefix );
        ~FrontEnd() = default;
        // 需要向前端传入imu和点云数据
        void addImu(const IMU&imu);
        void addPointCloud(const PointCloud&pointcloud);
        void addPose(const Pose&pose);
        // 跟踪
        bool track();
        // 点云读取
        const PCLPointCloud &readCurrentPointCloud();    
    };

}//namespace IESKFLIO
#endif