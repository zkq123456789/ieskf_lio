
#pragma once
#include "ieskf_slam/modules/frontEnd.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "ros_include/aviaProcess.hpp"
namespace ROSNOETIC
{
    enum LIDAR_TYPE{
        AVIA = 0,

    };
    class IESKFFrontEndWrapper
    {
    private:
        IESKFLIO::FrontEnd::Ptr front_end_ptr;//实现调用IESKFLIO
        ros::Subscriber cloud_subscriber;
        ros::Subscriber imu_subscriber;
        ros::Publisher curr_cloud_pub;
        ros::Publisher path_pub;
        
        std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

        // now status
        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
        void run();
        void publishMsg();
    public:
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
        ~IESKFFrontEndWrapper();

    };
    

    
} // namespace ROSNOETICieskfFrontendNoeticWrapper
