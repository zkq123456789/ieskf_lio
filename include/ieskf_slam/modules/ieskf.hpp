#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ieskf_slam/type/imu.hpp"
#include "ieskf_slam/modules/moduleBase.hpp"

namespace IESKFLIO{
    class IESKF: private ModuleBase
    { 
        public:
            struct State18
            {
                Eigen::Quaterniond rotation;
                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d bias_g;//陀螺仪
                Eigen::Vector3d bias_a;
                Eigen::Vector3d gravity;//单位向量
                State18(){
                    rotation = Eigen::Quaterniond::Identity();
                    pos = Eigen::Vector3d::Zero();
                    vel = Eigen::Vector3d::Zero();
                    bias_g = Eigen::Vector3d::Zero();
                    bias_a = Eigen::Vector3d::Zero();
                    gravity = Eigen::Vector3d::Zero();
                }
            };
            
            IESKF(const std::string & config_path,const std::string &prefix);
            ~IESKF();
        private:
        State18 state_x;
        public:
        void predict(const IMU& imu, double dt);
        bool update();
        const State18& getState() const;
        void setState(State18& state);

    };

}