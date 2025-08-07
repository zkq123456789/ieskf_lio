#include <Eigen/Core>
#include <Eigen/Geometry>
#include "imu.hpp"

namespace IESKFLIO{
    class IESKF{ 
        public:
            struct State18
            {
                Eigen::Quaterniond rotation;
                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d bias_g;
                Eigen::Vector3d bias_a;
                Eigen::Vector3d gravity;
                State18(){
                    rotation = Eigen::Quaterniond::Identity();
                    pos = Eigen::Vector3d::Zero();
                    vel = Eigen::Vector3d::Zero();
                    bias_g = Eigen::Vector3d::Zero();
                    bias_a = Eigen::Vector3d::Zero();
                    gravity = Eigen::Vector3d::Zero();
                }
            };
            
            IESKF();
            ~IESKF();
        private:
        void predict(const IMU& imu, double dt);
        bool update();
        const State18& getState() const;
        void serState(State18& state);

    };

}