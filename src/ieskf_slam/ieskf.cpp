#include "ieskf_slam/modules/ieskf.hpp"
#include <filesystem>
namespace IESKFLIO
{
    IESKF::IESKF(const std::string & config_path,const std::string &prefix)
    :ModuleBase(config_path,prefix,"IESKF")
    {
 
        P.setIdentity();
        P(9,9)   = P(10,10) = P(11,11) = 0.0001;
        P(12,12) = P(13,13) = P(14,14) = 0.001;
        P(15,15) = P(16,16) = P(17,17) = 0.00001; 
        double cov_gyroscope,cov_acceleration,cov_bias_acceleration,cov_bias_gyroscope;
        getParam("cov_gyroscope", cov_gyroscope, 0.1);
        getParam("cov_acceleration",cov_acceleration,0.1);
        getParam("cov_bias_acceleration",cov_bias_acceleration,0.1);
        getParam("cov_bias_gyroscope",cov_bias_gyroscope,0.1);
        Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        state_x.bias_g.setZero();
        state_x.bias_a.setZero();
        state_x.gravity.setZero();
        state_x.pos.setZero();
        state_x.rotation.setIdentity();
        state_x.vel.setZero();
    }
    IESKF::~IESKF()
    {
    }
    void IESKF::predict(IMU imu,double dt){
        //去除初始化的值
        imu.acceleration -= state_x.bias_a;
        imu.gyroscope -= state_x.bias_g;
        //状态递推
        auto rotation = state_x.rotation.toRotationMatrix();
        state_x.rotation = Eigen::Quaterniond(state_x.rotation.toRotationMatrix() * so3Exp((imu.gyroscope) * dt));
        state_x.rotation.normalize();
        state_x.pos += state_x.vel * dt;
        state_x.vel += (rotation*(imu.acceleration)+state_x.gravity)*dt;
        Eigen::Matrix<double,18,18> Fx;
        Eigen::Matrix<double,18,12> Fw;
        Fw.setZero();
        Fx.setIdentity();
        //方差计算
        Fx.block<3,3>(0,0) = so3Exp(-1 * imu.gyroscope * dt);
        Fx.block<3,3>(0,9) = -1 * A_T(-imu.gyroscope*dt)*dt;
        Fx.block<3,3>(3,6) =  Eigen::Matrix3d::Identity()*dt;
        Fx.block<3,3>(6,0) = rotation*skewSymmetric(imu.acceleration)*dt*(-1);
        Fx.block<3,3>(6,12) = rotation*dt*(-1);
        Fx.block<3,3>(6,15) = Eigen::Matrix3d::Identity()*dt;
        Fw.block<3,3>(0,0) = -1*A_T(-imu.gyroscope*dt)*dt;
        Fw.block<3,3>(6,3) = -1*rotation*dt;
        Fw.block<3,3>(9,6) = Fw.block<3,3>(12,9) = Eigen::Matrix3d::Identity()*dt;
        P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose(); 
        
    }
    bool IESKF::update(){
        return false;
    }
    const IESKF::State18& IESKF::getState() const{
        return state_x;
    }
    void IESKF::setState(State18& state){
        state_x = state;
    }
} // namespace IESKFLIO