#include "ieskf_slam/modules/ieskf.hpp"
namespace IESKFLIO
{
    IESKF::IESKF(const std::string & config_path,const std::string &prefix)
    :ModuleBase(config_path,prefix,"IESKF")
    {

    }
    
    IESKF::~IESKF()
    {
    }
    void IESKF::predict(const IMU&imu,double dt){
        //去除初始化的值
        imu.acceleration -= state_x.bias_a;
        imu.gyroscope -= state_x.bias_g;
        //状态递推
        auto rotation = state_x.rotation.toRotationMatrix();
        state_x.rotation = Eigen::Quaterniond(X.rotation.toRotationMatrix() * so3Exp((imu.gyroscope) * dt));
        state_x.rotation.normalize();
        state_x.position += state_x.velocity * dt;
        state_x.velocity += (rotation*(imu.acceleration)+X.gravity)*dt;
        Eigen::Matrix<double,18,18> Fx;

        Eigen::Matrix<double,18 ,12>Fw;
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