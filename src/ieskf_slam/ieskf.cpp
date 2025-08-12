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
        static int cnt_ = 0;
        auto state_x_k = state_x;
        auto state_x_k_last = state_x;

        Eigen::MatrixXd K;//卡尔曼增益
        Eigen::MatrixXd H;//观测模型的雅可比矩阵
        Eigen::Matrix<double, 18, 18> P_err;//误差协方差矩阵

        bool converge = false;

        for(int i = 0; i < iter_times; i++){
            //计算误差状态
            Eigen::Matrix<double, 18, 1> x_error = getStateError(state_x_k, state_x);//state_x_k - state_x
            Eigen::Matrix<double, 18, 18> J_k;//误差状态到系统状态的转换雅可比的逆矩阵，主要处理旋转误差的非线性特性
            J_k.setIdentity();
            J_k.block<3,3>(0,0) = A_T(x_error.block<3,1>(0,0));
            
            P_err = J_k * P * J_k.transpose();

            Eigen::MatrixXd z_k;//观测模型
            Eigen::MatrixXd R_inv;

            
            ob_zh_ptr->observe(state_x_k, z_k, H);
            Eigen::MatrixXd H_t = H.transpose();
            // R 直接写死0.001
            double R = 0.001;
            K = (H_t * (1/R) * H+P_err.inverse()).inverse() * H_t * (1/R);

            Eigen::MatrixXd left = -1*K*z_k;
            Eigen::MatrixXd right = -1*(Eigen::Matrix<double,18,18>::Identity()-K * H) * J_k.inverse() * x_error; 
            Eigen::MatrixXd update_x = left+right;

            // 收敛判断
            converge =true;
            for ( int idx = 0; idx < 18; idx++)
            {
                if (update_x(idx,0)>0.001)
                {
                    converge = false;
                    break;
                }
            
            }
            // x更新方程
            state_x_k.rotation = state_x_k.rotation.toRotationMatrix()*so3Exp(update_x.block<3,1>(0,0));
            state_x_k.rotation.normalize();
            state_x_k.pos = state_x_k.pos+update_x.block<3,1>(3,0);
            state_x_k.vel = state_x_k.vel+update_x.block<3,1>(6,0);
            state_x_k.bias_g = state_x_k.bias_g+update_x.block<3,1>(9,0);
            state_x_k.bias_a = state_x_k.bias_a+update_x.block<3,1>(12,0);
            state_x_k.gravity = state_x_k.gravity+update_x.block<3,1>(15,0);
            if(converge){
                break;
            }

        }
        cnt_++;
        state_x = state_x_k;
        P = (Eigen::Matrix<double,18,18>::Identity()-K * H) * P_err;
        return converge;
    }
    Eigen::Matrix<double,18,1> IESKF::getStateError(const State18 &s1, const  State18 &s2){
            Eigen::Matrix<double,18,1> es;
            es.setZero();
            es.block<3,1>(0,0) = SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
            es.block<3,1>(3,0) = s1.pos - s2.pos;
            es.block<3,1>(6,0) = s1.vel - s2.vel;
            es.block<3,1>(9,0) = s1.bias_g - s2.bias_g;
            es.block<3,1>(12,0) = s1.bias_a - s2.bias_a;
            es.block<3,1>(15,0) = s1.gravity - s2.gravity;
            return es;
    }
    const IESKF::State18& IESKF::getState() const{
        return state_x;
    }
    void IESKF::setState(State18& state){
        state_x = state;
    }
} // namespace IESKFLIO