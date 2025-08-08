#include <Eigen/Core>
#include <Eigen/Dense>

namespace IESKFLIO{
    Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond &q_, const Eigen::Vector3d &t_){
        Eigen::Matrix4d ans;
        ans.setIdentity();
        ans.block<3,3>(0,0) = q_.toRotationMatrix();
        ans.block<3,1>(0,3) = t_;
        return ans;
    }
        //将三维向量so3转换为3×3的反对称矩阵
    Eigen::Matrix3d  skewSymmetric(const Eigen::Vector3d &so3){
        Eigen::Matrix3d so3_skew_sym;
        so3_skew_sym.setZero();
        so3_skew_sym(0,1) = -1*so3(2);
        so3_skew_sym(1,0) = so3(2);
        so3_skew_sym(0,2) = so3(1);
        so3_skew_sym(2,0) = -1*so3(1);
        so3_skew_sym(1,2) = -1*so3(0);
        so3_skew_sym(2,1) = so3(0);
        return so3_skew_sym;
        /***
         *         [0, -z, y]
         *[x,y,z]->[z, 0, -x]
         *         [-y, x, 0]
        */
    }
    //李代数转换成李群
    Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
        Eigen::Matrix3d  SO3;
        double so3_norm = so3.norm();
        if (so3_norm<=0.0000001)
        {
            SO3.setIdentity();
            return SO3;
        }

        Eigen::Matrix3d so3_skew_sym = skewSymmetric(so3);
        SO3 = Eigen::Matrix3d::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym/(so3_norm*so3_norm))*(1-cos(so3_norm));
        return SO3;
        // 罗德里格斯公式核心计算：
        // exp(θ^) = I + (sinθ/θ)·θ^ + ((1-cosθ)/θ²)·(θ^)²
        // 其中：
        // - I：3x3单位矩阵
        // - θ^：李代数向量的反对称矩阵（skew_so3）
        // - θ：旋转角度（so3_norm = ||so3||）
    }
    //伴随矩阵
    Eigen::Matrix3d  A_T(const Eigen::Vector3d& v){
        Eigen::Matrix3d res;
        double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        double norm = std::sqrt(squaredNorm);
        if(norm <1e-11){
            res = Eigen::Matrix3d::Identity();
        }
        else{
            res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) + (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
        }
        return res;
        //Adexp(exp(θ^)) = I + (1-cosθ/θ²)·θ^ + ((1-sinθ/θ)/θ²)·(θ^)²
    }
}