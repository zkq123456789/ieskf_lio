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
}