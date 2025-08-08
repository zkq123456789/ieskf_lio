#pragma once
#include "ieskf_slam/type/syncMeasureGroup.hpp"
#include "ieskf_slam/modules/ieskf.hpp"

namespace IESKFLIO { 
    class frontBackPropagate{
    private:
    public:
        double imu_scale;
        IMU last_imu;
        frontBackPropagate();
        ~frontBackPropagate();
        void propagate(MeasureGroup&mg_, IESKF::Ptr ieskf_ptr_);
    };

}
