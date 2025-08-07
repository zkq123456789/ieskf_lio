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