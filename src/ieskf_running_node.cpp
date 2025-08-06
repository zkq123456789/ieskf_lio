/*
 * @Descripttion: 
 * @Author: MengKai
 * @version: 
 * @Date: 2023-06-08 21:06:45
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 15:02:22
 */
#include "../include/ros_include/ieskfFrontEndWrapper.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"front_end_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNOETIC::IESKFFrontEndWrapper>front_end_ptr;
    front_end_ptr = std::make_shared<ROSNOETIC::IESKFFrontEndWrapper>(nh);
    return 0;
}