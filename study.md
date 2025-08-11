# 学习记录
## 代码位置+内容

### include/ieskf_slam/type/Point.hpp

EIGEN_ALIGN16
要求该结构体按 16 字节对齐，适配 Eigen 库的向量运算（提高性能，避免内存对齐错误）。
PCL_ADD_POINT4D
PCL 的宏，自动添加点的三维坐标字段：
float x, y, z：三维坐标。
隐含一个float data[4]数组（兼容 PCL 内部存储，data[0]=x, data[1]=y, data[2]=z, data[3]=1.0）。
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
Eigen 库的宏，确保该结构体在动态分配（如new）时也能满足 16 字节对齐要求。
POINT_CLOUD_REGISTER_POINT_STRUCT
PCL的核心宏，用于将自定义点类型注册到 PCL 系统中，使其能被 PCL 的算法（如滤波、分割、可视化）识别和处理。
#pragma once
只编译一次？

### include/ieskf_slam/type/PointCloud.hpp

using// 给复杂类型定义别名（如模板类型）
using PCLPointCloud = pcl::PointCloud<Point>;  // 你的代码中用到的
PCLPointCloud cloud;  // 等价于 pcl::PointCloud<Point> cloud;

td::shared_ptr
是 C++ 的共享智能指针，会自动跟踪对象的引用计数：
当没有指针引用对象时，自动释放内存，避免内存泄漏。

### include/globalDefine.hpp
通过在CMakeists.txt中添加add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")将项目路径设置为全局宏

### include/ros_include/aviaProcess.hpp
继承的父类commonlidar，实际使用时定义父类指针，根据雷达类型调用子类的函数

### include/ieskf_slam/modules/math.hpp
.trace()返回矩阵的迹，对角线元素之和

### src/ieskf_slam/ieskf.cpp
size_t非负整数 头文件<cstddef>
.normalize() 归一化处理 单位向量

J:的作用是来更新每一次迭代的方差。

Z：残差矩阵，特征点到特征平面的距离，误差状态越接近真值，位置越确定，残差越小。

H：观测方程的雅克比矩阵，第N行对应Z中第N个残差，每一行中的内容分别是残差也就是点到平面的距离关于状态量的导数。

J_inv;误差状态到系统状态的转换雅可比的逆矩阵，主要处理旋转误差的非线性特性,将误差状态线性化

李群李代数 伴随矩阵？

![alt text](FxFw.png)
![alt text](状态更新方程.png)
### src/ieskf_slam/frontEnd.cpp
.normal() 求模

### src/ieskf_slam/mapManager.cpp
.cast<float>()将矩阵转换成float类型

### src/ieskf_slam/frontBackPropagate.cpp
auto &&begin = *it_imu;
auto &&end = *(it_imu+1);对于左值右值还有疑问
STL 容器的 end() 迭代器不指向任何实际元素

### include/ieskf_slam/modules/moduleBase.hpp
报错terminate called after throwing an instance of 'YAML::TypedBadConversion<double>'
  what():  bad conversion
void getParam(const std::string &param_name, T &val, T default_value)设置成返回 T，未设置return，

## 待解决的问题
在 C++ 项目中设置打印参数等级（如调试级、信息级、警告级、错误级），可以通过日志等级控制实现。以下是几种常用方案，结合你的项目代码结构给出推荐实现方式：

```C++
#pragma once
#include <iostream>
#include <string>

// 定义日志等级（数字越大等级越高，输出越严格）
enum LogLevel {
    DEBUG = 0,   // 调试信息（最详细）
    INFO = 1,    // 普通信息
    WARNING = 2, // 警告
    ERROR = 3,   // 错误
    FATAL = 4    // 致命错误（程序可能崩溃）
};

// 全局日志等级（可在配置文件中读取或代码中修改）
extern LogLevel g_log_level;

// 日志输出宏（带等级判断）
#define LOG(level, msg) \
    do { \
        if (level >= g_log_level) { \
            std::cerr << "[" #level "] " << msg << std::endl; \
        } \
    } while(0)

// 简化常用日志宏
#define LOG_DEBUG(msg) LOG(DEBUG, msg)
#define LOG_INFO(msg)  LOG(INFO, msg)
#define LOG_WARN(msg)  LOG(WARNING, msg)
#define LOG_ERROR(msg) LOG(ERROR, msg)
#define LOG_FATAL(msg) LOG(FATAL, msg)
```
实际使用
```C++
#include "ieskf_slam/common/log.hpp"

// 读取参数时打印（调试级）
getParam("cov_gyroscope", cov_gyroscope, 0.1);
LOG_DEBUG("cov_gyroscope: " << cov_gyroscope);  // 仅当等级为DEBUG时输出

// 关键流程信息（信息级）
LOG_INFO("IESKF initialized with Q matrix: " << Q);

// 错误信息（错误级）
if (measure_ground_.imu_deque.empty()) {
    LOG_ERROR("No IMU data for prediction!");
}
```