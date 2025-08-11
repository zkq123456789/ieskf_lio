#ifndef MODULEBASE_HPP
#define MODULEBASE_HPP


#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>


namespace IESKFLIO{
    /**
     * @brief 模块基类
    */
    class ModuleBase {
    private:
        YAML::Node config_node_;
        std::string name_;
    public:

        /**
         * @param config_file 配置文件
         * @param prefix 模块前缀
         * @param module_name 模块名称
         */
        ModuleBase(const std::string &config_file, const std::string &prefix, const std::string &module_name = "default"){
            name_ = module_name;
            if(config_file != ""){
                try{
                    config_node_ = YAML::LoadFile(config_file);
                }
                catch(YAML::Exception &e){
                    std::cerr << "读取Yaml错误" << e.msg << std::endl;
                }
            }
            if(prefix != "" && config_node_[prefix]){
                config_node_ = config_node_[prefix];
            }
        };
        virtual ~ModuleBase() = default;
        
        /**
         * @brief 获取参数
         * @param param_name 参数名称
         * @param val 参数值
         * @param default_value 默认值
        */
        template<typename T>
        void getParam(const std::string &param_name, T &val, T default_value){
            
            if(config_node_[param_name]){
                val = config_node_[param_name].as<T>();
            }
            else{
                std::cerr << "未找到参数" << param_name << std::endl;
                val = default_value;
            }
            // std::cout<<"name: "<< val <<std::endl; // 开启这一行即可在终端打印参数
        }
    };

}//namespace ISEKFLIO

#endif