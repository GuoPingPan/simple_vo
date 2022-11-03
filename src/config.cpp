#include "../include/myslam/config.h"


namespace myslam {

    bool Config::setParameterFile(const std::string &filename) {
        if(config_ == nullptr)
            config_ = std::shared_ptr<Config>(new Config);
        // 本质上还是用的opencv来处理参数文件
        // https://blog.csdn.net/ktigerhero3/article/details/77523094
        config_->file_ = cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        if(config_->file_.isOpened() == false){
            // 采用glog
            LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_->file_.release();
            return false;
        }
        return true;
    }

    Config::~Config() {
        if(file_.isOpened())
            file_.release();
    }

    // 静态成员变量需要类外初始化，在运行过程中只有一个类管理全局配置文件
    std::shared_ptr<Config> Config::config_ = nullptr;

} // namespace myslam