#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "common_include.h"

namespace myslam {

    class Config {
    private:
        static std::shared_ptr<Config> config_; // 为所有class所共享
        cv::FileStorage file_;

        Config() {}

    public:
        ~Config();

        static bool setParameterFile(const std::string &filename);

        template<typename T>
        static T get(const std::string &key){
            return T(Config::config_->file_[key]);
        }

    };

} // namespace myslam


#endif //MYSLAM_CONFIG_H