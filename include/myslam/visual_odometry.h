#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "common_include.h"
#include "backend.h"
#include "frontend.h"
#include "dataset.h"
#include "viewer.h"

namespace myslam {

    class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        VisualOdometry(std::string &config_path) : config_file_path_(config_path) {};

        bool init();

        void run();

        bool step();

        FrontendStatus getFrontendStatus() const { return frontend_->getStatus(); }

    private:
        bool inited_ = false;
        std::string config_file_path_ = nullptr;
        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Dataset::Ptr dataset_ = nullptr;


    };


} // namespace myslam



#endif // MYSLAM_VISUAL_ODOMETRY_H