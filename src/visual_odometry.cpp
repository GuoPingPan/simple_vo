#include "../include/myslam/visual_odometry.h"
#include "../include/myslam/config.h"

#include <chrono>

namespace myslam {

    bool VisualOdometry::init() {
        if(Config::setParameterFile(config_file_path_) == false){
            return false;
        }

        dataset_ = Dataset::Ptr(new Dataset(Config::get<std::string>("dataset_dir")));
        CHECK_EQ(dataset_->init(), true);

        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->setViewer(viewer_);
        frontend_->setMap(map_);
        frontend_->setBackEnd(backend_);
        frontend_->setCameras(dataset_->getCamera(0),dataset_->getCamera(1));

        backend_->setCameras(dataset_->getCamera(0),dataset_->getCamera(1));
        backend_->setMap(map_);

        viewer_->setMap(map_);

        return true;

    }

    bool VisualOdometry::step() {
        Frame::Ptr new_frame = dataset_->getNextFrame();
        if(new_frame == nullptr) return false;
        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->addFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";

        return success;
    }

    void VisualOdometry::run() {
        while (1) {
            LOG(INFO) << "VO is running";
            if(step() == false)
                break;
        }
    }



} // namespace myslam