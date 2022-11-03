#pragma once
#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam {

    class Map;

    class Backend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        Backend();

        // todo 这里是怎么连接到了 Camera 的？
        void setCameras(Camera::Ptr left, Camera::Ptr right){
            cam_left_ = left;
            cam_right_ = right;
        }

        void setMap(std::shared_ptr<Map> map) { map_ = map; }

        void updateMap();

        void stop();

    private:
        void backendLoop();

        void optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_; //frontend 没有设置thread，应该是主线程
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic_bool backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;


    };



} // namespace myslam



#endif // MYSLAM_BACKEND_H